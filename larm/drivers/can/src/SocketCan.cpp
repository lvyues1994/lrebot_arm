#include <larm/drivers/can/CanTransport.h>

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <utility>

namespace larm::drivers::can {
namespace {

constexpr std::size_t kBatch = 32;

struct FileDescriptor {
    explicit FileDescriptor(int const fd_) noexcept : fd{fd_} {}
    FileDescriptor(FileDescriptor &&other) noexcept : fd{std::exchange(other.fd, -1)} {}
    FileDescriptor &operator=(FileDescriptor &&) = delete;
    ~FileDescriptor() {
        if (fd >= 0) {
            ::close(fd);
        }
    }

    int fd;
};

Error systemError(std::string const &what) {
    return Error{.code = ErrorCode::Io, .message = what + ": " + std::strerror(errno)};
}

can_frame toKernel(CanFrame const &frame) noexcept {
    auto out = can_frame{};
    out.can_id = frame.extended ? ((frame.id & CAN_EFF_MASK) | CAN_EFF_FLAG) : (frame.id & CAN_SFF_MASK);
    out.len = std::min<std::uint8_t>(frame.length, CAN_MAX_DLEN);
    std::copy_n(frame.data.begin(), out.len, out.data);
    return out;
}

CanFrame fromKernel(can_frame const &frame) noexcept {
    auto out = CanFrame{};
    out.extended = (frame.can_id & CAN_EFF_FLAG) != 0;
    out.id = frame.can_id & (out.extended ? CAN_EFF_MASK : CAN_SFF_MASK);
    out.length = std::min<std::uint8_t>(frame.len, CAN_MAX_DLEN);
    std::copy_n(frame.data, out.length, out.data.begin());
    return out;
}

// Counters written by the control loop and read by diagnostics.
struct Counters {
    std::atomic<std::uint64_t> sent{0};
    std::atomic<std::uint64_t> received{0};
    std::atomic<std::uint64_t> sendFailures{0};
    std::atomic<std::uint64_t> errorFrames{0};
    std::atomic<bool> busOff{false};

    void add(std::atomic<std::uint64_t> &counter, std::uint64_t const value) noexcept {
        counter.fetch_add(value, std::memory_order_relaxed);
    }
};

struct SocketCan final : CanTransport {
    explicit SocketCan(FileDescriptor socket_) : socket{std::move(socket_)} {
        for (std::size_t i = 0; i < kBatch; ++i) {
            vectors[i] = iovec{.iov_base = &frames[i], .iov_len = sizeof(can_frame)};
            messages[i] = mmsghdr{};
            messages[i].msg_hdr.msg_iov = &vectors[i];
            messages[i].msg_hdr.msg_iovlen = 1;
        }
    }

    std::size_t send(std::span<CanFrame const> const out) noexcept override {
        auto queued = std::size_t{0};
        while (queued < out.size()) {
            auto const count = std::min(kBatch, out.size() - queued);
            for (std::size_t i = 0; i < count; ++i) {
                frames[i] = toKernel(out[queued + i]);
            }
            auto const accepted =
                ::sendmmsg(socket.fd, messages.data(), static_cast<unsigned>(count), MSG_DONTWAIT);
            if (accepted <= 0) {
                break;
            }
            queued += static_cast<std::size_t>(accepted);
            if (static_cast<std::size_t>(accepted) < count) {
                break;
            }
        }
        counters.add(counters.sent, queued);
        counters.add(counters.sendFailures, out.size() - queued);
        return queued;
    }

    std::size_t receive(std::span<CanFrame> const in) noexcept override {
        auto taken = std::size_t{0};
        while (taken < in.size()) {
            auto const count = std::min(kBatch, in.size() - taken);
            auto const arrived =
                ::recvmmsg(socket.fd, messages.data(), static_cast<unsigned>(count), MSG_DONTWAIT, nullptr);
            if (arrived <= 0) {
                break;
            }
            for (int i = 0; i < arrived; ++i) {
                auto const &frame = frames[static_cast<std::size_t>(i)];
                if ((frame.can_id & CAN_ERR_FLAG) != 0) {
                    counters.add(counters.errorFrames, 1);
                    if ((frame.can_id & CAN_ERR_BUSOFF) != 0) {
                        counters.busOff.store(true, std::memory_order_relaxed);
                    }
                    if ((frame.can_id & CAN_ERR_RESTARTED) != 0) {
                        counters.busOff.store(false, std::memory_order_relaxed);
                    }
                    continue;
                }
                if ((frame.can_id & CAN_RTR_FLAG) != 0) {
                    continue;
                }
                in[taken++] = fromKernel(frame);
            }
            if (static_cast<std::size_t>(arrived) < count) {
                break;
            }
        }
        counters.add(counters.received, taken);
        return taken;
    }

    bool waitReadable(Duration const timeout) noexcept override {
        auto request = pollfd{.fd = socket.fd, .events = POLLIN, .revents = 0};
        auto const milliseconds = std::chrono::ceil<std::chrono::milliseconds>(timeout).count();
        return ::poll(&request, 1, static_cast<int>(std::max<std::int64_t>(milliseconds, 0))) > 0;
    }

    TransportStatistics statistics() const noexcept override {
        return TransportStatistics{
            .sent = counters.sent.load(std::memory_order_relaxed),
            .received = counters.received.load(std::memory_order_relaxed),
            .sendFailures = counters.sendFailures.load(std::memory_order_relaxed),
            .errorFrames = counters.errorFrames.load(std::memory_order_relaxed),
            .busOff = counters.busOff.load(std::memory_order_relaxed),
        };
    }

  private:
    FileDescriptor socket;
    std::array<can_frame, kBatch> frames{};
    std::array<iovec, kBatch> vectors{};
    std::array<mmsghdr, kBatch> messages{};
    Counters counters;
};

} // namespace

Expected<std::unique_ptr<CanTransport>> openSocketCan(std::string const &interface) {
    if (interface.empty() or interface.size() >= IFNAMSIZ) {
        return makeError(ErrorCode::InvalidArgument, "invalid CAN interface name '" + interface + "'");
    }
    auto const context = "CAN interface '" + interface + "': ";
    auto socket = FileDescriptor{::socket(PF_CAN, SOCK_RAW, CAN_RAW)};
    if (socket.fd < 0) {
        return tl::make_unexpected(systemError(context + "cannot open a socket"));
    }
    auto request = ifreq{};
    std::copy(interface.begin(), interface.end(), request.ifr_name);
    if (::ioctl(socket.fd, SIOCGIFINDEX, &request) < 0) {
        return tl::make_unexpected(systemError(context + "not found"));
    }
    auto const errors = can_err_mask_t{CAN_ERR_MASK};
    if (::setsockopt(socket.fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &errors, sizeof(errors)) < 0) {
        return tl::make_unexpected(systemError(context + "cannot subscribe to error frames"));
    }
    auto address = sockaddr_can{};
    address.can_family = AF_CAN;
    address.can_ifindex = request.ifr_ifindex;
    if (::bind(socket.fd, reinterpret_cast<sockaddr const *>(&address), sizeof(address)) < 0) {
        return tl::make_unexpected(systemError(context + "cannot bind"));
    }
    return std::unique_ptr<CanTransport>{std::make_unique<SocketCan>(std::move(socket))};
}

} // namespace larm::drivers::can
