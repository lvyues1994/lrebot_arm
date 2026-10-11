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
#include <ctime>
#include <string_view>
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
    std::atomic<std::uint64_t> queueFull{0};
    std::atomic<std::uint64_t> errorFrames{0};
    std::atomic<std::uint64_t> unacknowledged{0};
    std::atomic<BusState> busState{BusState::Active};
    std::atomic<bool> interfaceDown{false};

    void add(std::atomic<std::uint64_t> &counter, std::uint64_t const value) noexcept {
        counter.fetch_add(value, std::memory_order_relaxed);
    }
};

bool queueIsFull(int const error) noexcept {
    return error == ENOBUFS or error == EAGAIN or error == EWOULDBLOCK;
}

bool interfaceIsGone(int const error) noexcept {
    return error == ENETDOWN or error == ENODEV or error == ENXIO;
}

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
        // A partial batch means the transmit queue filled up.
        auto full = false;
        auto down = false;
        while (queued < out.size()) {
            auto const count = std::min(kBatch, out.size() - queued);
            for (std::size_t i = 0; i < count; ++i) {
                frames[i] = toKernel(out[queued + i]);
            }
            auto const accepted =
                ::sendmmsg(socket.fd, messages.data(), static_cast<unsigned>(count), MSG_DONTWAIT);
            if (accepted <= 0) {
                auto const error = errno;
                full = queueIsFull(error);
                down = interfaceIsGone(error);
                break;
            }
            queued += static_cast<std::size_t>(accepted);
            if (static_cast<std::size_t>(accepted) < count) {
                full = true;
                break;
            }
        }
        auto const refused = out.size() - queued;
        counters.add(counters.sent, queued);
        counters.add(counters.sendFailures, refused);
        if (full) {
            counters.add(counters.queueFull, refused);
        }
        counters.interfaceDown.store(down, std::memory_order_relaxed);
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
                    record(fromKernel(frame).data, frame.can_id);
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
        auto const nanoseconds = std::max<std::int64_t>(timeout.count(), 0);
        auto const span =
            timespec{.tv_sec = nanoseconds / 1'000'000'000, .tv_nsec = nanoseconds % 1'000'000'000};
        return ::ppoll(&request, 1, &span, nullptr) > 0;
    }

    BusState busState() const noexcept override { return counters.busState.load(std::memory_order_relaxed); }

    TransportStatistics statistics() const noexcept override {
        return TransportStatistics{
            .sent = counters.sent.load(std::memory_order_relaxed),
            .received = counters.received.load(std::memory_order_relaxed),
            .sendFailures = counters.sendFailures.load(std::memory_order_relaxed),
            .queueFull = counters.queueFull.load(std::memory_order_relaxed),
            .errorFrames = counters.errorFrames.load(std::memory_order_relaxed),
            .unacknowledged = counters.unacknowledged.load(std::memory_order_relaxed),
            .busState = counters.busState.load(std::memory_order_relaxed),
            .interfaceDown = counters.interfaceDown.load(std::memory_order_relaxed),
        };
    }

  private:
    void record(std::array<std::uint8_t, 8> const &data, canid_t const id) noexcept {
        counters.add(counters.errorFrames, 1);
        auto const event = decodeErrorFrame(id, data);
        if (event.state) {
            counters.busState.store(*event.state, std::memory_order_relaxed);
        }
        if (event.unacknowledged) {
            counters.add(counters.unacknowledged, 1);
        }
    }

    FileDescriptor socket;
    std::array<can_frame, kBatch> frames{};
    std::array<iovec, kBatch> vectors{};
    std::array<mmsghdr, kBatch> messages{};
    Counters counters;
};

} // namespace

std::string_view toString(BusState const state) noexcept {
    switch (state) {
    case BusState::Active:
        return "active";
    case BusState::Warning:
        return "warning";
    case BusState::Passive:
        return "passive";
    case BusState::Off:
        return "off";
    }
    return "unknown";
}

BusEvent decodeErrorFrame(std::uint32_t const id, std::array<std::uint8_t, 8> const &data) noexcept {
    auto event = BusEvent{.state = std::nullopt, .unacknowledged = (id & CAN_ERR_ACK) != 0};
    if ((id & CAN_ERR_BUSOFF) != 0) {
        event.state = BusState::Off;
    } else if ((id & CAN_ERR_RESTARTED) != 0) {
        event.state = BusState::Active;
    } else if ((id & CAN_ERR_CRTL) != 0) {
        auto const controller = data[1];
        if ((controller & (CAN_ERR_CRTL_RX_PASSIVE | CAN_ERR_CRTL_TX_PASSIVE)) != 0) {
            event.state = BusState::Passive;
        } else if ((controller & (CAN_ERR_CRTL_RX_WARNING | CAN_ERR_CRTL_TX_WARNING)) != 0) {
            event.state = BusState::Warning;
        } else if ((controller & CAN_ERR_CRTL_ACTIVE) != 0) {
            event.state = BusState::Active;
        }
    }
    return event;
}

Expected<std::unique_ptr<CanTransport>> openSocketCan(std::string const &interface,
                                                      SocketCanOptions const &options) {
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
    if (::ioctl(socket.fd, SIOCGIFFLAGS, &request) < 0) {
        return tl::make_unexpected(systemError(context + "not found"));
    }
    if ((request.ifr_flags & IFF_UP) == 0) {
        return makeError(ErrorCode::Io, context + "is down; bring it up, e.g. 'sudo ip link set " +
                                            interface + " up type can bitrate 1000000'");
    }
    if (::ioctl(socket.fd, SIOCGIFTXQLEN, &request) < 0) {
        return tl::make_unexpected(systemError(context + "cannot read the transmit queue length"));
    }
    auto const queue = static_cast<std::size_t>(std::max(request.ifr_qlen, 0));
    if (queue != 0 and queue < options.burst) {
        return makeError(ErrorCode::InvalidConfig, context + "the transmit queue holds " +
                                                       std::to_string(queue) + " frames but " +
                                                       std::to_string(options.burst) +
                                                       " go out at once; raise it, e.g. 'sudo ip link set " +
                                                       interface + " txqueuelen 128'");
    }
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
