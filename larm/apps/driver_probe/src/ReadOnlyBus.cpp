#include "ReadOnlyBus.h"

#include <array>

namespace larm::probe {
namespace {

using drivers::can::CanFrame;
using drivers::robstride::decodeParameter;
using drivers::robstride::decodePing;
using drivers::robstride::encodePing;
using drivers::robstride::encodeReadParameter;
using drivers::robstride::FrameType;
using drivers::robstride::headerOf;
using drivers::robstride::ParameterValue;

constexpr auto kReplyTimeout = std::chrono::milliseconds{50};

} // namespace

ReadOnlyBus::ReadOnlyBus(DriverConfig config_, std::unique_ptr<drivers::can::CanTransport> transport_)
    : config{std::move(config_)}, transport{std::move(transport_)} {}

template <class Matches>
std::optional<CanFrame> ReadOnlyBus::request(CanFrame const &frame, Matches matches) {
    if (transport->send({&frame, 1}) != 1) {
        return std::nullopt;
    }
    auto received = std::array<CanFrame, 32>{};
    auto const deadline = std::chrono::steady_clock::now() + kReplyTimeout;
    for (auto now = std::chrono::steady_clock::now(); now < deadline;
         now = std::chrono::steady_clock::now()) {
        if (not transport->waitReadable(deadline - now)) {
            continue;
        }
        auto const count = transport->receive(received);
        auto found = std::optional<CanFrame>{};
        for (std::size_t i = 0; i < count; ++i) {
            if (not found and matches(received[i])) {
                found = received[i];
            } else if (auto const header = headerOf(received[i]);
                       header and
                       (header->type == FrameType::Status or header->type == FrameType::ActiveReport)) {
                ++unsolicited;
            }
        }
        if (found) {
            return found;
        }
    }
    return std::nullopt;
}

bool ReadOnlyBus::ping(ActuatorConfig const &actuator) {
    auto const id = actuator.id;
    return request(encodePing(id, config.hostId),
                   [id](CanFrame const &frame) {
                       auto const reply = decodePing(frame);
                       return reply and reply->motor == id;
                   })
        .has_value();
}

std::optional<ParameterValue> ReadOnlyBus::read(ActuatorConfig const &actuator, std::uint16_t const index) {
    auto const id = actuator.id;
    auto const reply =
        request(encodeReadParameter(id, config.hostId, index), [id, index](CanFrame const &frame) {
            auto const value = decodeParameter(frame);
            return value and value->motor == id and value->index == index;
        });
    auto const value = reply ? decodeParameter(*reply) : std::nullopt;
    return value and value->ok ? value : std::nullopt;
}

} // namespace larm::probe
