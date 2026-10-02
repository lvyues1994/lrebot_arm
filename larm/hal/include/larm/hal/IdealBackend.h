#pragma once

#include <larm/hal/Backend.h>

#include <memory>

namespace larm::hal {

struct IdealBackendConfig {
    JointVector initialPosition;
    Duration period{};
};

// Joints reach every commanded position, velocity and effort by the next cycle; time advances one
// period per advance(). For tests and dry runs of the control stack.
struct IdealBackend : Backend {
    // While set, read() reports stale feedback.
    virtual void setFeedbackLost(bool lost) noexcept = 0;
    virtual JointCommand const &lastCommand() const noexcept = 0;
    virtual DrivePower lastRequestedPower() const noexcept = 0;
};

std::unique_ptr<IdealBackend> makeIdealBackend(IdealBackendConfig const &config);

} // namespace larm::hal
