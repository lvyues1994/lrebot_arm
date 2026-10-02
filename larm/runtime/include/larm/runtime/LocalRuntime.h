#pragma once

#include <larm/control/RealtimeRunner.h>
#include <larm/hal/Backend.h>
#include <larm/runtime/RobotSession.h>

#include <memory>

namespace larm::runtime {

struct RuntimeOptions {
    control::RealtimeRunnerConfig realtime;
    std::uint32_t planningThreads = 2;
    // How often the runtime drains control-loop events; bounds completion latency.
    Duration eventPeriod = std::chrono::milliseconds{2};
    // Longest wait for enable, disable and fault reset to take effect.
    Duration stateTimeout = std::chrono::seconds{5};
};

// Starts the control loop on `backend` and serves the session in this process. Destroying the session
// stops the loop; operations still running then complete with MotionFailure::Shutdown.
Expected<std::unique_ptr<RobotSession>>
startLocalRuntime(RobotProfile profile, std::unique_ptr<hal::Backend> backend, RuntimeOptions const &options);

} // namespace larm::runtime
