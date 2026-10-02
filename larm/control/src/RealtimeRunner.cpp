#include <larm/control/RealtimeRunner.h>

#include <pthread.h>
#include <sched.h>

#include <cstring>

namespace larm::control {

RealtimeRunner::RealtimeRunner(ControlCycle &cycle, hal::Timeline &timeline,
                               RealtimeRunnerConfig const &config)
    : thread{[this, &cycle, &timeline](std::stop_token const stop) {
          while (not stop.stop_requested()) {
              cycle.tick();
              timeline.advance();
              completed.fetch_add(1, std::memory_order_relaxed);
          }
      }} {
    auto const handle = thread.native_handle();
    if (config.priority) {
        auto parameters = sched_param{};
        parameters.sched_priority = *config.priority;
        if (auto const error = pthread_setschedparam(handle, SCHED_FIFO, &parameters); error != 0) {
            problems.push_back("SCHED_FIFO priority " + std::to_string(*config.priority) +
                               " not applied: " + std::strerror(error));
        }
    }
    if (config.cpu) {
        auto cpus = cpu_set_t{};
        CPU_ZERO(&cpus);
        CPU_SET(static_cast<std::size_t>(*config.cpu), &cpus);
        if (auto const error = pthread_setaffinity_np(handle, sizeof(cpus), &cpus); error != 0) {
            problems.push_back("CPU affinity " + std::to_string(*config.cpu) +
                               " not applied: " + std::strerror(error));
        }
    }
}

RealtimeRunner::~RealtimeRunner() = default;

} // namespace larm::control
