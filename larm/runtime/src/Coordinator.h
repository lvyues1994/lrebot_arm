#pragma once

#include <larm/control/Channels.h>
#include <larm/runtime/RobotSession.h>

#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <thread>
#include <variant>
#include <vector>

namespace larm::runtime::detail {

struct GoalOutcome {
    control::ControlStatus status{};
    control::FaultCode fault{};
    // Set when the runtime failed the goal itself, e.g. it could not be planned at activation.
    std::optional<MotionError> error;
    control::RobotSnapshot snapshot;
};

// Implemented by the operation awaiting a goal; called once, on the coordinator thread.
struct GoalListener {
    virtual ~GoalListener() = default;
    virtual void finished(GoalOutcome const &outcome) noexcept = 0;
};

// Builds the controller from the robot state at activation, which a preempted goal delays. Its error
// fails the goal.
using ControllerFactory = std::function<tl::expected<std::unique_ptr<control::Controller>, MotionError>(
    control::RobotSnapshot const &)>;

struct GoalRequest {
    JointMask joints;
    ControllerFactory makeController;
};

struct Waiting {};
struct Reached {};
using WaitVerdict = std::variant<Waiting, Reached, MotionError>;

// Implemented by operations awaiting a robot state. Polled on the coordinator thread after every
// refresh until it returns true.
struct StateWaiter {
    virtual ~StateWaiter() = default;
    virtual bool poll(control::RobotSnapshot const &snapshot,
                      std::span<control::ControlEvent const> events) noexcept = 0;
    virtual void abandon(MotionError const &error) noexcept = 0;
};

// The non-real-time side of the control loop channels. One thread drains loop events, keeps the
// latest snapshot, activates goals once the joints they need are free, and delivers completions.
struct Coordinator {
    Coordinator(control::RuntimeChannels &channels, Duration eventPeriod);
    ~Coordinator();
    Coordinator(Coordinator const &) = delete;
    Coordinator &operator=(Coordinator const &) = delete;

    // Running goals on overlapping joints are cancelled first; queued ones are superseded.
    std::uint64_t submit(GoalRequest request, std::shared_ptr<GoalListener> listener);
    void cancel(std::uint64_t goal);
    void watch(std::shared_ptr<StateWaiter> waiter);
    // Thread-safe producer for the control loop's request ring; false when it is full.
    bool send(control::ControlRequest request);
    control::RobotSnapshot latest() const;
    // Stops the thread and fails everything still pending with MotionFailure::Shutdown.
    void shutdown();

  private:
    struct Pending {
        std::shared_ptr<GoalListener> listener;
        JointMask joints;
        ControllerFactory makeController;
        bool activated{};
        bool cancelRequested{};
    };
    struct Delivery {
        std::shared_ptr<GoalListener> listener;
        GoalOutcome outcome;
    };

    void run(std::stop_token const &stop);
    void activateReady();
    void activate(std::uint64_t goal, Pending &pending);
    void conclude(std::uint64_t goal, GoalOutcome outcome);
    bool overlapsActive(JointMask const &joints) const;

    control::RuntimeChannels &channels;
    Duration eventPeriod;
    mutable std::mutex mutex;
    std::mutex producer;
    std::uint64_t nextGoal{};
    std::map<std::uint64_t, Pending> pending;
    std::vector<Delivery> outbox;
    std::vector<std::shared_ptr<StateWaiter>> waiters;
    control::RobotSnapshot snapshot;
    bool stopped{};
    std::jthread thread;
};

} // namespace larm::runtime::detail
