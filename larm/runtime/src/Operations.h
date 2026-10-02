#pragma once

#include "Coordinator.h"

#include <lexec/execution.hpp>

#include <chrono>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>

namespace larm::runtime::detail {

// Keeps the shared operation state alive across late callbacks; the lexec op itself only starts it.
template <class State> struct SharedOperation {
    using operation_state_concept = lexec::operation_state_t;
    explicit SharedOperation(std::shared_ptr<State> state_) : state{std::move(state_)} {}
    SharedOperation(SharedOperation &&) = delete;
    void start() & noexcept {
        auto const current = state;
        current->start();
    }

  private:
    std::shared_ptr<State> state;
};

// The receiver, taken exactly once by whichever path completes first.
template <class Receiver> struct ReceiverSlot {
    explicit ReceiverSlot(Receiver receiver_) : receiver{std::move(receiver_)} {}

    std::optional<Receiver> take() noexcept {
        auto const lock = std::lock_guard{mutex};
        return std::exchange(receiver, std::nullopt);
    }

  private:
    std::mutex mutex;
    std::optional<Receiver> receiver;
};

template <class Result> struct GoalPlan {
    GoalRequest request;
    std::function<Result(control::RobotSnapshot const &)> result;
};

inline std::exception_ptr failure(GoalOutcome const &outcome) {
    if (outcome.error) {
        return std::make_exception_ptr(*outcome.error);
    }
    auto const reason =
        outcome.fault == control::FaultCode::NotEnabled ? MotionFailure::NotEnabled : MotionFailure::Fault;
    return std::make_exception_ptr(
        MotionError{reason, outcome.fault,
                    "goal ended: " + std::string{control::toString(outcome.status)} + ", " +
                        std::string{control::toString(outcome.fault)}});
}

template <class Result, class Receiver>
struct GoalOperation final : GoalListener, std::enable_shared_from_this<GoalOperation<Result, Receiver>> {
    using Token = lexec::stop_token_of_t<lexec::env_of_t<Receiver>>;
    struct OnStop {
        void operator()() const noexcept {
            if (auto const operation = weak.lock()) {
                operation->requestCancel();
            }
        }
        std::weak_ptr<GoalOperation> weak;
    };
    using StopCallback = lexec::stop_callback_for_t<Token, OnStop>;

    GoalOperation(Coordinator *coordinator_, GoalPlan<Result> plan_, Receiver receiver)
        : coordinator{coordinator_}, plan{std::move(plan_)},
          token{lexec::get_stop_token(lexec::get_env(receiver))}, slot{std::move(receiver)} {}

    void start() noexcept {
        if (token.stop_requested()) {
            if (auto receiver = slot.take()) {
                lexec::set_stopped(std::move(*receiver));
            }
            return;
        }
        stopCallback.emplace(token, OnStop{this->weak_from_this()});
        auto const id = coordinator->submit(std::move(plan.request), this->shared_from_this());
        auto const lock = std::lock_guard{mutex};
        goal = id;
        if (cancelRequested) {
            coordinator->cancel(id);
        }
    }

    void finished(GoalOutcome const &outcome) noexcept override {
        auto receiver = slot.take();
        if (not receiver) {
            return;
        }
        stopCallback.reset();
        if (outcome.status == control::ControlStatus::Succeeded and not outcome.error) {
            try {
                lexec::set_value(std::move(*receiver), plan.result(outcome.snapshot));
            } catch (...) {
                lexec::set_error(std::move(*receiver), std::current_exception());
            }
        } else if (outcome.status == control::ControlStatus::Stopped and
                   outcome.fault == control::FaultCode::None and not outcome.error) {
            lexec::set_stopped(std::move(*receiver));
        } else {
            lexec::set_error(std::move(*receiver), failure(outcome));
        }
    }

  private:
    void requestCancel() noexcept {
        auto const lock = std::lock_guard{mutex};
        if (goal) {
            coordinator->cancel(*goal);
        } else {
            cancelRequested = true;
        }
    }

    Coordinator *coordinator;
    GoalPlan<Result> plan;
    Token token;
    ReceiverSlot<Receiver> slot;
    std::optional<StopCallback> stopCallback;
    std::mutex mutex;
    std::optional<std::uint64_t> goal;
    bool cancelRequested{};
};

template <class Result> struct GoalSender {
    using sender_concept = lexec::sender_t;
    using completion_signatures =
        lexec::completion_signatures<lexec::set_value_t(Result), lexec::set_error_t(std::exception_ptr),
                                     lexec::set_stopped_t()>;

    template <class Receiver> auto connect(Receiver receiver) && {
        using State = GoalOperation<Result, Receiver>;
        return SharedOperation<State>{
            std::make_shared<State>(coordinator, std::move(plan), std::move(receiver))};
    }

    Coordinator *coordinator;
    GoalPlan<Result> plan;
};

struct WaitPlan {
    // Sends whatever request moves the robot toward the state; false when it could not be sent.
    std::function<bool()> begin;
    std::function<WaitVerdict(control::RobotSnapshot const &, std::span<control::ControlEvent const>)> check;
    Duration timeout{};
};

template <class Receiver>
struct WaitOperation final : StateWaiter, std::enable_shared_from_this<WaitOperation<Receiver>> {
    using Token = lexec::stop_token_of_t<lexec::env_of_t<Receiver>>;

    WaitOperation(Coordinator *coordinator_, WaitPlan plan_, Receiver receiver)
        : coordinator{coordinator_}, plan{std::move(plan_)},
          token{lexec::get_stop_token(lexec::get_env(receiver))}, slot{std::move(receiver)} {}

    void start() noexcept {
        deadline = std::chrono::steady_clock::now() + plan.timeout;
        if (not plan.begin()) {
            abandon(MotionError{MotionFailure::Fault, control::FaultCode::None,
                                "the control loop request queue is full"});
            return;
        }
        coordinator->watch(this->shared_from_this());
    }

    bool poll(control::RobotSnapshot const &snapshot,
              std::span<control::ControlEvent const> const events) noexcept override {
        if (token.stop_requested()) {
            if (auto receiver = slot.take()) {
                lexec::set_stopped(std::move(*receiver));
            }
            return true;
        }
        auto const verdict = plan.check(snapshot, events);
        if (std::holds_alternative<Reached>(verdict)) {
            if (auto receiver = slot.take()) {
                lexec::set_value(std::move(*receiver));
            }
            return true;
        }
        if (auto const *const error = std::get_if<MotionError>(&verdict)) {
            abandon(*error);
            return true;
        }
        if (std::chrono::steady_clock::now() > deadline) {
            abandon(MotionError{MotionFailure::Timeout, control::FaultCode::None,
                                "timed out waiting for the robot"});
            return true;
        }
        return false;
    }

    void abandon(MotionError const &error) noexcept override {
        if (auto receiver = slot.take()) {
            lexec::set_error(std::move(*receiver), std::make_exception_ptr(error));
        }
    }

  private:
    Coordinator *coordinator;
    WaitPlan plan;
    Token token;
    ReceiverSlot<Receiver> slot;
    std::chrono::steady_clock::time_point deadline;
};

struct WaitSender {
    using sender_concept = lexec::sender_t;
    using completion_signatures =
        lexec::completion_signatures<lexec::set_value_t(), lexec::set_error_t(std::exception_ptr),
                                     lexec::set_stopped_t()>;

    template <class Receiver> auto connect(Receiver receiver) && {
        using State = WaitOperation<Receiver>;
        return SharedOperation<State>{
            std::make_shared<State>(coordinator, std::move(plan), std::move(receiver))};
    }

    Coordinator *coordinator;
    WaitPlan plan;
};

} // namespace larm::runtime::detail
