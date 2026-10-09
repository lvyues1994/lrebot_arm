#include "Coordinator.h"

#include <algorithm>

namespace larm::runtime::detail {
namespace {

MotionError shutdownError() {
    return MotionError{MotionFailure::Shutdown, control::FaultCode::None, "the runtime is shutting down"};
}

} // namespace

Coordinator::Coordinator(control::RuntimeChannels &channels_, Duration const eventPeriod_)
    : channels{channels_}, eventPeriod{eventPeriod_}, snapshot{channels_.snapshot.current()},
      thread{[this](std::stop_token const stop) { run(stop); }} {}

Coordinator::~Coordinator() { shutdown(); }

std::uint64_t Coordinator::submit(GoalRequest request, std::shared_ptr<GoalListener> listener) {
    auto lock = std::unique_lock{mutex};
    auto const goal = ++nextGoal;
    if (stopped) {
        lock.unlock();
        listener->finished(GoalOutcome{.status = control::ControlStatus::Failed, .error = shutdownError()});
        return goal;
    }
    for (auto it = pending.begin(); it != pending.end();) {
        auto &[id, other] = *it;
        if ((other.joints & request.joints).none()) {
            ++it;
            continue;
        }
        if (other.activated) {
            if (not other.cancelRequested) {
                other.cancelRequested = send(control::CancelGoal{.goal = control::GoalId{id}});
            }
            ++it;
        } else {
            outbox.push_back(
                Delivery{.listener = std::move(other.listener),
                         .outcome = {.status = control::ControlStatus::Stopped, .snapshot = snapshot}});
            it = pending.erase(it);
        }
    }
    auto &entry = pending[goal] = Pending{.listener = std::move(listener),
                                          .joints = request.joints,
                                          .makeController = std::move(request.makeController)};
    if (not overlapsActive(entry.joints)) {
        activate(goal, entry);
        if (not entry.listener) {
            pending.erase(goal);
        }
    }
    return goal;
}

void Coordinator::cancel(std::uint64_t const goal) {
    auto const lock = std::lock_guard{mutex};
    auto const found = pending.find(goal);
    if (found == pending.end()) {
        return;
    }
    auto &entry = found->second;
    if (entry.activated) {
        if (not entry.cancelRequested) {
            entry.cancelRequested = send(control::CancelGoal{.goal = control::GoalId{goal}});
        }
        return;
    }
    outbox.push_back(Delivery{.listener = std::move(entry.listener),
                              .outcome = {.status = control::ControlStatus::Stopped, .snapshot = snapshot}});
    pending.erase(found);
}

void Coordinator::watch(std::shared_ptr<StateWaiter> waiter) {
    auto lock = std::unique_lock{mutex};
    if (stopped) {
        lock.unlock();
        waiter->abandon(shutdownError());
        return;
    }
    waiters.push_back(std::move(waiter));
}

bool Coordinator::send(control::ControlRequest request) {
    auto const lock = std::lock_guard{producer};
    return channels.requests.tryPush(std::move(request));
}

control::RobotSnapshot Coordinator::latest() const {
    auto const lock = std::lock_guard{mutex};
    return snapshot;
}

void Coordinator::shutdown() {
    if (thread.joinable()) {
        thread.request_stop();
        thread.join();
    }
    auto remaining = std::map<std::uint64_t, Pending>{};
    auto watching = std::vector<std::shared_ptr<StateWaiter>>{};
    auto undelivered = std::vector<Delivery>{};
    {
        auto const lock = std::lock_guard{mutex};
        stopped = true;
        remaining.swap(pending);
        watching.swap(waiters);
        undelivered.swap(outbox);
    }
    for (auto &delivery : undelivered) {
        delivery.listener->finished(delivery.outcome);
    }
    for (auto &[goal, entry] : remaining) {
        entry.listener->finished(
            GoalOutcome{.status = control::ControlStatus::Failed, .error = shutdownError()});
    }
    for (auto const &waiter : watching) {
        waiter->abandon(shutdownError());
    }
}

void Coordinator::run(std::stop_token const &stop) {
    auto events = std::vector<control::ControlEvent>{};
    while (not stop.stop_requested()) {
        events.clear();
        while (auto event = channels.events.tryPop()) {
            events.push_back(std::move(*event));
        }
        while (channels.retired.tryPop()) {
        }
        channels.snapshot.refresh();

        auto deliveries = std::vector<Delivery>{};
        auto watching = std::vector<std::shared_ptr<StateWaiter>>{};
        {
            auto const lock = std::lock_guard{mutex};
            snapshot = channels.snapshot.current();
            for (auto const &event : events) {
                if (auto const *const finished = std::get_if<control::GoalFinished>(&event)) {
                    conclude(finished->goal.value, GoalOutcome{.status = finished->status,
                                                               .fault = finished->fault,
                                                               .snapshot = snapshot});
                }
            }
            activateReady();
            deliveries.swap(outbox);
            watching = waiters;
        }
        for (auto &delivery : deliveries) {
            delivery.listener->finished(delivery.outcome);
        }
        auto done = std::vector<StateWaiter *>{};
        auto const current = latest();
        for (auto const &waiter : watching) {
            if (waiter->poll(current, events)) {
                done.push_back(waiter.get());
            }
        }
        if (not done.empty()) {
            auto const lock = std::lock_guard{mutex};
            std::erase_if(waiters, [&](std::shared_ptr<StateWaiter> const &waiter) {
                return std::find(done.begin(), done.end(), waiter.get()) != done.end();
            });
        }
        std::this_thread::sleep_for(eventPeriod);
    }
}

void Coordinator::activateReady() {
    for (auto &[goal, entry] : pending) {
        if (not entry.activated and not overlapsActive(entry.joints)) {
            activate(goal, entry);
        }
    }
    std::erase_if(pending, [](auto const &item) { return not item.second.listener; });
}

void Coordinator::activate(std::uint64_t const goal, Pending &entry) {
    auto controller = entry.makeController(snapshot);
    entry.makeController = {};
    if (not controller) {
        outbox.push_back(Delivery{.listener = std::move(entry.listener),
                                  .outcome = {.status = control::ControlStatus::Failed,
                                              .error = std::move(controller.error()),
                                              .snapshot = snapshot}});
        return;
    }
    if (not send(control::ActivateController{.goal = control::GoalId{goal},
                                             .controller = std::move(*controller)})) {
        auto error = MotionError{MotionFailure::Fault, control::FaultCode::None,
                                 "the control loop request queue is full"};
        outbox.push_back(Delivery{.listener = std::move(entry.listener),
                                  .outcome = {.status = control::ControlStatus::Failed,
                                              .error = std::move(error),
                                              .snapshot = snapshot}});
        return;
    }
    entry.activated = true;
}

void Coordinator::conclude(std::uint64_t const goal, GoalOutcome outcome) {
    auto const found = pending.find(goal);
    if (found == pending.end()) {
        return;
    }
    outbox.push_back(Delivery{.listener = std::move(found->second.listener), .outcome = std::move(outcome)});
    pending.erase(found);
}

bool Coordinator::overlapsActive(JointMask const &joints) const {
    return std::any_of(pending.begin(), pending.end(), [&](auto const &item) {
        return item.second.activated and (item.second.joints & joints).any();
    });
}

} // namespace larm::runtime::detail
