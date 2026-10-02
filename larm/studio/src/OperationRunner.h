#pragma once

#include "StudioContext.h"

#include <lqtexec/ObjectScope.h>
#include <lqtexec/WaitSignal.h>

#include <QPushButton>
#include <QString>

#include <cstdint>
#include <exception>
#include <functional>

namespace larm::studio {

enum class OutcomeKind : std::uint8_t { Succeeded, Stopped, Failed };

struct Outcome {
    OutcomeKind kind{};
    QString message;
};

QString describe(std::exception_ptr const &error);

// Runs a panel's session operations one at a time. The panel's stop button races each operation,
// which is then stopped and drained; closing the panel or the application stops it as well. The
// journal records "<name>: started" and "<name>: succeeded|stopped|failed".
struct OperationRunner {
    using Finished = std::function<void(Outcome const &)>;

    OperationRunner(QWidget *owner, StudioContext const &context_, QPushButton *stopButton_)
        : context{context_}, stopButton{stopButton_},
          scope{owner, context_.gui, context_.appScope->get_token()} {
        stopButton->setEnabled(false);
    }

    bool busy() const noexcept { return running; }

    template <class Sender> void run(QString const &name, Sender &&work, Finished onFinished = {}) {
        if (running) {
            context.journal->write(name + QStringLiteral(": ignored, another operation is running"));
            return;
        }
        running = true;
        stopButton->setEnabled(true);
        context.journal->write(name + QStringLiteral(": started"));
        auto settled =
            static_cast<Sender &&>(work) |
            lexec::then([](auto &&...) noexcept { return Outcome{.kind = OutcomeKind::Succeeded}; }) |
            lexec::upon_error([](std::exception_ptr const &error) noexcept {
                return Outcome{.kind = OutcomeKind::Failed, .message = describe(error)};
            }) |
            lexec::upon_stopped([]() noexcept { return Outcome{.kind = OutcomeKind::Stopped}; });
        auto stopped =
            lqtexec::wait_signal(stopButton, &QPushButton::clicked) | lexec::then([](bool) noexcept {
                return Outcome{.kind = OutcomeKind::Stopped, .message = QStringLiteral("by the user")};
            });
        lexec::spawn(
            lexec::when_any(std::move(settled), std::move(stopped)) |
                lqtexec::then_on(
                    scope, [this, name, onFinished = std::move(onFinished)](
                               Outcome const &outcome) noexcept { finish(name, outcome, onFinished); }) |
                lexec::upon_error([](auto const &) noexcept {}) | lexec::upon_stopped([]() noexcept {}),
            scope.get_token());
    }

  private:
    void finish(QString const &name, Outcome const &outcome, Finished const &onFinished) noexcept;

    StudioContext context;
    QPushButton *stopButton;
    bool running{};
    // Last: destroyed first, which stops the running operation.
    lqtexec::ObjectScope scope;
};

} // namespace larm::studio
