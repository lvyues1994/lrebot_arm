#include "OperationRunner.h"

namespace larm::studio {

QString describe(std::exception_ptr const &error) {
    try {
        std::rethrow_exception(error);
    } catch (runtime::MotionError const &motionError) {
        return QStringLiteral("%1: %2").arg(QString::fromUtf8(runtime::toString(motionError.reason).data()),
                                            QString::fromUtf8(motionError.what()));
    } catch (std::exception const &exception) {
        return QString::fromUtf8(exception.what());
    } catch (...) {
        return QStringLiteral("unknown error");
    }
}

void OperationRunner::finish(QString const &name, Outcome const &outcome,
                             Finished const &onFinished) noexcept {
    running = false;
    stopButton->setEnabled(false);
    auto const kind = outcome.kind == OutcomeKind::Succeeded ? QStringLiteral("succeeded")
                      : outcome.kind == OutcomeKind::Stopped ? QStringLiteral("stopped")
                                                             : QStringLiteral("failed");
    context.journal->write(
        name + QStringLiteral(": ") + kind +
        (outcome.message.isEmpty() ? QString{} : QStringLiteral(" (") + outcome.message + ')'));
    if (onFinished) {
        onFinished(outcome);
    }
}

} // namespace larm::studio
