#include "Script.h"

#include <QCoreApplication>
#include <QDebug>
#include <QPainter>
#include <QPixmap>

namespace larm::studio {
namespace {

constexpr qint64 kTimeoutMs = 180'000;

bool seen(QStringList const &lines, QString const &text) {
    return std::any_of(lines.begin(), lines.end(), [&](QString const &line) { return line.contains(text); });
}

auto after(QString text) {
    return [text = std::move(text)](QStringList const &lines) { return seen(lines, text); };
}

JointVector armPose(double const base) {
    auto q = JointVector{6};
    q << base, 1.0, 1.4, -0.5, 0.4, 1.0;
    return q;
}

} // namespace

Script::Script(MainWindow &window_, runtime::RobotSession &session_, Journal &journal_,
               std::optional<QDir> screenshots_, std::function<void()> quit_)
    : window{window_}, session{session_}, journal{journal_}, screenshots{std::move(screenshots_)},
      quit{std::move(quit_)} {
    auto *const sessionPanel = window.sessionPanel();
    auto *const joints = window.jointPanel();
    auto *const cartesian = window.cartesianPanel();
    auto *const gripper = window.gripperPanel();
    auto *const path = window.pathPanel();
    if (joints == nullptr or gripper == nullptr or sessionPanel->readyButton() == nullptr) {
        journal.write(
            QStringLiteral("script: the profile needs an arm group, a gripper group and a ready pose"));
        failed = true;
        return;
    }

    steps.push_back({QStringLiteral("enable"),
                     [this](QStringList const &) { return session.latest().state.cycle > 0; },
                     [sessionPanel] { sessionPanel->enableButton()->click(); }});
    steps.push_back({QStringLiteral("go to the ready pose"), after(QStringLiteral("enable: succeeded")),
                     [sessionPanel] { sessionPanel->readyButton()->click(); }});
    steps.push_back(
        {QStringLiteral("move to joint targets"), after(QStringLiteral("ready: succeeded")), [joints] {
             joints->setTargets(armPose(0.6));
             joints->setSpeed(0.6);
             joints->moveButton()->click();
         }});
    steps.push_back({QStringLiteral("move to a pose 5 cm lower"),
                     after(QStringLiteral("move joints: succeeded")), [this, cartesian] {
                         capture(QStringLiteral("1-joints"));
                         window.showPanel(cartesian);
                         cartesian->copyButton()->click();
                         auto pose = cartesian->target();
                         pose.translation.z() -= 0.05;
                         cartesian->setTarget(pose);
                         cartesian->moveButton()->click();
                     }});
    steps.push_back(
        {QStringLiteral("jog along +x"), after(QStringLiteral("move to pose: succeeded")), [this, cartesian] {
             capture(QStringLiteral("2-cartesian"));
             cartesian->jogButton(0, true)->click();
         }});
    steps.push_back({QStringLiteral("back off 2 cm along the tool's x axis, straight"),
                     after(QStringLiteral("jog: succeeded")), [cartesian] {
                         cartesian->setJogFrame(runtime::Frame::Tool);
                         cartesian->jogButton(0, false)->click();
                     }});
    steps.push_back({QStringLiteral("turn 10° about the tool's x axis"),
                     after(QStringLiteral("jog: succeeded")),
                     [cartesian] { cartesian->jogButton(3, true)->click(); }});
    steps.push_back({QStringLiteral("grip"), after(QStringLiteral("jog: succeeded")), [this, gripper] {
                         window.showPanel(gripper);
                         gripper->setWidth(0.035);
                         gripper->gripButton()->click();
                     }});
    steps.push_back({QStringLiteral("run a two-waypoint path"), after(QStringLiteral("grip: succeeded")),
                     [this, joints, path] {
                         capture(QStringLiteral("3-gripper"));
                         window.showPanel(path);
                         path->clearButton()->click();
                         joints->setTargets(armPose(0.2));
                         path->addTargetsButton()->click();
                         joints->setTargets(armPose(-0.2));
                         path->addTargetsButton()->click();
                         path->runButton()->click();
                     }});
    steps.push_back({QStringLiteral("start a long, slow move"),
                     after(QStringLiteral("follow path: succeeded")), [this, joints] {
                         capture(QStringLiteral("4-path"));
                         window.showPanel(joints);
                         joints->setTargets(armPose(2.0));
                         joints->setSpeed(0.2);
                         joints->moveButton()->click();
                     }});
    steps.push_back({QStringLiteral("stop it half way"),
                     [this](QStringList const &lines) {
                         return seen(lines, QStringLiteral("move joints: started")) and
                                sinceAct.elapsed() > 800;
                     },
                     [joints] { joints->stopButton()->click(); }});
    steps.push_back({QStringLiteral("emergency stop"), after(QStringLiteral("move joints: stopped")),
                     [sessionPanel] { sessionPanel->emergencyStopButton()->click(); }});
    steps.push_back({QStringLiteral("reset the fault"),
                     [this](QStringList const &) {
                         return session.latest().safety == control::SafetyState::EmergencyStopped;
                     },
                     [this, sessionPanel] {
                         capture(QStringLiteral("5-emergency-stop"));
                         sessionPanel->resetButton()->click();
                     }});
    steps.push_back({QStringLiteral("park"), after(QStringLiteral("reset fault: succeeded")),
                     [sessionPanel] { sessionPanel->parkButton()->click(); }});
    steps.push_back({QStringLiteral("disable"), after(QStringLiteral("park: succeeded")),
                     [sessionPanel] { sessionPanel->disableButton()->click(); }});
    steps.push_back({QStringLiteral("quit"), after(QStringLiteral("disable: succeeded")), [this] {
                         capture(QStringLiteral("6-parked"));
                         quit();
                     }});
    QObject::connect(&timer, &QTimer::timeout, &timer, [this] { tick(); });
}

void Script::start() {
    total.start();
    sinceAct.start();
    timer.start(50);
}

void Script::tick() {
    if (not failed and next < steps.size() and total.elapsed() > kTimeoutMs) {
        failed = true;
        journal.write(QStringLiteral("script: timed out waiting to %1").arg(steps[next].name));
    }
    if (failed or next == steps.size()) {
        timer.stop();
        if (failed) {
            quit();
        }
        return;
    }
    auto &step = steps[next];
    if (not step.ready(journal.since(mark))) {
        return;
    }
    journal.write(QStringLiteral("script: %1").arg(step.name));
    mark = journal.size();
    sinceAct.restart();
    ++next;
    step.act();
}

void Script::capture(QString const &name) {
    if (not screenshots) {
        return;
    }
    window.refresh();
    QCoreApplication::sendPostedEvents();
    auto image = window.grab().toImage();
    if (auto *const view = window.sceneView()) {
        auto painter = QPainter{&image};
        auto const *const host = window.viewport();
        painter.drawImage(QRect{host->mapTo(&window, QPoint{0, 0}), host->size()}, view->grabFramebuffer());
    }
    image.save(screenshots->filePath(name + QStringLiteral(".png")));
}

bool Script::verify() const {
    auto const failures = journal.since(0).filter(QStringLiteral(": failed"));
    for (auto const &failure : failures) {
        qWarning().noquote() << "script: an operation failed:" << failure;
    }
    auto const complete = next == steps.size() and not failed;
    qInfo().noquote() << (complete and failures.isEmpty() ? "script: passed" : "script: FAILED");
    return complete and failures.isEmpty();
}

} // namespace larm::studio
