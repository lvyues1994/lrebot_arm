#include "SessionPanel.h"

#include "Widgets.h"

#include <QFormLayout>
#include <QGridLayout>
#include <QVBoxLayout>

#include <algorithm>

namespace larm::studio {

SessionPanel::SessionPanel(StudioContext const &context_, QWidget *const parent)
    : QWidget{parent}, context{context_} {
    enable = new QPushButton{QStringLiteral("Enable")};
    park = new QPushButton{QStringLiteral("Park")};
    disable = new QPushButton{QStringLiteral("Disable")};
    reset = new QPushButton{QStringLiteral("Reset fault")};
    stop = new QPushButton{QStringLiteral("Stop")};
    emergencyStop = new QPushButton{QStringLiteral("EMERGENCY STOP")};
    emergencyStop->setStyleSheet(QStringLiteral(
        "QPushButton { background: #c62828; color: white; font-weight: bold; padding: 8px; }"));
    speed = makeSpin(0.05, 1.0, 0.05, 2);
    speed->setValue(0.2);
    power = new QLabel;
    safety = new QLabel;
    fault = new QLabel;
    active = new QLabel;

    auto *const buttons = new QGridLayout;
    buttons->addWidget(enable, 0, 0);
    buttons->addWidget(park, 0, 1);
    buttons->addWidget(disable, 0, 2);
    buttons->addWidget(reset, 1, 0);
    buttons->addWidget(stop, 1, 1);
    auto const &profile = context.session->profile();
    auto const arm =
        std::find_if(profile.groups.begin(), profile.groups.end(), [&](JointGroupSpec const &group) {
            return context.session->motion(group.name) != nullptr;
        });
    auto const groupValues = [&](JointVector const &all) {
        auto values = zeroJointVector(arm->joints.size());
        for (std::size_t i = 0; i < arm->joints.size(); ++i) {
            values[idx(i)] = all[idx(arm->joints[i])];
        }
        return values;
    };
    if (arm != profile.groups.end()) {
        armGroup = arm->name;
        gravity = new QPushButton{QStringLiteral("Gravity mode")};
        gravity->setCheckable(true);
        gravity->setToolTip(
            QStringLiteral("Move the arm by hand; turn off (or Stop) to hold it where it is"));
        gravity->setStyleSheet(QStringLiteral("QPushButton:checked { background: #ef6c00; color: white; }"));
        buttons->addWidget(gravity, 2, 0, 1, 3);
        QObject::connect(gravity, &QPushButton::clicked, this,
                         [this](bool const on) { toggleGravityMode(on); });
        if (auto const pose = profile.poses.find("ready"); pose != profile.poses.end()) {
            ready = new QPushButton{QStringLiteral("Ready")};
            buttons->addWidget(ready, 1, 2);
            QObject::connect(ready, &QPushButton::clicked, this, [this, target = groupValues(pose->second)] {
                operations->run(QStringLiteral("ready"), context.session->motion(armGroup)->moveToJoints(
                                                             {.position = target, .speed = speed->value()}));
            });
        }
        QObject::connect(
            park, &QPushButton::clicked, this, [this, rest = groupValues(profile.safety.restPose)] {
                operations->run(QStringLiteral("park"), context.session->motion(armGroup)->moveToJoints(
                                                            {.position = rest, .speed = speed->value()}));
            });
    } else {
        QObject::connect(park, &QPushButton::clicked, this,
                         [this] { operations->run(QStringLiteral("park"), context.session->park()); });
    }
    auto *const status = new QFormLayout;
    status->addRow(QStringLiteral("Speed"), speed);
    status->addRow(QStringLiteral("Power"), power);
    status->addRow(QStringLiteral("Safety"), safety);
    status->addRow(QStringLiteral("Fault"), fault);
    status->addRow(QStringLiteral("Active"), active);
    auto *const layout = new QVBoxLayout{this};
    layout->addWidget(emergencyStop);
    layout->addLayout(buttons);
    layout->addLayout(status);

    operations = std::make_unique<OperationRunner>(this, context, stop);
    auto *const session = context.session;
    QObject::connect(enable, &QPushButton::clicked, this,
                     [this, session] { operations->run(QStringLiteral("enable"), session->enable()); });
    QObject::connect(disable, &QPushButton::clicked, this,
                     [this, session] { operations->run(QStringLiteral("disable"), session->disable({})); });
    QObject::connect(reset, &QPushButton::clicked, this, [this, session] {
        operations->run(QStringLiteral("reset fault"), session->resetFault());
    });
    QObject::connect(emergencyStop, &QPushButton::clicked, this, [this, session] {
        session->emergencyStop();
        context.journal->write(QStringLiteral("emergency stop: requested"));
    });
}

// Gravity mode runs as this panel's operation, so it ends with Stop as well; when it ends for any reason
// the button comes back up.
void SessionPanel::toggleGravityMode(bool const on) {
    if (not on) {
        stop->click();
        return;
    }
    if (operations->busy()) {
        gravity->setChecked(false);
        context.journal->write(QStringLiteral("gravity mode: ignored, another operation is running"));
        return;
    }
    operations->run(QStringLiteral("gravity mode"), context.session->motion(armGroup)->freedrive(),
                    [this](Outcome const &) { gravity->setChecked(false); });
}

void SessionPanel::refresh(control::RobotSnapshot const &snapshot) {
    power->setText(snapshot.power == hal::DrivePower::Enabled ? QStringLiteral("enabled")
                                                              : QStringLiteral("disabled"));
    auto const safe = snapshot.safety == control::SafetyState::Normal;
    safety->setText(safe ? QStringLiteral("normal")
                    : snapshot.safety == control::SafetyState::EmergencyStopped
                        ? QStringLiteral("EMERGENCY STOP")
                        : QStringLiteral("faulted"));
    safety->setStyleSheet(safe ? QString{} : QStringLiteral("color: #c62828; font-weight: bold;"));
    fault->setText(QString::fromUtf8(control::toString(snapshot.fault).data()));
    active->setText(snapshot.activeCount == 0
                        ? QStringLiteral("idle")
                        : QStringLiteral("%1 goal(s) running").arg(snapshot.activeCount));
}

} // namespace larm::studio
