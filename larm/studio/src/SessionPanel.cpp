#include "SessionPanel.h"

#include <QFormLayout>
#include <QGridLayout>
#include <QVBoxLayout>

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
    auto *const status = new QFormLayout;
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
    QObject::connect(park, &QPushButton::clicked, this,
                     [this, session] { operations->run(QStringLiteral("park"), session->park()); });
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
