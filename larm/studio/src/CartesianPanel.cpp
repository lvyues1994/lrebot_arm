#include "CartesianPanel.h"

#include "Widgets.h"

#include <QFormLayout>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <numbers>

namespace larm::studio {

CartesianPanel::CartesianPanel(StudioContext const &context_, JointGroupSpec group_,
                               TargetChanged targetChanged_, QWidget *const parent)
    : QWidget{parent}, context{context_}, group{std::move(group_)}, targetChanged{std::move(targetChanged_)},
      kinematics{context.model->makeKinematics()}, base{kinematics->findFrame(group.baseFrame).value()},
      tool{kinematics->findFrame(group.toolFrame).value()},
      current{zeroJointVector(context.session->profile().dof())} {
    auto const names =
        std::array<QString, 6>{QStringLiteral("x"),    QStringLiteral("y"),     QStringLiteral("z"),
                               QStringLiteral("roll"), QStringLiteral("pitch"), QStringLiteral("yaw")};
    auto *const form = new QFormLayout;
    for (std::size_t i = 0; i < spins.size(); ++i) {
        spins[i] = i < 3 ? makeSpin(-2.0, 2.0, 0.01, 4, QStringLiteral(" m"))
                         : makeSpin(-std::numbers::pi, std::numbers::pi, 0.05, 3, QStringLiteral(" rad"));
        form->addRow(names[i], spins[i]);
        QObject::connect(spins[i], &QDoubleSpinBox::valueChanged, this, [this] { announceTarget(); });
    }
    measured = new QLabel{QStringLiteral("—")};
    form->addRow(QStringLiteral("Measured"), measured);

    step = makeSpin(0.001, 0.2, 0.005, 3, QStringLiteral(" m"));
    step->setValue(0.02);
    speed = makeSpin(0.05, 1.0, 0.05, 2);
    speed->setValue(0.5);
    auto *const jogGrid = new QGridLayout;
    for (std::size_t axis = 0; axis < 3; ++axis) {
        for (auto const positive : {false, true}) {
            auto *const button =
                new QPushButton{(positive ? QStringLiteral("+") : QStringLiteral("−")) + names[axis]};
            jog[2 * axis + (positive ? 1 : 0)] = button;
            jogGrid->addWidget(button, static_cast<int>(axis), positive ? 1 : 0);
            QObject::connect(button, &QPushButton::clicked, this,
                             [this, axis, positive] { jogAlong(axis, positive ? 1.0 : -1.0); });
        }
    }
    auto *const jogBox = new QHBoxLayout;
    jogBox->addLayout(jogGrid);
    auto *const jogSettings = new QFormLayout;
    jogSettings->addRow(QStringLiteral("Jog step"), step);
    jogSettings->addRow(QStringLiteral("Speed"), speed);
    jogBox->addLayout(jogSettings);

    copy = new QPushButton{QStringLiteral("Copy current")};
    move = new QPushButton{QStringLiteral("Move")};
    stop = new QPushButton{QStringLiteral("Stop")};
    auto *const actions = new QHBoxLayout;
    actions->addStretch();
    actions->addWidget(copy);
    actions->addWidget(move);
    actions->addWidget(stop);

    auto *const layout = new QVBoxLayout{this};
    layout->addLayout(form);
    layout->addLayout(jogBox);
    layout->addLayout(actions);
    layout->addStretch();

    operations = std::make_unique<OperationRunner>(this, context, stop);
    QObject::connect(copy, &QPushButton::clicked, this, [this] { setTarget(currentPose()); });
    QObject::connect(move, &QPushButton::clicked, this, [this] {
        auto *const motion = context.session->motion(group.name);
        operations->run(QStringLiteral("move to pose"),
                        motion->moveToPose({.target = target(), .speed = speed->value()}));
    });
}

void CartesianPanel::refresh(control::RobotSnapshot const &snapshot) {
    current = snapshot.state.joints.position;
    auto const pose = currentPose();
    measured->setText(QStringLiteral("%1, %2, %3 m")
                          .arg(formatJoint(pose.translation.x()), formatJoint(pose.translation.y()),
                               formatJoint(pose.translation.z())));
}

Pose3 CartesianPanel::target() const {
    return Pose3{
        .translation = Eigen::Vector3d{spins[0]->value(), spins[1]->value(), spins[2]->value()},
        .rotation = fromRollPitchYaw(spins[3]->value(), spins[4]->value(), spins[5]->value()),
    };
}

void CartesianPanel::setTarget(Pose3 const &pose) {
    auto const angles = rollPitchYaw(pose.rotation);
    auto const values = std::array<double, 6>{
        pose.translation.x(), pose.translation.y(), pose.translation.z(), angles[0], angles[1], angles[2]};
    for (std::size_t i = 0; i < spins.size(); ++i) {
        auto const blocker = QSignalBlocker{spins[i]};
        spins[i]->setValue(values[i]);
    }
    announceTarget();
}

Pose3 CartesianPanel::currentPose() {
    kinematics->update(current);
    return kinematics->framePose(base).inverse() * kinematics->framePose(tool);
}

void CartesianPanel::announceTarget() {
    kinematics->update(current);
    targetChanged(kinematics->framePose(base) * target());
}

void CartesianPanel::jogAlong(std::size_t const axis, double const sign) {
    auto pose = currentPose();
    pose.translation[static_cast<Eigen::Index>(axis)] += sign * step->value();
    setTarget(pose);
    auto *const motion = context.session->motion(group.name);
    operations->run(QStringLiteral("jog"), motion->moveToPose({.target = pose, .speed = speed->value()}));
}

} // namespace larm::studio
