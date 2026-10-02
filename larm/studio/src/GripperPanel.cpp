#include "GripperPanel.h"

#include "Widgets.h"

#include <QFormLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace larm::studio {

GripperPanel::GripperPanel(StudioContext const &context_, JointGroupSpec group_, QWidget *const parent)
    : QWidget{parent}, context{context_}, group{std::move(group_)},
      joint{context.session->profile().joints[group.joints.front()]} {
    auto const unit = joint.unit == JointUnit::Meter ? QStringLiteral(" m") : QStringLiteral(" rad");
    width = makeSpin(joint.limits.lower, joint.limits.upper, 0.005, 4, unit);
    width->setValue(joint.limits.upper);
    effort = makeSpin(0.5, joint.limits.effort, 0.5, 1);
    effort->setValue(std::min(10.0, joint.limits.effort));
    measured = new QLabel{QStringLiteral("—")};
    auto *const form = new QFormLayout;
    form->addRow(QStringLiteral("Width"), width);
    form->addRow(QStringLiteral("Max effort"), effort);
    form->addRow(QStringLiteral("Measured"), measured);

    open = new QPushButton{QStringLiteral("Open")};
    close = new QPushButton{QStringLiteral("Close")};
    grip = new QPushButton{QStringLiteral("Grip")};
    stop = new QPushButton{QStringLiteral("Stop")};
    auto *const actions = new QHBoxLayout;
    actions->addWidget(open);
    actions->addWidget(close);
    actions->addStretch();
    actions->addWidget(grip);
    actions->addWidget(stop);
    auto *const layout = new QVBoxLayout{this};
    layout->addLayout(form);
    layout->addLayout(actions);
    layout->addStretch();

    operations = std::make_unique<OperationRunner>(this, context, stop);
    QObject::connect(open, &QPushButton::clicked, this, [this] { gripTo(joint.limits.upper); });
    QObject::connect(close, &QPushButton::clicked, this, [this] { gripTo(joint.limits.lower); });
    QObject::connect(grip, &QPushButton::clicked, this, [this] { gripTo(width->value()); });
}

void GripperPanel::refresh(control::RobotSnapshot const &snapshot) {
    measured->setText(formatJoint(snapshot.state.joints.position[idx(group.joints.front())], 4));
}

void GripperPanel::setWidth(double const value) { width->setValue(value); }

void GripperPanel::gripTo(double const position) {
    auto *const gripper = context.session->gripper(group.name);
    operations->run(QStringLiteral("grip"),
                    gripper->grip({.position = position, .maxEffort = effort->value()}));
}

} // namespace larm::studio
