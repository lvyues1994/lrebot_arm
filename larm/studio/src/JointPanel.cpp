#include "JointPanel.h"

#include "Widgets.h"

#include <QGridLayout>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace larm::studio {

JointPanel::JointPanel(StudioContext const &context_, JointGroupSpec group_, QWidget *const parent)
    : QWidget{parent}, context{context_}, group{std::move(group_)},
      current{zeroJointVector(group.joints.size())} {
    auto const &profile = context.session->profile();
    auto *const grid = new QGridLayout;
    grid->addWidget(new QLabel{QStringLiteral("Joint")}, 0, 0);
    grid->addWidget(new QLabel{QStringLiteral("Target")}, 0, 1);
    grid->addWidget(new QLabel{QStringLiteral("Measured")}, 0, 2);
    for (std::size_t i = 0; i < group.joints.size(); ++i) {
        auto const &joint = profile.joints[group.joints[i]];
        auto *const spin =
            makeSpin(joint.limits.lower, joint.limits.upper, 0.05, 3,
                     joint.unit == JointUnit::Meter ? QStringLiteral(" m") : QStringLiteral(" rad"));
        auto *const value = new QLabel{QStringLiteral("—")};
        auto const row = static_cast<int>(i) + 1;
        grid->addWidget(new QLabel{QString::fromStdString(joint.name)}, row, 0);
        grid->addWidget(spin, row, 1);
        grid->addWidget(value, row, 2);
        spins.push_back(spin);
        measured.push_back(value);
    }
    speed = makeSpin(0.05, 1.0, 0.05, 2);
    speed->setValue(0.5);
    copy = new QPushButton{QStringLiteral("Copy current")};
    move = new QPushButton{QStringLiteral("Move")};
    stop = new QPushButton{QStringLiteral("Stop")};
    auto *const actions = new QHBoxLayout;
    actions->addWidget(new QLabel{QStringLiteral("Speed")});
    actions->addWidget(speed);
    actions->addStretch();
    actions->addWidget(copy);
    actions->addWidget(move);
    actions->addWidget(stop);
    auto *const layout = new QVBoxLayout{this};
    layout->addLayout(grid);
    layout->addLayout(actions);
    layout->addStretch();

    operations = std::make_unique<OperationRunner>(this, context, stop);
    QObject::connect(copy, &QPushButton::clicked, this, [this] { setTargets(current); });
    QObject::connect(move, &QPushButton::clicked, this, [this] {
        auto *const motion = context.session->motion(group.name);
        operations->run(QStringLiteral("move joints"),
                        motion->moveToJoints({.position = targets(), .speed = speed->value()}));
    });
}

void JointPanel::refresh(control::RobotSnapshot const &snapshot) {
    for (std::size_t i = 0; i < group.joints.size(); ++i) {
        current[idx(i)] = snapshot.state.joints.position[idx(group.joints[i])];
        measured[i]->setText(formatJoint(current[idx(i)]));
    }
}

JointVector JointPanel::targets() const {
    auto values = zeroJointVector(spins.size());
    for (std::size_t i = 0; i < spins.size(); ++i) {
        values[idx(i)] = spins[i]->value();
    }
    return values;
}

void JointPanel::setTargets(JointVector const &values) {
    for (std::size_t i = 0; i < spins.size() and i < dofOf(values); ++i) {
        spins[i]->setValue(values[idx(i)]);
    }
}

void JointPanel::setSpeed(double const value) { speed->setValue(value); }

} // namespace larm::studio
