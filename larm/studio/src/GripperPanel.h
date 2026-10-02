#pragma once

#include "OperationRunner.h"

#include <larm/control/Channels.h>

#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QWidget>

#include <memory>

namespace larm::studio {

// A single-joint group as a gripper: open, close, or grip to a width with an effort limit.
struct GripperPanel final : QWidget {
    GripperPanel(StudioContext const &context_, JointGroupSpec group_, QWidget *parent = nullptr);

    void refresh(control::RobotSnapshot const &snapshot);
    void setWidth(double value);

    QPushButton *gripButton() const { return grip; }
    QPushButton *stopButton() const { return stop; }

  private:
    void gripTo(double position);

    StudioContext context;
    JointGroupSpec group;
    JointSpec joint;
    QDoubleSpinBox *width{};
    QDoubleSpinBox *effort{};
    QLabel *measured{};
    QPushButton *open{};
    QPushButton *close{};
    QPushButton *grip{};
    QPushButton *stop{};
    std::unique_ptr<OperationRunner> operations;
};

} // namespace larm::studio
