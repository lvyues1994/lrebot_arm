#pragma once

#include "OperationRunner.h"

#include <larm/control/Channels.h>

#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QWidget>

#include <memory>
#include <vector>

namespace larm::studio {

// Joint targets of one group: edit, copy from the arm, move.
struct JointPanel final : QWidget {
    JointPanel(StudioContext const &context_, JointGroupSpec group_, QWidget *parent = nullptr);

    void refresh(control::RobotSnapshot const &snapshot);
    JointVector targets() const;
    void setTargets(JointVector const &values);
    void setSpeed(double value);

    QPushButton *copyButton() const { return copy; }
    QPushButton *moveButton() const { return move; }
    QPushButton *stopButton() const { return stop; }

  private:
    StudioContext context;
    JointGroupSpec group;
    std::vector<QDoubleSpinBox *> spins;
    std::vector<QLabel *> measured;
    QDoubleSpinBox *speed{};
    QPushButton *copy{};
    QPushButton *move{};
    QPushButton *stop{};
    JointVector current;
    std::unique_ptr<OperationRunner> operations;
};

} // namespace larm::studio
