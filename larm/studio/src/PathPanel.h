#pragma once

#include "OperationRunner.h"

#include <larm/control/Channels.h>

#include <QDoubleSpinBox>
#include <QListWidget>
#include <QPushButton>
#include <QWidget>

#include <functional>
#include <memory>
#include <vector>

namespace larm::studio {

// A list of joint waypoints run as one timed path, a fixed time apart.
struct PathPanel final : QWidget {
    using JointSource = std::function<JointVector()>;

    PathPanel(StudioContext const &context_, JointGroupSpec group_, JointSource jointTargets_,
              QWidget *parent = nullptr);

    void refresh(control::RobotSnapshot const &snapshot);
    std::size_t waypointCount() const { return waypoints.size(); }

    QPushButton *addCurrentButton() const { return addCurrent; }
    QPushButton *addTargetsButton() const { return addTargets; }
    QPushButton *clearButton() const { return clear; }
    QPushButton *runButton() const { return run; }

  private:
    void append(JointVector const &waypoint);

    StudioContext context;
    JointGroupSpec group;
    JointSource jointTargets;
    JointVector current;
    std::vector<JointVector> waypoints;
    QListWidget *list{};
    QDoubleSpinBox *segment{};
    QPushButton *addCurrent{};
    QPushButton *addTargets{};
    QPushButton *remove{};
    QPushButton *clear{};
    QPushButton *run{};
    QPushButton *stop{};
    std::unique_ptr<OperationRunner> operations;
};

} // namespace larm::studio
