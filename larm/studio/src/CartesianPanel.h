#pragma once

#include "OperationRunner.h"

#include <larm/control/Channels.h>

#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QWidget>

#include <array>
#include <functional>
#include <memory>
#include <optional>

namespace larm::studio {

// The group's tool pose in its base frame: edit, copy from the arm, jog along an axis, move.
struct CartesianPanel final : QWidget {
    // Receives the target in world coordinates whenever it changes.
    using TargetChanged = std::function<void(std::optional<Pose3> const &)>;

    CartesianPanel(StudioContext const &context_, JointGroupSpec group_, TargetChanged targetChanged_,
                   QWidget *parent = nullptr);

    void refresh(control::RobotSnapshot const &snapshot);
    Pose3 target() const;
    void setTarget(Pose3 const &pose);
    // The tool's current pose in the base frame.
    Pose3 currentPose();

    QPushButton *copyButton() const { return copy; }
    QPushButton *moveButton() const { return move; }
    QPushButton *stopButton() const { return stop; }
    // Axis 0..2 is x, y, z.
    QPushButton *jogButton(std::size_t axis, bool positive) const {
        return jog[2 * axis + (positive ? 1 : 0)];
    }

  private:
    void announceTarget();
    void jogAlong(std::size_t axis, double sign);

    StudioContext context;
    JointGroupSpec group;
    TargetChanged targetChanged;
    std::unique_ptr<model::Kinematics> kinematics;
    model::FrameId base;
    model::FrameId tool;
    JointVector current;
    std::array<QDoubleSpinBox *, 6> spins{};
    QDoubleSpinBox *step{};
    QDoubleSpinBox *speed{};
    QLabel *measured{};
    QPushButton *copy{};
    QPushButton *move{};
    QPushButton *stop{};
    std::array<QPushButton *, 6> jog{};
    std::unique_ptr<OperationRunner> operations;
};

} // namespace larm::studio
