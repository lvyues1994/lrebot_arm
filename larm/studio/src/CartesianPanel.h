#pragma once

#include "OperationRunner.h"

#include <larm/control/Channels.h>

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QWidget>

#include <array>
#include <functional>
#include <memory>
#include <optional>

namespace larm::studio {

// The group's TCP pose in its base frame: edit, copy from the arm, move, and jog along or about the
// axes of the base frame or of the TCP, jointwise or along a straight line.
struct CartesianPanel final : QWidget {
    // Receives the target in world coordinates whenever it changes.
    using TargetChanged = std::function<void(std::optional<Pose3> const &)>;

    CartesianPanel(StudioContext const &context_, JointGroupSpec group_, TargetChanged targetChanged_,
                   QWidget *parent = nullptr);

    void refresh(control::RobotSnapshot const &snapshot);
    Pose3 target() const;
    void setTarget(Pose3 const &pose);
    // The TCP's current pose in the base frame, and in the world.
    Pose3 currentPose();
    Pose3 currentPoseInWorld();
    void setJogFrame(runtime::Frame frame);

    QPushButton *copyButton() const { return copy; }
    QPushButton *moveButton() const { return move; }
    QPushButton *stopButton() const { return stop; }
    // Axes 0..2 move along x, y, z; 3..5 turn about them.
    QPushButton *jogButton(std::size_t axis, bool positive) const {
        return jog[2 * axis + (positive ? 1 : 0)];
    }

  private:
    void announceTarget();
    void jogAlong(std::size_t axis, double sign);
    runtime::PathShape pathShape() const;

    StudioContext context;
    JointGroupSpec group;
    TargetChanged targetChanged;
    std::unique_ptr<model::Kinematics> kinematics;
    model::FrameId base;
    model::FrameId tool;
    JointVector current;
    std::array<QDoubleSpinBox *, 6> spins{};
    QComboBox *frame{};
    QComboBox *path{};
    QDoubleSpinBox *step{};
    QDoubleSpinBox *turn{};
    QDoubleSpinBox *speed{};
    QLabel *measured{};
    QPushButton *copy{};
    QPushButton *move{};
    QPushButton *stop{};
    std::array<QPushButton *, 12> jog{};
    std::unique_ptr<OperationRunner> operations;
};

} // namespace larm::studio
