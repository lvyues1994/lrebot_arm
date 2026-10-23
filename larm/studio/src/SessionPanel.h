#pragma once

#include "OperationRunner.h"

#include <larm/control/Channels.h>

#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QWidget>

#include <memory>
#include <string>

namespace larm::studio {

// Power, safety, the emergency stop, the rest and ready poses, and gravity mode. Ready and park move the
// arm group at the panel's speed; the gripper stays as it is.
struct SessionPanel final : QWidget {
    explicit SessionPanel(StudioContext const &context_, QWidget *parent = nullptr);

    void refresh(control::RobotSnapshot const &snapshot);

    QPushButton *enableButton() const { return enable; }
    QPushButton *parkButton() const { return park; }
    // Null when the profile has no "ready" pose.
    QPushButton *readyButton() const { return ready; }
    // Checkable; null when the profile has no group with a tool frame.
    QPushButton *gravityButton() const { return gravity; }
    QPushButton *disableButton() const { return disable; }
    QPushButton *resetButton() const { return reset; }
    QPushButton *emergencyStopButton() const { return emergencyStop; }

  private:
    void toggleGravityMode(bool on);

    StudioContext context;
    QPushButton *enable{};
    QPushButton *park{};
    QPushButton *ready{};
    QPushButton *gravity{};
    QPushButton *disable{};
    QPushButton *reset{};
    QPushButton *stop{};
    QPushButton *emergencyStop{};
    QDoubleSpinBox *speed{};
    // The group with a tool frame, which ready, park and gravity mode move; empty when there is none.
    std::string armGroup;
    QLabel *power{};
    QLabel *safety{};
    QLabel *fault{};
    QLabel *active{};
    std::unique_ptr<OperationRunner> operations;
};

} // namespace larm::studio
