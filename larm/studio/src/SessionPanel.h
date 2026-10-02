#pragma once

#include "OperationRunner.h"

#include <larm/control/Channels.h>

#include <QLabel>
#include <QPushButton>
#include <QWidget>

#include <memory>

namespace larm::studio {

// Power, safety and the emergency stop.
struct SessionPanel final : QWidget {
    explicit SessionPanel(StudioContext const &context_, QWidget *parent = nullptr);

    void refresh(control::RobotSnapshot const &snapshot);

    QPushButton *enableButton() const { return enable; }
    QPushButton *parkButton() const { return park; }
    QPushButton *disableButton() const { return disable; }
    QPushButton *resetButton() const { return reset; }
    QPushButton *emergencyStopButton() const { return emergencyStop; }

  private:
    StudioContext context;
    QPushButton *enable{};
    QPushButton *park{};
    QPushButton *disable{};
    QPushButton *reset{};
    QPushButton *stop{};
    QPushButton *emergencyStop{};
    QLabel *power{};
    QLabel *safety{};
    QLabel *fault{};
    QLabel *active{};
    std::unique_ptr<OperationRunner> operations;
};

} // namespace larm::studio
