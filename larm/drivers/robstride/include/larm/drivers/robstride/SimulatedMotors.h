#pragma once

#include <larm/drivers/can/CanTransport.h>
#include <larm/drivers/robstride/DriverConfig.h>

#include <functional>
#include <memory>
#include <span>

namespace larm::drivers::robstride {

struct SimulatedMotorsOptions {
    // Simulated time per batch of motion frames; one control period.
    Duration step = std::chrono::milliseconds{4};
    // Rotor plus reflected load inertia at the motor, kg·m².
    double inertia = 0.02;
    double viscousFriction = 0.05;
    // External torque on each motor (motor units) given all motor positions, e.g. gravity of the arm.
    std::function<void(std::span<double const> position, std::span<double> torque)> load;
    // Initial motor positions; zeros when empty.
    std::vector<double> position;
};

// RobStride motors answering on an in-process bus, for tests and rehearsals without hardware. Each
// motor runs the motion law t = kp (p - q) + kd (v - dq) + t_ff while enabled and stays still while
// disabled, as if resting on its support. The bus does not allocate after construction, so a control
// loop may run on it while another thread inspects the motors or injects faults.
struct SimulatedMotors {
    virtual ~SimulatedMotors() = default;
    // The bus endpoint for the driver; valid while this object lives.
    virtual std::unique_ptr<can::CanTransport> transport() = 0;
    virtual double position(std::size_t actuator) const = 0;
    virtual bool enabled(std::size_t actuator) const = 0;
    // Latches a fault: the motor drops to reset mode and sends a fault report.
    virtual void injectFault(std::size_t actuator, std::uint8_t statusFlags, std::uint32_t faults) = 0;
    // A silent motor neither acts on frames nor answers them.
    virtual void setSilent(std::size_t actuator, bool silent) = 0;
    // Run mode the motor reports through parameter 0x7005.
    virtual void setRunMode(std::size_t actuator, std::int8_t mode) = 0;
};

std::unique_ptr<SimulatedMotors> makeSimulatedMotors(DriverConfig const &config,
                                                     SimulatedMotorsOptions options);

// A motor load from joint-space torques carried through the transmissions: `jointTorque(q, out)` fills
// the external torque on each joint at positions q, e.g. the arm's gravity negated. The returned load
// does not allocate.
std::function<void(std::span<double const>, std::span<double>)>
jointSpaceLoad(DriverConfig const &config,
               std::function<void(JointVector const &, JointVector &)> jointTorque);

} // namespace larm::drivers::robstride
