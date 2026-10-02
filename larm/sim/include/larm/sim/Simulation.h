#pragma once

#include <larm/core/RobotProfile.h>
#include <larm/hal/Backend.h>

#include <mujoco/mujoco.h>

#include <cstdint>
#include <memory>

namespace larm::sim {

enum class Pacing : std::uint8_t { AsFastAsPossible, RealTime };

struct SimulationOptions {
    Pacing pacing = Pacing::AsFastAsPossible;
    // Simulated seconds per wall-clock second when pacing in real time.
    double realTimeFactor = 1.0;
};

// MuJoCo backend. The control loop drives it like hardware; each advance() integrates one control
// period of physics. Each profile joint maps to the MJCF joint named by its description joint and to
// a torque actuator named after the profile joint.
struct Simulation : hal::Backend {
    // Not for the control loop: sets joint positions, zeroes velocities and recomputes derived state.
    virtual void reset(JointVector const &position) = 0;
    virtual mjModel const &model() const noexcept = 0;
    virtual mjData const &data() const noexcept = 0;
    virtual Duration physicsStep() const noexcept = 0;
};

// Reads `sim.physics_step_us` from the profile; the control period must be a whole number of steps.
Expected<std::unique_ptr<Simulation>> makeSimulation(RobotProfile const &profile,
                                                     SimulationOptions const &options);

} // namespace larm::sim
