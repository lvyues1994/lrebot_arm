#pragma once

#include <larm/core/RobotProfile.h>

#include <mujoco/mujoco.h>

#include <memory>

namespace larm::sim {

// Poses the profile's MJCF from joint positions without simulating, for viewers. Joints coupled by
// joint equality constraints (a mimic finger) follow their leaders.
struct SceneMirror {
    virtual ~SceneMirror() = default;
    virtual mjModel const &model() const noexcept = 0;
    // Mutable because MuJoCo's visualizer takes mjData by pointer.
    virtual mjData &data() noexcept = 0;
    virtual void show(JointVector const &position) noexcept = 0;
};

Expected<std::unique_ptr<SceneMirror>> loadSceneMirror(RobotProfile const &profile);

} // namespace larm::sim
