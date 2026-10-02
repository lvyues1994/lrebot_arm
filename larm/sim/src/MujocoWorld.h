#pragma once

#include <larm/core/Error.h>
#include <larm/core/RobotProfile.h>

#include <mujoco/mujoco.h>

#include <array>
#include <filesystem>
#include <memory>

namespace larm::sim {

struct ModelDeleter {
    void operator()(mjModel *model) const noexcept { mj_deleteModel(model); }
};
struct DataDeleter {
    void operator()(mjData *data) const noexcept { mj_deleteData(data); }
};
using ModelHandle = std::unique_ptr<mjModel, ModelDeleter>;
using DataHandle = std::unique_ptr<mjData, DataDeleter>;

struct JointBinding {
    int qposAddress{};
    int dofAddress{};
    int actuator{};
};

// The loaded model, its data, and where each profile joint lives in them.
struct MujocoWorld {
    ModelHandle model;
    DataHandle data;
    std::array<JointBinding, kMaxDof> joints{};
    std::size_t dof{};
};

Expected<MujocoWorld> loadMujocoWorld(RobotProfile const &profile);

} // namespace larm::sim
