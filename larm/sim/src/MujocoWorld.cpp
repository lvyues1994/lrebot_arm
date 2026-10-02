#include "MujocoWorld.h"

#include <array>
#include <string>

namespace larm::sim {
namespace {

Expected<ModelHandle> loadModel(std::filesystem::path const &mjcf) {
    if (mj_version() != mjVERSION_HEADER) {
        return makeError(ErrorCode::Unsupported, "MuJoCo library version " + std::to_string(mj_version()) +
                                                     " does not match headers " +
                                                     std::to_string(mjVERSION_HEADER));
    }
    auto error = std::array<char, 1024>{};
    auto model = ModelHandle{mj_loadXML(mjcf.c_str(), nullptr, error.data(), static_cast<int>(error.size()))};
    if (not model) {
        return makeError(ErrorCode::InvalidConfig, "cannot load MJCF " + mjcf.string() + ": " + error.data());
    }
    return model;
}

Expected<JointBinding> bindJoint(mjModel const &model, JointSpec const &joint) {
    auto const jointId = mj_name2id(&model, mjOBJ_JOINT, joint.descriptionJoint.c_str());
    if (jointId < 0) {
        return makeError(ErrorCode::InvalidConfig, "MJCF has no joint '" + joint.descriptionJoint + "'");
    }
    auto const type = model.jnt_type[jointId];
    if (type != mjJNT_HINGE and type != mjJNT_SLIDE) {
        return makeError(ErrorCode::Unsupported, "MJCF joint '" + joint.descriptionJoint + "' is not 1-dof");
    }
    auto const actuator = mj_name2id(&model, mjOBJ_ACTUATOR, joint.name.c_str());
    if (actuator < 0) {
        return makeError(ErrorCode::InvalidConfig, "MJCF has no actuator '" + joint.name + "'");
    }
    if (model.actuator_trntype[actuator] != mjTRN_JOINT or model.actuator_trnid[2 * actuator] != jointId or
        model.actuator_gear[6 * actuator] != 1.0 or model.actuator_dyntype[actuator] != mjDYN_NONE) {
        return makeError(ErrorCode::InvalidConfig, "actuator '" + joint.name + "' must drive joint '" +
                                                       joint.descriptionJoint +
                                                       "' directly with gear 1 and no activation dynamics");
    }
    return JointBinding{
        .qposAddress = model.jnt_qposadr[jointId],
        .dofAddress = model.jnt_dofadr[jointId],
        .actuator = actuator,
    };
}

} // namespace

Expected<MujocoWorld> loadMujocoWorld(RobotProfile const &profile) {
    auto model = loadModel(profile.mjcf);
    if (not model) {
        return tl::make_unexpected(model.error());
    }
    auto world = MujocoWorld{.model = std::move(*model), .dof = profile.dof()};
    for (std::size_t i = 0; i < profile.dof(); ++i) {
        auto binding = bindJoint(*world.model, profile.joints[i]);
        if (not binding) {
            return tl::make_unexpected(binding.error());
        }
        world.joints[i] = *binding;
    }
    world.data = DataHandle{mj_makeData(world.model.get())};
    if (not world.data) {
        return makeError(ErrorCode::Internal, "cannot allocate MuJoCo data");
    }
    return world;
}

} // namespace larm::sim
