#include "MujocoWorld.h"

#include <larm/sim/SceneMirror.h>

#include <vector>

namespace larm::sim {
namespace {

// follower - follower_ref = c0 + c1 x + ... + c4 x^4, with x = leader - leader_ref.
struct JointCoupling {
    int follower{};
    int leader{};
    std::array<mjtNum, 5> coefficients{};
};

std::vector<JointCoupling> jointCouplings(mjModel const &model) {
    auto couplings = std::vector<JointCoupling>{};
    for (int i = 0; i < model.neq; ++i) {
        if (model.eq_type[i] != mjEQ_JOINT or model.eq_obj2id[i] < 0) {
            continue;
        }
        auto coupling = JointCoupling{.follower = model.eq_obj1id[i], .leader = model.eq_obj2id[i]};
        std::copy_n(model.eq_data + static_cast<std::ptrdiff_t>(i) * mjNEQDATA, coupling.coefficients.size(),
                    coupling.coefficients.begin());
        couplings.push_back(coupling);
    }
    return couplings;
}

struct SceneMirrorImpl final : SceneMirror {
    explicit SceneMirrorImpl(MujocoWorld world_)
        : world{std::move(world_)}, couplings{jointCouplings(*world.model)} {}

    mjModel const &model() const noexcept override { return *world.model; }
    mjData &data() noexcept override { return *world.data; }

    void show(JointVector const &position) noexcept override {
        auto const &model = *world.model;
        auto &data = *world.data;
        for (std::size_t i = 0; i < world.dof and i < dofOf(position); ++i) {
            data.qpos[world.joints[i].qposAddress] = position[idx(i)];
        }
        for (auto const &coupling : couplings) {
            auto const leader = model.jnt_qposadr[coupling.leader];
            auto const follower = model.jnt_qposadr[coupling.follower];
            auto const x = data.qpos[leader] - model.qpos0[leader];
            auto offset = mjtNum{0};
            for (auto it = coupling.coefficients.rbegin(); it != coupling.coefficients.rend(); ++it) {
                offset = offset * x + *it;
            }
            data.qpos[follower] = model.qpos0[follower] + offset;
        }
        mj_forward(world.model.get(), world.data.get());
    }

  private:
    MujocoWorld world;
    std::vector<JointCoupling> couplings;
};

} // namespace

Expected<std::unique_ptr<SceneMirror>> loadSceneMirror(RobotProfile const &profile) {
    auto world = loadMujocoWorld(profile);
    if (not world) {
        return tl::make_unexpected(world.error());
    }
    return std::unique_ptr<SceneMirror>{std::make_unique<SceneMirrorImpl>(std::move(*world))};
}

} // namespace larm::sim
