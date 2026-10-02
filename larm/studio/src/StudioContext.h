#pragma once

#include "Journal.h"

#include <larm/model/RobotModel.h>
#include <larm/runtime/RobotSession.h>

#include <lexec/execution.hpp>
#include <lqtexec/EventLoopScheduler.h>

namespace larm::studio {

// What every panel borrows from the composition root.
struct StudioContext {
    runtime::RobotSession *session{};
    model::RobotModel const *model{};
    lqtexec::EventLoopScheduler gui;
    lexec::counting_scope *appScope{};
    Journal *journal{};
};

} // namespace larm::studio
