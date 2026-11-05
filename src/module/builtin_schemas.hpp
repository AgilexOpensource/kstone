#pragma once

#include "kstone/module/builtins.hpp"
#include "kstone/module/schema.hpp"

// Internal: parameter schemas of the builtin modules, shared by the registry and the modules' tune().
namespace kstone::builtin_schemas {

const ParamSchema<FrameTaskModuleConfig>& frame_task();
const ParamSchema<JointTargetModuleConfig>& joint_target();
const ParamSchema<GazeModuleConfig>& gaze();
const ParamSchema<CenterOfMassModuleConfig>& center_of_mass();
const ParamSchema<NullSpacePreferenceModuleConfig>& null_space_preference();
/// Only the runtime-tunable `constraint` map; the registry reads the URDF-based parameters separately.
const ParamSchema<SelfCollisionModuleConfig>& self_collision();
const ParamSchema<WorldCollisionModuleConfig>& world_collision();
/// Registry fields plus the tune-only `pose_limits` map.
const ParamSchema<ChassisModuleConfig>& chassis();
/// The `solver` section of an engine config; every field is runtime-tunable (ModuleSet::tune_solver).
const ParamSchema<SolverOptions>& solver();

}  // namespace kstone::builtin_schemas
