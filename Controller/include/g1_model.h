// Loads the Unitree G1 URDF (Controller/model/g1/g1.urdf, no-hand 29dof
// variant, arms+wrists fixed) as a free-floating-base Pinocchio model.
#pragma once

#include <pinocchio/multibody/model.hpp>

namespace walking_kjw {

// Builds the G1 Pinocchio model with a free-flyer root joint (pelvis is not
// bolted to the world). Throws std::runtime_error if the URDF can't be
// found/parsed.
pinocchio::Model LoadG1Model();

}  // namespace walking_kjw
