#include "g1_model.h"

#include <pinocchio/parsers/urdf.hpp>

#ifndef CONTROLLER_MODEL_DIR
#error "CONTROLLER_MODEL_DIR must be defined by the build (see CMakeLists.txt)"
#endif

namespace walking_kjw {

pinocchio::Model LoadG1Model() {
  pinocchio::Model model;
  pinocchio::urdf::buildModel(std::string(CONTROLLER_MODEL_DIR) + "/g1/g1.urdf",
                               pinocchio::JointModelFreeFlyer(), model);
  return model;
}

}  // namespace walking_kjw
