// CommandInputThread (Th3): non-periodic stdin command parser.
// See Controller/docs/architecture.md section 3.
#pragma once

#include <atomic>

#include "data_warehouse.h"

namespace walking_kjw {

// Reads commands from stdin until EOF or "quit" (which also sets *stop).
// Commands (all radians, one per line):
//   motion walk|run|updown|walkready
//   compos <x> <y> <z>          -- Com_pos_Cal
//   comori <roll> <pitch> <yaw> -- Com_ori_Cal
//   vel <v>
//   wbc on|off
//   controlmode taskspace|jointspace
//   gain wbc <kp_roll> <kd_roll> <kp_pitch> <kd_pitch> <kp_height>
//   gain foot <kp> <kd>
//   gain joint <kp> <kd>
//   save start|end
//   drop on|off|auto  -- auto: slowly lower the pelvis_hold weld and
//                        auto-release once it's calculated to be just above
//                        the ground for the current stance (no touchdown
//                        sensor -- see kinematics_thread.cpp)
//   status  -- reprint the full command list with current values
//   quit
// Prints the full command list with each field's current value at startup
// and after every command, so stdin stays a legible "menu" to type against.
void RunCommandInputThread(DataWarehouse& dw, std::atomic<bool>& stop);

}  // namespace walking_kjw
