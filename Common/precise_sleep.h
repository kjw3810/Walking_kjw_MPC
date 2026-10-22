// Tightens sleep_until()'s wake-up jitter for a 1kHz-target control loop.
// A single sleep_until(deadline) call relies on the OS to wake the thread
// back up exactly at `deadline`, but on a non-RT Linux kernel that wake-up
// is measurably late (see Common/loop_timer.h-based measurements,
// 2026-09-29: Controller's 1kHz threads ran their loop ~6.5% over target
// with negligible compute time, i.e. the overrun is scheduling wake latency,
// not application work). Mirrors the fix already applied to Mujoco's own
// physics loop (Mujoco/mujoco_engine/simulate/main.cc's PhysicsLoop): sleep
// coarsely for most of the wait (cheap, no CPU spent), then switch to
// short sleep_for() polling for the final stretch before the deadline, so
// lateness from that point on is bounded by ~one poll interval instead of
// the OS's full scheduling jitter.
#pragma once

#include <chrono>
#include <thread>

namespace walking_kjw {

// Defaults match the 2026-09-29 tuning: for a 1ms-period loop, switch to
// 50us polling for the last 200us (i.e. from the 0.8ms mark onward).
inline void SleepUntilPrecise(
    std::chrono::steady_clock::time_point deadline,
    std::chrono::microseconds poll_from_before_deadline = std::chrono::microseconds(200),
    std::chrono::microseconds poll_interval = std::chrono::microseconds(50)) {
  const auto coarse_until = deadline - poll_from_before_deadline;
  if (std::chrono::steady_clock::now() < coarse_until) {
    std::this_thread::sleep_until(coarse_until);
  }
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(poll_interval);
  }
}

}  // namespace walking_kjw
