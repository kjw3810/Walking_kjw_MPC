// Diagnostic timing for a periodic loop (a 1kHz Controller thread, or
// Mujoco's mjcb_control callback -- called once per physics step, so its
// call-to-call interval doubles as "how long did this physics step actually
// take in wall time"). Tracks exactly two things per iteration, no more:
//   1) compute time: wall time actually spent working (TickStart() to
//      ComputeDone()), i.e. everything before the loop's sleep_until (or,
//      for a callback with no sleep of its own, everything the callback
//      itself does).
//   2) loop period: wall time between successive TickStart() calls -- the
//      *real* rate the loop runs at. If compute time + scheduling delay
//      ever exceeds the target period, sleep_until(tick_start + period)
//      returns immediately (the deadline's already past) instead of
//      sleeping, and the loop silently free-runs slower than intended --
//      this is what loop period catches that compute time alone can't.
// Prints a min/avg/max summary to stderr once per report_window, then resets.
#pragma once

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <limits>

namespace walking_kjw {

class LoopTimer {
 public:
  // `out` defaults to stderr; pass a dedicated log file instead when stderr
  // is shared with something interactive (e.g. Controller's CommandInput
  // prompt -- these reports were interleaving with typed input there).
  // Doesn't take ownership; caller opens/closes `out`.
  LoopTimer(const char* name, double target_period_ms,
            std::chrono::seconds report_window = std::chrono::seconds(1), std::FILE* out = stderr)
      : name_(name), target_period_ms_(target_period_ms), report_window_(report_window), out_(out) {}

  // Call once at the very top of each loop iteration, before any real work.
  void TickStart() {
    const auto now = std::chrono::steady_clock::now();
    if (have_prev_tick_) {
      RecordPeriod(std::chrono::duration<double, std::milli>(now - prev_tick_).count());
    }
    prev_tick_ = now;
    have_prev_tick_ = true;
    compute_start_ = now;
  }

  // Call once right before sleep_until (or at the end of a callback with no
  // sleep of its own), after all real work for this iteration is done.
  void ComputeDone() {
    const auto now = std::chrono::steady_clock::now();
    RecordCompute(std::chrono::duration<double, std::milli>(now - compute_start_).count());
    MaybeReport(now);
  }

 private:
  void RecordCompute(double ms) {
    compute_sum_ += ms;
    compute_min_ = std::min(compute_min_, ms);
    compute_max_ = std::max(compute_max_, ms);
    ++compute_n_;
  }

  void RecordPeriod(double ms) {
    period_sum_ += ms;
    period_min_ = std::min(period_min_, ms);
    period_max_ = std::max(period_max_, ms);
    ++period_n_;
  }

  void MaybeReport(std::chrono::steady_clock::time_point now) {
    if (!have_window_start_) {
      window_start_ = now;
      have_window_start_ = true;
      return;
    }
    if (now - window_start_ < report_window_) return;

    std::fprintf(out_,
                 "[%s] target=%.2fms | compute avg=%.3f min=%.3f max=%.3fms (n=%d) | loop "
                 "period avg=%.3f min=%.3f max=%.3fms (n=%d)\n",
                 name_, target_period_ms_, compute_n_ ? compute_sum_ / compute_n_ : 0.0,
                 compute_n_ ? compute_min_ : 0.0, compute_n_ ? compute_max_ : 0.0, compute_n_,
                 period_n_ ? period_sum_ / period_n_ : 0.0, period_n_ ? period_min_ : 0.0,
                 period_n_ ? period_max_ : 0.0, period_n_);
    std::fflush(out_);

    compute_sum_ = 0.0;
    compute_min_ = std::numeric_limits<double>::max();
    compute_max_ = 0.0;
    compute_n_ = 0;
    period_sum_ = 0.0;
    period_min_ = std::numeric_limits<double>::max();
    period_max_ = 0.0;
    period_n_ = 0;
    window_start_ = now;
  }

  const char* name_;
  double target_period_ms_;
  std::chrono::seconds report_window_;
  std::FILE* out_;

  std::chrono::steady_clock::time_point prev_tick_;
  bool have_prev_tick_ = false;
  std::chrono::steady_clock::time_point compute_start_;

  std::chrono::steady_clock::time_point window_start_;
  bool have_window_start_ = false;

  double compute_sum_ = 0.0;
  double compute_min_ = std::numeric_limits<double>::max();
  double compute_max_ = 0.0;
  int compute_n_ = 0;

  double period_sum_ = 0.0;
  double period_min_ = std::numeric_limits<double>::max();
  double period_max_ = 0.0;
  int period_n_ = 0;
};

}  // namespace walking_kjw
