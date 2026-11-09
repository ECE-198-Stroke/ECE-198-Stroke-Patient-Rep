/*
 * AccelCore.h - counts reps and measures movement quality from one accelerometer axis.
 *
 * This code has no Arduino dependency, so tests/test_accel_core.cpp can run it on a laptop.
 *
 * How to use it:
 *   1. Create a Tracker.
 *   2. Call startSet() while the sled is still.
 *   3. Call update() with every new accelerometer reading, about 100 times per second.
 *   4. Call endSet() to get a summary of the set.
 *
 * Words used in this file:
 *   sled         The part that slides back and forth, with the accelerometer attached.
 *   stroke       One slide in one direction.
 *   rep          Two strokes in opposite directions (out and back), with or without a pause.
 *   calibration  Measuring the sensor while it is still, to learn what "not moving" reads as.
 *   baseline     The reading when the sled is still. Tilt makes this different from zero.
 *   jerk         How quickly acceleration changes. A jerky movement has a high value.
 *
 * Units are written at the end of variable names:
 *   _g    acceleration, where 1 g = 9.81 m/s^2
 *   _m_s  velocity in metres per second
 *   _s    seconds
 *   _ms   milliseconds
 *
 * How direction is found: adding up acceleration over time gives an estimate of velocity, and
 * the sign of that velocity gives the direction. Small errors add up too, so the estimate is
 * reset to zero whenever the sled is at rest. It is accurate enough to tell direction and too
 * inaccurate for distance or speed, so this code reports neither of those.
 *
 * Limit: very slow slides produce too little acceleration to separate from sensor noise.
 * The values in Config are starting points and need tuning on real data ("r" raw mode).
 */

// Tells the compiler to include this file only once, however many files ask for it.
#pragma once

#include <math.h>    // fabsf() is absolute value, sqrtf() is square root, fmaxf() is the larger of two.
#include <stdint.h>  // uint32_t is a 32-bit whole number with no sign. int8_t is an 8-bit signed one.

// A namespace groups related code. Code outside this file writes accel::Tracker, accel::Config...
namespace accel {

static const float GRAVITY_MS2 = 9.81f;

// A struct is a group of named values. These are the settings you can tune.
// The "f" after a number means it is a float, which is a number with a decimal point.
struct Config {
  // How much of each new reading to mix into the smoothed value, from 0 to 1.
  // Smaller values smooth more. 0.3 removes vibration above about 7 Hz at 100 readings per second.
  float filter_alpha = 0.3f;

  // Acceleration above this means the sled has started moving.
  float start_g = 0.012f;

  // Acceleration below this counts as quiet.
  float stop_g = 0.010f;

  // The velocity estimate must pass this before a stroke gets a direction.
  float direction_m_s = 0.02f;

  // To count as a reversal, velocity in the opposite direction must reach this fraction of the
  // stroke's fastest velocity.
  float reversal_fraction = 0.5f;

  // Velocity below this fraction of the stroke's fastest velocity can count as stopped.
  float rest_fraction = 0.3f;

  // A stroke that never reaches this velocity is treated as an accidental bump.
  float min_peak_m_s = 0.03f;

  // Shrinks the velocity estimate over time, so that errors fade during long movements.
  // A larger number means a slower fade.
  float velocity_decay_s = 8.0f;

  // How quickly the baseline follows slow changes in the sensor while the sled is at rest.
  float baseline_alpha = 0.01f;

  // Quiet for this long and moving slowly means the sled is at rest.
  uint32_t quiet_ms = 300;

  // Quiet for this long means the sled is at rest, whatever the velocity estimate says.
  uint32_t max_quiet_ms = 700;

  // A stroke shorter than this is treated as an accidental bump.
  uint32_t min_stroke_ms = 300;

  // How long the sled must stay still at the start of a set.
  uint32_t calibrate_ms = 1000;

  // If readings vary by more than this during calibration, the sled was moving.
  float calibrate_spread_g = 0.04f;
};

// An enum is a list of named options. update() returns one of these to say what just happened.
// ": uint8_t" stores the value in one byte to save memory on the microcontroller.
enum Event : uint8_t {
  EVENT_NONE,               // Nothing to report.
  EVENT_CALIBRATED,         // Calibration finished, so the patient can start.
  EVENT_CALIBRATION_RETRY,  // The sled moved during calibration, so it started again.
  EVENT_REP                 // A rep was completed. Call lastResult() for its details.
};

// The measurements for one rep.
struct RepResult {
  uint32_t number = 0;   // 1 for the first rep of the set, 2 for the second, and so on.
  float duration_s = 0;  // Time taken for the whole rep.
  float peak_g = 0;      // Largest acceleration during the rep.

  // Typical size of the jerk during the rep (root mean square), in g per second.
  // Lower is smoother. Only compare readings taken with the same setup.
  float jerk_rms_g_s = 0;
};

// The measurements for a whole set.
struct SetSummary {
  uint32_t reps = 0;
  float duration_s = 0;
  float mean_rep_s = 0;         // Average time per rep.
  float rep_spread_s = 0;       // Standard deviation of rep time. Smaller means a more even pace.
  float mean_jerk_rms_g_s = 0;  // Average of jerk_rms_g_s over the set.
};

class Tracker {
public:
  // Runs when a Tracker is created. With no argument, it uses the default values in Config.
  Tracker(Config config = Config()) {
    config_ = config;
  }

  // "const" after these function names means that calling them leaves the Tracker unchanged.
  bool active() const {
    return mode_ != MODE_IDLE;
  }

  bool calibrating() const {
    return mode_ == MODE_CALIBRATING;
  }

  uint32_t reps() const {
    return reps_;
  }

  RepResult lastResult() const {
    return result_;
  }

  // Starts a new set. The sled must be still, because calibration measures how much of
  // gravity falls on this axis, and that depends on the tilt.
  void startSet(uint32_t now_ms) {
    mode_ = MODE_CALIBRATING;
    reps_ = 0;
    time_sum_ = 0;
    time_squared_sum_ = 0;
    jerk_sum_ = 0;
    set_start_ms_ = now_ms;
    last_ms_ = now_ms;
    beginCalibration(now_ms);
  }

  // Finishes the set and returns its summary.
  SetSummary endSet(uint32_t now_ms) {
    SetSummary summary;
    summary.reps = reps_;
    summary.duration_s = (now_ms - set_start_ms_) / 1000.0f;

    // The check for at least one rep avoids dividing by zero.
    if (reps_ > 0) {
      summary.mean_rep_s = time_sum_ / reps_;
      summary.mean_jerk_rms_g_s = jerk_sum_ / reps_;

      // Variance is the average of the squares minus the square of the average.
      const float variance = time_squared_sum_ / reps_ - summary.mean_rep_s * summary.mean_rep_s;

      // Rounding can make variance slightly negative, and sqrtf() needs a positive number.
      if (variance > 0) {
        summary.rep_spread_s = sqrtf(variance);
      }
    }

    mode_ = MODE_IDLE;
    return summary;
  }

  // Call this with every new reading of the slide axis, in g, at a steady rate.
  Event update(float axis_g, uint32_t now_ms) {
    if (mode_ == MODE_IDLE) {
      return EVENT_NONE;
    }

    // Subtracting uint32_t times gives the right gap even after the clock wraps back to zero.
    const uint32_t elapsed_ms = now_ms - last_ms_;
    last_ms_ = now_ms;

    if (mode_ == MODE_CALIBRATING) {
      return updateCalibration(axis_g, now_ms);
    }

    // Smooth the reading, then subtract the baseline to get the acceleration from movement.
    filtered_g_ = filtered_g_ + config_.filter_alpha * (axis_g - filtered_g_);
    const float accel_g = filtered_g_ - baseline_g_;

    if (mode_ == MODE_REST) {
      const bool still_at_rest = fabsf(accel_g) <= config_.start_g;
      if (still_at_rest) {
        baseline_g_ = baseline_g_ + config_.baseline_alpha * (filtered_g_ - baseline_g_);
        return EVENT_NONE;
      }
      startMoving(accel_g, now_ms);
    }

    return updateMoving(accel_g, elapsed_ms, now_ms);
  }

// Everything below can only be used from inside this class.
private:
  // The four situations the Tracker can be in.
  enum Mode : uint8_t {
    MODE_IDLE,         // No set is running.
    MODE_CALIBRATING,  // Measuring the sled while it is still.
    MODE_REST,         // In a set, with the sled stopped.
    MODE_MOVING        // In a set, with the sled sliding.
  };

  void beginCalibration(uint32_t now_ms) {
    calibration_start_ms_ = now_ms;
    calibration_sum_ = 0;
    calibration_count_ = 0;

    // These start far outside any real reading, so the first reading replaces both.
    calibration_min_ = 1e9f;
    calibration_max_ = -1e9f;
  }

  Event updateCalibration(float axis_g, uint32_t now_ms) {
    calibration_sum_ += axis_g;
    calibration_count_++;
    if (axis_g < calibration_min_) {
      calibration_min_ = axis_g;
    }
    if (axis_g > calibration_max_) {
      calibration_max_ = axis_g;
    }

    const bool still_collecting = now_ms - calibration_start_ms_ < config_.calibrate_ms;
    if (still_collecting) {
      return EVENT_NONE;
    }

    const bool sled_moved = calibration_max_ - calibration_min_ > config_.calibrate_spread_g;
    if (sled_moved) {
      beginCalibration(now_ms);
      return EVENT_CALIBRATION_RETRY;
    }

    baseline_g_ = calibration_sum_ / calibration_count_;
    filtered_g_ = baseline_g_;
    last_direction_ = 0;
    clearRep();
    mode_ = MODE_REST;
    return EVENT_CALIBRATED;
  }

  // Forgets the measurements of the rep in progress.
  void clearRep() {
    rep_start_ms_ = 0;
    rep_peak_g_ = 0;
    jerk_squared_sum_ = 0;
    jerk_count_ = 0;
  }

  // Runs on the first reading after the sled starts to move.
  void startMoving(float accel_g, uint32_t now_ms) {
    mode_ = MODE_MOVING;
    velocity_ = 0;
    direction_ = 0;
    quiet_ms_ = 0;
    stroke_peak_ = 0;
    motion_start_ms_ = now_ms;
    previous_accel_g_ = accel_g;

    // A last_direction_ of 0 means the outward stroke has not happened yet, so a new rep starts.
    if (last_direction_ == 0) {
      clearRep();
    }
  }

  // Runs on every reading while the sled is moving.
  Event updateMoving(float accel_g, uint32_t elapsed_ms, uint32_t now_ms) {
    const float time_step_s = elapsed_ms / 1000.0f;
    updateMotion(accel_g, time_step_s);

    const Event reversal_event = updateDirection(now_ms);
    const Event stop_event = updateStop(accel_g, elapsed_ms, now_ms);

    if (stop_event != EVENT_NONE) {
      return stop_event;
    }
    return reversal_event;
  }

  // Updates the velocity estimate and collects the rep's jerk and largest acceleration.
  void updateMotion(float accel_g, float time_step_s) {
    // Velocity changes by acceleration times time. GRAVITY_MS2 converts g to m/s^2.
    velocity_ += accel_g * GRAVITY_MS2 * time_step_s;

    // Shrink the estimate slightly, so that errors fade away.
    velocity_ -= velocity_ * time_step_s / config_.velocity_decay_s;

    // The check for a positive time step avoids dividing by zero.
    if (time_step_s > 0) {
      const float jerk = (accel_g - previous_accel_g_) / time_step_s;
      jerk_squared_sum_ += jerk * jerk;
      jerk_count_++;
    }
    previous_accel_g_ = accel_g;

    if (fabsf(accel_g) > rep_peak_g_) {
      rep_peak_g_ = fabsf(accel_g);
    }
  }

  // Works out the stroke's direction, and ends the stroke if the sled reversed without stopping.
  Event updateDirection(uint32_t now_ms) {
    const float speed = fabsf(velocity_);

    // A direction_ of 0 means this stroke has no direction yet.
    if (direction_ == 0) {
      if (speed > config_.direction_m_s) {
        if (velocity_ > 0) {
          direction_ = 1;
        } else {
          direction_ = -1;
        }
        stroke_start_ms_ = motion_start_ms_;
        stroke_peak_ = speed;
      }
      return EVENT_NONE;
    }

    const bool velocity_is_positive = velocity_ > 0;
    const bool stroke_is_positive = direction_ > 0;
    if (velocity_is_positive == stroke_is_positive) {
      // Still going the same way, so only the fastest velocity can change.
      if (speed > stroke_peak_) {
        stroke_peak_ = speed;
      }
      return EVENT_NONE;
    }

    // The velocity has the opposite sign. Both checks must pass to count as a real reversal.
    const bool above_minimum = speed > config_.direction_m_s;
    const bool large_compared_to_stroke = speed > stroke_peak_ * config_.reversal_fraction;
    if (!above_minimum || !large_compared_to_stroke) {
      return EVENT_NONE;
    }

    // endStroke() reads direction_, so the old direction must stay in place until it has run.
    const int8_t next_direction = -direction_;
    const Event event = endStroke(now_ms);
    direction_ = next_direction;
    stroke_start_ms_ = now_ms;
    stroke_peak_ = speed;
    return event;
  }

  // Ends the stroke and goes back to rest once the sled has been quiet for long enough.
  Event updateStop(float accel_g, uint32_t elapsed_ms, uint32_t now_ms) {
    if (fabsf(accel_g) < config_.stop_g) {
      quiet_ms_ = quiet_ms_ + elapsed_ms;
    } else {
      quiet_ms_ = 0;
    }

    // The limit depends on the stroke's fastest velocity because the estimate still holds some
    // error at the end of a slide. A sled that is coasting keeps most of its fastest velocity,
    // so it stays above this limit and is treated as moving.
    const float slow_limit = fmaxf(config_.direction_m_s, stroke_peak_ * config_.rest_fraction);
    const bool slow = fabsf(velocity_) < slow_limit;

    const bool quiet_and_slow = quiet_ms_ >= config_.quiet_ms && slow;
    const bool quiet_for_a_long_time = quiet_ms_ >= config_.max_quiet_ms;
    if (!quiet_and_slow && !quiet_for_a_long_time) {
      return EVENT_NONE;
    }

    // The stroke really ended when the quiet period began, so that time is passed in.
    const Event event = endStroke(now_ms - quiet_ms_);
    mode_ = MODE_REST;
    velocity_ = 0;
    direction_ = 0;
    return event;
  }

  // Decides what a finished stroke means. Returns EVENT_REP if it completed a rep.
  Event endStroke(uint32_t end_ms) {
    const bool has_direction = direction_ != 0;
    const bool long_enough = end_ms - stroke_start_ms_ >= config_.min_stroke_ms;
    const bool fast_enough = stroke_peak_ >= config_.min_peak_m_s;
    if (!has_direction || !long_enough || !fast_enough) {
      return EVENT_NONE;
    }

    // The first stroke of a rep: remember its direction and wait for the return stroke.
    if (last_direction_ == 0) {
      last_direction_ = direction_;
      rep_start_ms_ = stroke_start_ms_;
      return EVENT_NONE;
    }

    // A second push the same way is still part of the outward stroke.
    if (direction_ == last_direction_) {
      return EVENT_NONE;
    }

    // A stroke in the opposite direction completes the rep.
    reps_++;
    result_.number = reps_;
    result_.duration_s = (end_ms - rep_start_ms_) / 1000.0f;
    result_.peak_g = rep_peak_g_;
    if (jerk_count_ > 0) {
      result_.jerk_rms_g_s = sqrtf(jerk_squared_sum_ / jerk_count_);
    } else {
      result_.jerk_rms_g_s = 0.0f;
    }

    // Running totals, used by endSet() for the averages.
    time_sum_ += result_.duration_s;
    time_squared_sum_ += result_.duration_s * result_.duration_s;
    jerk_sum_ += result_.jerk_rms_g_s;

    last_direction_ = 0;
    clearRep();
    return EVENT_REP;
  }

  // The variables below keep their values between calls.
  // A trailing underscore marks a variable that belongs to the class.

  Config config_;
  Mode mode_ = MODE_IDLE;
  uint32_t last_ms_ = 0;  // Time of the previous reading.

  // Totals for the set.
  uint32_t set_start_ms_ = 0;
  uint32_t reps_ = 0;
  float time_sum_ = 0;          // All rep times added together.
  float time_squared_sum_ = 0;  // All rep times squared and added together.
  float jerk_sum_ = 0;          // All values of jerk_rms_g_s added together.

  // Calibration.
  uint32_t calibration_start_ms_ = 0;
  float calibration_sum_ = 0;
  uint32_t calibration_count_ = 0;
  float calibration_min_ = 0;
  float calibration_max_ = 0;

  // The current motion.
  float baseline_g_ = 0;        // The reading when the sled is still.
  float filtered_g_ = 0;        // The smoothed reading.
  float velocity_ = 0;          // Estimated velocity in m/s.
  float previous_accel_g_ = 0;  // Acceleration at the previous reading, used for jerk.
  uint32_t quiet_ms_ = 0;       // How long the acceleration has stayed below stop_g.
  uint32_t motion_start_ms_ = 0;

  // The current stroke.
  int8_t direction_ = 0;  // 1 or -1 once known, and 0 before that.
  uint32_t stroke_start_ms_ = 0;
  float stroke_peak_ = 0;  // The stroke's fastest velocity so far.

  // The current rep.
  int8_t last_direction_ = 0;  // Direction of the outward stroke, or 0 if there is none yet.
  uint32_t rep_start_ms_ = 0;
  float rep_peak_g_ = 0;
  float jerk_squared_sum_ = 0;
  uint32_t jerk_count_ = 0;

  RepResult result_;  // The most recent completed rep.
};

}  // namespace accel
