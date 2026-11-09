#pragma once

#include <math.h>
#include <stdint.h>

namespace accel {
static const float GRAVITY_MS2 = 9.81f;

struct Config {
  float filter_alpha = 0.3f;
  float start_g = 0.012f;
  float stop_g = 0.010f;
  float direction_m_s = 0.02f;
  float reversal_fraction = 0.5f;
  float rest_fraction = 0.3f;
  float min_peak_m_s = 0.03f;
  float velocity_decay_s = 8.0f;
  float baseline_alpha = 0.01f;
  uint32_t quiet_ms = 300;
  uint32_t max_quiet_ms = 700;
  uint32_t min_stroke_ms = 300;
  uint32_t calibrate_ms = 1000;
  float calibrate_spread_g = 0.04f;
};

enum Event : uint8_t {
  EVENT_NONE,
  EVENT_CALIBRATED,
  EVENT_CALIBRATION_RETRY,
  EVENT_REP
};

struct RepResult {
  uint32_t number = 0;
  float duration_s = 0;
  float peak_g = 0;

  float jerk_rms_g_s = 0;
};

struct SetSummary {
  uint32_t reps = 0;
  float duration_s = 0;
  float mean_rep_s = 0;
  float rep_spread_s = 0;
  float mean_jerk_rms_g_s = 0;
};

class Tracker {
public:
  Tracker(Config config = Config()) {
    config_ = config;
  }

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

  SetSummary endSet(uint32_t now_ms) {
    SetSummary summary;
    summary.reps = reps_;
    summary.duration_s = (now_ms - set_start_ms_) / 1000.0f;

    if (reps_ > 0) {
      summary.mean_rep_s = time_sum_ / reps_;
      summary.mean_jerk_rms_g_s = jerk_sum_ / reps_;

      const float variance = time_squared_sum_ / reps_ - summary.mean_rep_s * summary.mean_rep_s;

      if (variance > 0) {
        summary.rep_spread_s = sqrtf(variance);
      }
    }

    mode_ = MODE_IDLE;
    return summary;
  }

  Event update(float axis_g, uint32_t now_ms) {
    if (mode_ == MODE_IDLE) {
      return EVENT_NONE;
    }

    const uint32_t elapsed_ms = now_ms - last_ms_;
    last_ms_ = now_ms;

    if (mode_ == MODE_CALIBRATING) {
      return updateCalibration(axis_g, now_ms);
    }

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

private:
  enum Mode : uint8_t {
    MODE_IDLE,
    MODE_CALIBRATING,
    MODE_REST,
    MODE_MOVING
  };

  void beginCalibration(uint32_t now_ms) {
    calibration_start_ms_ = now_ms;
    calibration_sum_ = 0;
    calibration_count_ = 0;

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

  void clearRep() {
    rep_start_ms_ = 0;
    rep_peak_g_ = 0;
    jerk_squared_sum_ = 0;
    jerk_count_ = 0;
  }

  void startMoving(float accel_g, uint32_t now_ms) {
    mode_ = MODE_MOVING;
    velocity_ = 0;
    direction_ = 0;
    quiet_ms_ = 0;
    stroke_peak_ = 0;
    motion_start_ms_ = now_ms;
    previous_accel_g_ = accel_g;

    if (last_direction_ == 0) {
      clearRep();
    }
  }

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

  void updateMotion(float accel_g, float time_step_s) {
    velocity_ += accel_g * GRAVITY_MS2 * time_step_s;

    velocity_ -= velocity_ * time_step_s / config_.velocity_decay_s;

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

  Event updateDirection(uint32_t now_ms) {
    const float speed = fabsf(velocity_);

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
      if (speed > stroke_peak_) {
        stroke_peak_ = speed;
      }
      return EVENT_NONE;
    }

    const bool above_minimum = speed > config_.direction_m_s;
    const bool large_compared_to_stroke = speed > stroke_peak_ * config_.reversal_fraction;
    if (!above_minimum || !large_compared_to_stroke) {
      return EVENT_NONE;
    }

    const int8_t next_direction = -direction_;
    const Event event = endStroke(now_ms);
    direction_ = next_direction;
    stroke_start_ms_ = now_ms;
    stroke_peak_ = speed;
    return event;
  }

  Event updateStop(float accel_g, uint32_t elapsed_ms, uint32_t now_ms) {
    if (fabsf(accel_g) < config_.stop_g) {
      quiet_ms_ = quiet_ms_ + elapsed_ms;
    } else {
      quiet_ms_ = 0;
    }

    const float slow_limit = fmaxf(config_.direction_m_s, stroke_peak_ * config_.rest_fraction);
    const bool slow = fabsf(velocity_) < slow_limit;

    const bool quiet_and_slow = quiet_ms_ >= config_.quiet_ms && slow;
    const bool quiet_for_a_long_time = quiet_ms_ >= config_.max_quiet_ms;
    if (!quiet_and_slow && !quiet_for_a_long_time) {
      return EVENT_NONE;
    }

    const Event event = endStroke(now_ms - quiet_ms_);
    mode_ = MODE_REST;
    velocity_ = 0;
    direction_ = 0;
    return event;
  }

  Event endStroke(uint32_t end_ms) {
    const bool has_direction = direction_ != 0;
    const bool long_enough = end_ms - stroke_start_ms_ >= config_.min_stroke_ms;
    const bool fast_enough = stroke_peak_ >= config_.min_peak_m_s;
    if (!has_direction || !long_enough || !fast_enough) {
      return EVENT_NONE;
    }

    if (last_direction_ == 0) {
      last_direction_ = direction_;
      rep_start_ms_ = stroke_start_ms_;
      return EVENT_NONE;
    }

    if (direction_ == last_direction_) {
      return EVENT_NONE;
    }

    reps_++;
    result_.number = reps_;
    result_.duration_s = (end_ms - rep_start_ms_) / 1000.0f;
    result_.peak_g = rep_peak_g_;
    if (jerk_count_ > 0) {
      result_.jerk_rms_g_s = sqrtf(jerk_squared_sum_ / jerk_count_);
    } else {
      result_.jerk_rms_g_s = 0.0f;
    }

    time_sum_ += result_.duration_s;
    time_squared_sum_ += result_.duration_s * result_.duration_s;
    jerk_sum_ += result_.jerk_rms_g_s;

    last_direction_ = 0;
    clearRep();
    return EVENT_REP;
  }

  Config config_;
  Mode mode_ = MODE_IDLE;
  uint32_t last_ms_ = 0;

  uint32_t set_start_ms_ = 0;
  uint32_t reps_ = 0;
  float time_sum_ = 0;
  float time_squared_sum_ = 0;
  float jerk_sum_ = 0;

  uint32_t calibration_start_ms_ = 0;
  float calibration_sum_ = 0;
  uint32_t calibration_count_ = 0;
  float calibration_min_ = 0;
  float calibration_max_ = 0;

  float baseline_g_ = 0;
  float filtered_g_ = 0;
  float velocity_ = 0;
  float previous_accel_g_ = 0;
  uint32_t quiet_ms_ = 0;
  uint32_t motion_start_ms_ = 0;

  int8_t direction_ = 0;
  uint32_t stroke_start_ms_ = 0;
  float stroke_peak_ = 0;

  int8_t last_direction_ = 0;
  uint32_t rep_start_ms_ = 0;
  float rep_peak_g_ = 0;
  float jerk_squared_sum_ = 0;
  uint32_t jerk_count_ = 0;

  RepResult result_;
};
}
