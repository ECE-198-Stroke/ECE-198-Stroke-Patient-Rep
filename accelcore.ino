/*
 * AccelCore.h - rep counting and movement quality from one accelerometer axis.
 *
 * No Arduino dependency, so tests/test_accel_core.cpp can run it on a laptop.
 *
 * A rep is two strokes in opposite directions (out and back), with or without a pause
 * between them. Direction comes from integrating acceleration into a velocity estimate,
 * which is reset whenever the sled is at rest so errors cannot build up across a set.
 * That estimate is only good enough for direction. This code deliberately reports no
 * distance or speed in physical units.
 *
 * Limit: very slow slides produce too little acceleration to separate from noise.
 * The Config thresholds are starting values and need tuning on real data ("r" raw mode).
 */
#pragma once

#include <math.h>
#include <stdint.h>

namespace accel {

static const float GRAVITY_MS2 = 9.81f;

struct Config {
  float filterAlpha = 0.3f;       // Low-pass strength. About 7 Hz at 100 Hz sampling.
  float startG = 0.012f;          // Acceleration that counts as movement starting.
  float stopG = 0.010f;           // Acceleration below this counts as quiet.
  float directionMs = 0.02f;      // Velocity (m/s) needed to establish a direction.
  float reversalFraction = 0.5f;  // Opposite velocity needed, as a fraction of the stroke's peak.
  float restFraction = 0.3f;       // Velocity below this fraction of the peak can count as stopped.
  float minPeakMs = 0.03f;        // Strokes below this velocity are bumps.
  float leakTauS = 8.0f;          // Bleeds off velocity drift during long movements.
  uint32_t quietMs = 300;         // Quiet and slow for this long = at rest.
  uint32_t maxQuietMs = 700;      // Quiet this long = at rest, whatever the velocity estimate.
  uint32_t minStrokeMs = 300;     // Shorter strokes are bumps.
  uint32_t calibrateMs = 1000;    // Stillness needed at the start of a set.
  float calibrateSpreadG = 0.04f;  // More variation than this means it was not still.
};

enum Event : uint8_t { EVENT_NONE, EVENT_CALIBRATED, EVENT_CALIBRATION_RETRY, EVENT_REP };

struct RepResult {
  uint32_t number = 0;
  float durationS = 0;
  float peakG = 0;
  float jerkRmsGS = 0;  // Lower is smoother. Only compare readings from the same setup.
};

struct SetSummary {
  uint32_t reps = 0;
  float durationS = 0;
  float meanRepS = 0;
  float repSpreadS = 0;  // Standard deviation of rep time: how even the pace was.
  float meanJerkRmsGS = 0;
};

class Tracker {
public:
  explicit Tracker(const Config &config = Config()) : config_(config) {}

  bool active() const { return mode_ != MODE_IDLE; }
  bool calibrating() const { return mode_ == MODE_CALIBRATING; }
  uint32_t reps() const { return reps_; }
  const RepResult &lastResult() const { return result_; }

  // The sled must be still. Calibration measures gravity on this axis, which is the tilt.
  void startSet(uint32_t nowMs) {
    mode_ = MODE_CALIBRATING;
    reps_ = 0;
    timeSum_ = timeSquaredSum_ = jerkSum_ = 0;
    setStartMs_ = nowMs;
    lastMs_ = nowMs;
    beginCalibration(nowMs);
  }

  SetSummary endSet(uint32_t nowMs) {
    SetSummary summary;
    summary.reps = reps_;
    summary.durationS = (nowMs - setStartMs_) / 1000.0f;
    if (reps_ > 0) {
      summary.meanRepS = timeSum_ / reps_;
      const float variance = timeSquaredSum_ / reps_ - summary.meanRepS * summary.meanRepS;
      summary.repSpreadS = variance > 0 ? sqrtf(variance) : 0.0f;
      summary.meanJerkRmsGS = jerkSum_ / reps_;
    }
    mode_ = MODE_IDLE;
    return summary;
  }

  // Feed one reading of the slide axis, in g, at a steady rate.
  Event update(float axisG, uint32_t nowMs) {
    if (mode_ == MODE_IDLE) {
      return EVENT_NONE;
    }

    const uint32_t elapsedMs = nowMs - lastMs_;
    lastMs_ = nowMs;

    if (mode_ == MODE_CALIBRATING) {
      return updateCalibration(axisG, nowMs);
    }

    filtered_ += config_.filterAlpha * (axisG - filtered_);
    const float a = filtered_ - baselineG_;

    if (mode_ == MODE_REST) {
      if (fabsf(a) <= config_.startG) {
        baselineG_ += 0.01f * (filtered_ - baselineG_);  // Follows slow sensor drift.
        return EVENT_NONE;
      }
      mode_ = MODE_MOVING;
      velocity_ = 0;
      direction_ = 0;
      quietMs_ = 0;
      strokePeak_ = 0;
      motionStartMs_ = nowMs;
      previousA_ = a;
      if (lastDirection_ == 0) {
        clearRep();
      }
    }
    return updateMoving(a, elapsedMs, nowMs);
  }

private:
  enum Mode : uint8_t { MODE_IDLE, MODE_CALIBRATING, MODE_REST, MODE_MOVING };

  void beginCalibration(uint32_t nowMs) {
    calibrationStartMs_ = nowMs;
    calibrationSum_ = 0;
    calibrationCount_ = 0;
    calibrationMin_ = 1e9f;
    calibrationMax_ = -1e9f;
  }

  Event updateCalibration(float axisG, uint32_t nowMs) {
    calibrationSum_ += axisG;
    calibrationCount_++;
    if (axisG < calibrationMin_) calibrationMin_ = axisG;
    if (axisG > calibrationMax_) calibrationMax_ = axisG;

    if (nowMs - calibrationStartMs_ < config_.calibrateMs) {
      return EVENT_NONE;
    }
    if (calibrationMax_ - calibrationMin_ > config_.calibrateSpreadG) {
      beginCalibration(nowMs);
      return EVENT_CALIBRATION_RETRY;
    }

    baselineG_ = calibrationSum_ / calibrationCount_;
    filtered_ = baselineG_;
    lastDirection_ = 0;
    clearRep();
    mode_ = MODE_REST;
    return EVENT_CALIBRATED;
  }

  void clearRep() {
    repStartMs_ = 0;
    repStarted_ = false;
    repPeakG_ = 0;
    jerkSquaredSum_ = 0;
    jerkCount_ = 0;
  }

  Event updateMoving(float a, uint32_t elapsedMs, uint32_t nowMs) {
    const float dt = elapsedMs / 1000.0f;
    velocity_ += a * GRAVITY_MS2 * dt;
    velocity_ -= velocity_ * dt / config_.leakTauS;

    if (dt > 0) {
      const float jerk = (a - previousA_) / dt;
      jerkSquaredSum_ += jerk * jerk;
      jerkCount_++;
    }
    previousA_ = a;
    if (fabsf(a) > repPeakG_) {
      repPeakG_ = fabsf(a);
    }

    const float speed = fabsf(velocity_);
    Event event = EVENT_NONE;

    if (direction_ == 0) {
      if (speed > config_.directionMs) {
        direction_ = velocity_ > 0 ? 1 : -1;
        strokeStartMs_ = motionStartMs_;
        strokePeak_ = speed;
      }
    } else if ((velocity_ > 0) == (direction_ > 0)) {
      if (speed > strokePeak_) {
        strokePeak_ = speed;
      }
    } else if (speed > config_.directionMs &&
               speed > strokePeak_ * config_.reversalFraction) {
      // Reversed without stopping.
      const int8_t next = -direction_;
      event = endStroke(nowMs);
      direction_ = next;
      strokeStartMs_ = nowMs;
      strokePeak_ = speed;
    }

    quietMs_ = fabsf(a) < config_.stopG ? quietMs_ + elapsedMs : 0;
    // Compared with the stroke's peak because the estimate ends a slide with some error left in
    // it. A coasting sled keeps most of its peak, so it does not pass for stopped.
    const float slowLimit = fmaxf(config_.directionMs, strokePeak_ * config_.restFraction);
    const bool slow = speed < slowLimit;
    if ((quietMs_ >= config_.quietMs && slow) || quietMs_ >= config_.maxQuietMs) {
      const Event ended = endStroke(nowMs - quietMs_);
      if (ended != EVENT_NONE) {
        event = ended;
      }
      mode_ = MODE_REST;
      velocity_ = 0;
      direction_ = 0;
    }
    return event;
  }

  Event endStroke(uint32_t endMs) {
    const bool real = direction_ != 0 && endMs - strokeStartMs_ >= config_.minStrokeMs &&
                      strokePeak_ >= config_.minPeakMs;
    if (!real) {
      return EVENT_NONE;
    }

    if (lastDirection_ == 0) {
      lastDirection_ = direction_;
      repStartMs_ = strokeStartMs_;
      repStarted_ = true;
      return EVENT_NONE;
    }
    if (direction_ == lastDirection_) {
      return EVENT_NONE;  // A second push the same way: still the outward stroke.
    }

    reps_++;
    result_.number = reps_;
    result_.durationS = (endMs - repStartMs_) / 1000.0f;
    result_.peakG = repPeakG_;
    result_.jerkRmsGS = jerkCount_ ? sqrtf(jerkSquaredSum_ / jerkCount_) : 0.0f;
    timeSum_ += result_.durationS;
    timeSquaredSum_ += result_.durationS * result_.durationS;
    jerkSum_ += result_.jerkRmsGS;
    lastDirection_ = 0;
    clearRep();
    return EVENT_REP;
  }

  Config config_;
  Mode mode_ = MODE_IDLE;
  uint32_t lastMs_ = 0;
  uint32_t setStartMs_ = 0;
  uint32_t reps_ = 0;
  float timeSum_ = 0, timeSquaredSum_ = 0, jerkSum_ = 0;

  uint32_t calibrationStartMs_ = 0;
  float calibrationSum_ = 0, calibrationMin_ = 0, calibrationMax_ = 0;
  uint32_t calibrationCount_ = 0;

  float baselineG_ = 0, filtered_ = 0, velocity_ = 0, previousA_ = 0;
  int8_t direction_ = 0, lastDirection_ = 0;
  uint32_t quietMs_ = 0, motionStartMs_ = 0, strokeStartMs_ = 0;
  float strokePeak_ = 0;

  uint32_t repStartMs_ = 0;
  bool repStarted_ = false;
  float repPeakG_ = 0, jerkSquaredSum_ = 0;
  uint32_t jerkCount_ = 0;
  RepResult result_;
};

}  // namespace accel
