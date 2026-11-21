#include "EleroCover.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace elero {

using namespace esphome::cover;

static const char *const TAG = "elero.cover";

// Position shown in Home Assistant while the blind sits on the sill with the
// slats still open (only used when lamella_duration is configured)
static const float LAMELLA_OPEN_POSITION = 0.01f;
static const float POSITION_EPSILON = 0.005f;

void EleroCover::dump_config() {
  LOG_COVER("", "Elero Cover", this);
  if(this->lamella_duration_ > 0)
    ESP_LOGCONFIG(TAG, "  Lamella duration: %u ms", this->lamella_duration_);
  if(this->intermediate_position_ >= 0.0f)
    ESP_LOGCONFIG(TAG, "  Intermediate position: %.0f%%", this->intermediate_position_ * 100.0f);
  if(this->ventilation_position_ >= 0.0f)
    ESP_LOGCONFIG(TAG, "  Ventilation position: %.0f%%", this->ventilation_position_ * 100.0f);
}

void EleroCover::setup() {
  this->parent_->register_cover(this);
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    if(this->has_durations_())
      this->position = 0.5f;
  }
  this->raw_position_ = this->to_raw_(this->position);
}

bool EleroCover::has_durations_() {
  return (this->open_duration_ > 0) && (this->close_duration_ > 0);
}

// ---------------------------------------------------------------------------
// Position mapping
//
// raw_position_ is the travel-time based position (0 = closed, 1 = open).
// The position published to Home Assistant can differ when lamella_duration
// is set: 0 % = fully closed, 1 % = sitting on the sill with open slats,
// 1..100 % = evenly spread over the remaining travel.
// ---------------------------------------------------------------------------
float EleroCover::lamella_raw_() {
  if((this->lamella_duration_ == 0) || (this->close_duration_ == 0))
    return 0.0f;
  float rs = (float) this->lamella_duration_ / (float) this->close_duration_;
  return clamp(rs, 0.0f, 0.5f);
}

float EleroCover::to_display_(float raw) {
  float rs = this->lamella_raw_();
  if(rs <= 0.0f)
    return raw;
  if(raw <= 0.001f)
    return COVER_CLOSED;
  if(raw >= 0.999f)
    return COVER_OPEN;
  if(raw <= rs)
    return LAMELLA_OPEN_POSITION;
  return LAMELLA_OPEN_POSITION + (raw - rs) / (1.0f - rs) * (1.0f - LAMELLA_OPEN_POSITION);
}

float EleroCover::to_raw_(float display) {
  float rs = this->lamella_raw_();
  if(rs <= 0.0f)
    return display;
  if(display <= 0.0f)
    return 0.0f;
  if(display >= 1.0f)
    return 1.0f;
  if(display <= LAMELLA_OPEN_POSITION)
    return rs;
  return rs + (display - LAMELLA_OPEN_POSITION) / (1.0f - LAMELLA_OPEN_POSITION) * (1.0f - rs);
}

void EleroCover::publish_(bool save) {
  this->position = this->to_display_(this->raw_position_);
  this->publish_state(save);
  this->last_publish_ = millis();
}

// ---------------------------------------------------------------------------

void EleroCover::loop() {
  uint32_t intvl = this->poll_intvl_;
  uint32_t now = millis();
  if(this->current_operation != COVER_OPERATION_IDLE) {
    // do not poll frequently for an extended period of time
    // (written as a difference: "now - TIMEOUT" underflowed during the first two
    // minutes after boot and disabled fast polling completely)
    if((now - this->movement_start_) < ELERO_TIMEOUT_MOVEMENT)
      intvl = ELERO_POLL_INTERVAL_MOVING;
  }

  if((now > this->poll_offset_) && (now - this->poll_offset_ - this->last_poll_) > intvl) {
    this->commands_to_send_.push(this->command_check_);
    this->last_poll_ = now - this->poll_offset_;
  }

  // Status request after a STOP/release from the original remote
  if((this->force_poll_at_ != 0) && ((int32_t)(now - this->force_poll_at_) >= 0)) {
    this->commands_to_send_.push(this->command_check_);
    if(this->force_poll_count_ > 1) {
      this->force_poll_count_--;
      this->force_poll_at_ = now + ELERO_REMOTE_STOP_POLL_DELAY * 2;
    } else {
      this->force_poll_count_ = 0;
      this->force_poll_at_ = 0;
    }
  }

  this->handle_commands(now);

  if((this->current_operation != COVER_OPERATION_IDLE) && this->has_durations_()) {
    this->recompute_position();
    if(this->special_target_ >= 0.0f) {
      // Movement to a position stored in the motor: the motor stops by itself,
      // so we must not send a stop command - just stop tracking at the target.
      bool reached = ((this->current_operation == COVER_OPERATION_OPENING) && (this->raw_position_ >= this->special_target_)) ||
                     ((this->current_operation == COVER_OPERATION_CLOSING) && (this->raw_position_ <= this->special_target_));
      if(reached) {
        this->raw_position_ = this->special_target_;
        this->special_target_ = -1.0f;
        this->target_position_ = COVER_OPEN;
        this->set_idle_and_publish();
        return;
      }
    } else if(this->is_at_target()) {
      this->commands_to_send_.push(this->command_stop_);
      this->target_position_ = COVER_OPEN;
      // Publish immediately, otherwise the cover stays "opening"/"closing" in HA
      this->set_idle_and_publish();
      return;
    }

    // End position reached by time: the motor stops by itself. Do not wait for
    // its status reply (which can take a long time) - assume idle after a grace period.
    bool at_end = ((this->current_operation == COVER_OPERATION_OPENING) && (this->raw_position_ >= 1.0f)) ||
                  ((this->current_operation == COVER_OPERATION_CLOSING) && (this->raw_position_ <= 0.0f));
    if(at_end) {
      if(this->end_reached_time_ == 0) {
        this->end_reached_time_ = now;
      } else if((now - this->end_reached_time_) > ELERO_END_POSITION_GRACE) {
        this->end_reached_time_ = 0;
        this->set_idle_and_publish();
        return;
      }
    } else {
      this->end_reached_time_ = 0;
    }

    // Publish position every second
    if(now - this->last_publish_ > 1000) {
      this->publish_(false);
    }
  }
}

void EleroCover::set_idle_and_publish() {
  this->current_operation = COVER_OPERATION_IDLE;
  this->publish_();
}

void EleroCover::track_movement(CoverOperation op, float target) {
  this->target_position_ = target;
  if(op == this->current_operation)
    return;
  // Bring the position up to date before changing direction
  this->recompute_position();
  this->current_operation = op;
  const uint32_t now = millis();
  this->movement_start_ = now;  // also enables fast status polling while moving
  this->last_recompute_time_ = now;
  if(op != COVER_OPERATION_IDLE) {
    this->last_operation_ = op;
    this->tilt = 0.0;
  }
  this->publish_();
}

void EleroCover::start_special_movement(float raw_target) {
  if(fabsf(raw_target - this->raw_position_) < 0.01f) {
    // Already there (as far as we know)
    this->special_target_ = -1.0f;
    this->raw_position_ = raw_target;
    this->set_idle_and_publish();
    return;
  }
  this->special_target_ = raw_target;
  this->track_movement(raw_target > this->raw_position_ ? COVER_OPERATION_OPENING : COVER_OPERATION_CLOSING, raw_target);
}

void EleroCover::handle_remote_command(uint8_t cmd) {
  ESP_LOGD(TAG, "Remote command 0x%02x for blind 0x%06x", cmd, this->command_.blind_addr);
  if(cmd == this->command_up_) {
    this->special_target_ = -1.0f;
    this->track_movement(COVER_OPERATION_OPENING, COVER_OPEN);
  } else if(cmd == this->command_down_) {
    this->special_target_ = -1.0f;
    this->track_movement(COVER_OPERATION_CLOSING, COVER_CLOSED);
  } else if(cmd == this->command_stop_) {
    // Careful: some remotes (e.g. MonoTel 868) also send 0x10 when an UP/DOWN
    // button is released - the motor keeps moving then. So do not stop tracking
    // here; ask the motor for its state shortly afterwards and let the reply
    // (stopped / moving) decide.
    this->force_poll_at_ = millis() + ELERO_REMOTE_STOP_POLL_DELAY;
    this->force_poll_count_ = 2;  // ask twice in case a reply gets lost
  } else if((cmd == this->command_intermediate_) && (this->intermediate_position_ >= 0.0f)) {
    this->start_special_movement(this->to_raw_(this->intermediate_position_));
  } else if((cmd == this->command_tilt_) && (this->ventilation_position_ >= 0.0f)) {
    this->start_special_movement(this->to_raw_(this->ventilation_position_));
  }
}

void EleroCover::move_to_intermediate() {
  this->commands_to_send_.push(this->command_intermediate_);
  if(this->intermediate_position_ >= 0.0f)
    this->start_special_movement(this->to_raw_(this->intermediate_position_));
}

void EleroCover::move_to_ventilation() {
  this->commands_to_send_.push(this->command_tilt_);
  if(this->ventilation_position_ >= 0.0f)
    this->start_special_movement(this->to_raw_(this->ventilation_position_));
}

bool EleroCover::is_at_target() {
  // We return false as we don't want to send a stop command for completely open or
  // close - this is handled by the cover
  if((this->target_position_ == COVER_OPEN) || (this->target_position_ == COVER_CLOSED))
    return false;

  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      return this->raw_position_ >= this->target_position_;
    case COVER_OPERATION_CLOSING:
      return this->raw_position_ <= this->target_position_;
    case COVER_OPERATION_IDLE:
    default:
      return true;
  }
}

void EleroCover::handle_commands(uint32_t now) {
  if((now - this->last_command_) > ELERO_DELAY_SEND_PACKETS) {
    if(this->commands_to_send_.size() > 0) {
      this->command_.payload[4] = this->commands_to_send_.front();
      if(this->parent_->send_command(&this->command_)) {
        this->send_packets_++;
        this->send_retries_ = 0;
        if(this->send_packets_ >= ELERO_SEND_PACKETS) {
          this->commands_to_send_.pop();
          this->send_packets_ = 0;
          this->increase_counter();
        }
      } else {
        ESP_LOGD(TAG, "Retry #%d for blind 0x%02x", this->send_retries_, this->command_.blind_addr);
        this->send_retries_++;
        if(this->send_retries_ > ELERO_SEND_RETRIES) {
          ESP_LOGE(TAG, "Hit maximum number of retries, giving up.");
          this->send_retries_ = 0;
          this->commands_to_send_.pop();
        }
      }
      this->last_command_ = now;
    }
  }
}

float EleroCover::get_setup_priority() const { return setup_priority::DATA; }

cover::CoverTraits EleroCover::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_supports_stop(true);
  traits.set_supports_position(this->has_durations_());
  traits.set_supports_toggle(true);
  traits.set_is_assumed_state(true);
  traits.set_supports_tilt(this->supports_tilt_);
  return traits;
}

void EleroCover::set_rx_state(uint8_t state) {
  ESP_LOGV(TAG, "Got state: 0x%02x for blind 0x%06x", state, this->command_.blind_addr);
  // Bring the estimated position up to date first
  this->recompute_position();
  float pos = this->raw_position_;
  float current_tilt = this->tilt;
  CoverOperation op = this->current_operation;
  bool end_position = false;

  switch(state) {
  case ELERO_STATE_TOP:
    pos = COVER_OPEN;
    op = COVER_OPERATION_IDLE;
    current_tilt = 0.0;
    end_position = true;
    break;
  case ELERO_STATE_BOTTOM:
    pos = COVER_CLOSED;
    op = COVER_OPERATION_IDLE;
    current_tilt = 0.0;
    end_position = true;
    break;
  case ELERO_STATE_START_MOVING_UP:
  case ELERO_STATE_MOVING_UP:
    op = COVER_OPERATION_OPENING;
    current_tilt = 0.0;
    break;
  case ELERO_STATE_START_MOVING_DOWN:
  case ELERO_STATE_MOVING_DOWN:
    op = COVER_OPERATION_CLOSING;
    current_tilt = 0.0;
    break;
  case ELERO_STATE_TILT:
    op = COVER_OPERATION_IDLE;
    current_tilt = 1.0;
    break;
  case ELERO_STATE_STOPPED:
  case ELERO_STATE_INTERMEDIATE:
  default:
    op = COVER_OPERATION_IDLE;
    current_tilt = 0.0;
  }

  if(op == COVER_OPERATION_IDLE) {
    // The motor stopped at a position stored in it -> we know exactly where it is
    if(!end_position && (this->special_target_ >= 0.0f))
      pos = this->special_target_;
    this->special_target_ = -1.0f;
  }

  bool movement_started = (op != COVER_OPERATION_IDLE) && (op != this->current_operation);
  if((pos != this->raw_position_) || (op != this->current_operation) || (current_tilt != this->tilt)) {
    this->raw_position_ = pos;
    this->tilt = current_tilt;
    this->current_operation = op;
    if(movement_started) {
      // Movement started by someone else (e.g. the original remote): start tracking now,
      // otherwise the position would jump and status polling would stay slow
      const uint32_t now = millis();
      this->movement_start_ = now;
      this->last_recompute_time_ = now;
      this->last_operation_ = op;
      if(this->special_target_ < 0.0f)
        this->target_position_ = (op == COVER_OPERATION_OPENING) ? COVER_OPEN : COVER_CLOSED;
    }
    this->publish_();
  }
}

void EleroCover::increase_counter() {
  if(this->command_.counter == 0xff)
    this->command_.counter = 1;
  else
    this->command_.counter += 1;
}

bool EleroCover::try_special_position_(float display_target) {
  // Use the positions stored in the motor when they are requested. The motor
  // reaches the intermediate position coming from above and the ventilation
  // position coming from below (that is how they are taught in), so only use
  // them in that direction - otherwise fall back to the time-based movement.
  if((this->intermediate_position_ >= 0.0f) &&
     (fabsf(display_target - this->intermediate_position_) < POSITION_EPSILON)) {
    float raw = this->to_raw_(this->intermediate_position_);
    if(this->raw_position_ > raw + 0.01f) {
      this->move_to_intermediate();
      return true;
    }
  }
  if((this->ventilation_position_ >= 0.0f) &&
     (fabsf(display_target - this->ventilation_position_) < POSITION_EPSILON)) {
    float raw = this->to_raw_(this->ventilation_position_);
    if(this->raw_position_ < raw - 0.01f) {
      this->move_to_ventilation();
      return true;
    }
  }
  return false;
}

void EleroCover::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    this->start_movement(COVER_OPERATION_IDLE);
  }
  if (call.get_position().has_value()) {
    auto display_pos = *call.get_position();
    if(this->has_durations_() && (this->current_operation == COVER_OPERATION_IDLE) &&
       this->try_special_position_(display_pos)) {
      return;
    }
    auto pos = this->to_raw_(display_pos);
    this->target_position_ = pos;
    if((pos > this->raw_position_) || (pos == COVER_OPEN)) {
      this->start_movement(COVER_OPERATION_OPENING);
    } else {
      this->start_movement(COVER_OPERATION_CLOSING);
    }
  }
  if (call.get_tilt().has_value()) {
    auto tilt = *call.get_tilt();
    if(tilt > 0) {
      this->commands_to_send_.push(this->command_tilt_);
      this->tilt = 1.0;
    } else {
      this->tilt = 0.0;
    }
  }
  if (call.get_toggle().has_value()) {
    if(this->current_operation != COVER_OPERATION_IDLE) {
      this->start_movement(COVER_OPERATION_IDLE);
    } else {
      if(this->raw_position_ == COVER_CLOSED || this->last_operation_ == COVER_OPERATION_CLOSING) {
        this->target_position_ = COVER_OPEN;
        this->start_movement(COVER_OPERATION_OPENING);
      } else {
        this->target_position_ = COVER_CLOSED;
        this->start_movement(COVER_OPERATION_CLOSING);
      }
    }
  }
}

// FIXME: Most of this should probably be moved to the
// handle_commands function to only publish a new state
// if at least the transmission was successful
void EleroCover::start_movement(CoverOperation dir) {
  this->special_target_ = -1.0f;
  switch(dir) {
    case COVER_OPERATION_OPENING:
      ESP_LOGV(TAG, "Sending OPEN command");
      this->commands_to_send_.push(this->command_up_);
      // Reset tilt state on movement
      this->tilt = 0.0;
      this->last_operation_ = COVER_OPERATION_OPENING;
    break;
    case COVER_OPERATION_CLOSING:
      ESP_LOGV(TAG, "Sending CLOSE command");
      this->commands_to_send_.push(this->command_down_);
      // Reset tilt state on movement
      this->tilt = 0.0;
      this->last_operation_ = COVER_OPERATION_CLOSING;
    break;
    case COVER_OPERATION_IDLE:
      this->commands_to_send_.push(this->command_stop_);
    break;
  }

  if(dir == this->current_operation)
    return;

  // Bring the position up to date before changing direction or stopping
  this->recompute_position();
  this->current_operation = dir;
  this->movement_start_ = millis();
  this->last_recompute_time_ = millis();
  this->publish_();
}

void EleroCover::recompute_position() {
  if(this->current_operation == COVER_OPERATION_IDLE)
    return;
  if(!this->has_durations_())
    return;

  float dir;
  float action_dur;
  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      dir = 1.0f;
      action_dur = this->open_duration_;
      break;
    case COVER_OPERATION_CLOSING:
      dir = -1.0f;
      action_dur = this->close_duration_;
      break;
    default:
      return;
  }

  const uint32_t now = millis();
  this->raw_position_ += dir * (now - this->last_recompute_time_) / action_dur;
  this->raw_position_ = clamp(this->raw_position_, 0.0f, 1.0f);

  this->last_recompute_time_ = now;
}

} // namespace elero
} // namespace esphome
