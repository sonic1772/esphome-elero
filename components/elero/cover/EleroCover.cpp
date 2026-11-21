#include "EleroCover.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace elero {

using namespace esphome::cover;

static const char *const TAG = "elero.cover";

void EleroCover::dump_config() {
  LOG_COVER("", "Elero Cover", this);
}

void EleroCover::setup() {
  this->parent_->register_cover(this);
  auto restore = this->restore_state_();
  if (restore.has_value()) {
    restore->apply(this);
  } else {
    if((this->open_duration_ > 0) && (this->close_duration_ > 0))
      this->position = 0.5f;
  }
}

void EleroCover::loop() {
  uint32_t intvl = this->poll_intvl_;
  uint32_t now = millis();
  if(this->current_operation != COVER_OPERATION_IDLE) {
    if((now - ELERO_TIMEOUT_MOVEMENT) < this->movement_start_) // do not poll frequently for an extended period of time
      intvl = ELERO_POLL_INTERVAL_MOVING;
  }

  if((now > this->poll_offset_) && (now - this->poll_offset_ - this->last_poll_) > intvl) {
    this->commands_to_send_.push(this->command_check_);
    this->last_poll_ = now - this->poll_offset_;
  }

  this->handle_commands(now);

  if((this->current_operation != COVER_OPERATION_IDLE) && (this->open_duration_ > 0) && (this->close_duration_ > 0)) {
    this->recompute_position();
    if(this->special_target_ >= 0.0f) {
      // Movement to a position stored in the motor: the motor stops by itself,
      // so we must not send a stop command - just stop tracking at the target.
      bool reached = ((this->current_operation == COVER_OPERATION_OPENING) && (this->position >= this->special_target_)) ||
                     ((this->current_operation == COVER_OPERATION_CLOSING) && (this->position <= this->special_target_));
      if(reached) {
        this->position = this->special_target_;
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

    // Publish position every second
    if(now - this->last_publish_ > 1000) {
      this->publish_state(false);
      this->last_publish_ = now;
    }
  }
}

void EleroCover::set_idle_and_publish() {
  this->current_operation = COVER_OPERATION_IDLE;
  this->publish_state();
  this->last_publish_ = millis();
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
  this->publish_state();
  this->last_publish_ = now;
}

void EleroCover::start_special_movement(float target) {
  if(fabsf(target - this->position) < 0.01f) {
    // Already there (as far as we know)
    this->special_target_ = -1.0f;
    this->position = target;
    this->set_idle_and_publish();
    return;
  }
  this->special_target_ = target;
  this->track_movement(target > this->position ? COVER_OPERATION_OPENING : COVER_OPERATION_CLOSING, target);
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
    this->special_target_ = -1.0f;
    if(this->current_operation != COVER_OPERATION_IDLE) {
      this->recompute_position();
      this->target_position_ = COVER_OPEN;
      this->set_idle_and_publish();
    }
  } else if((cmd == this->command_intermediate_) && (this->intermediate_position_ >= 0.0f)) {
    this->start_special_movement(this->intermediate_position_);
  } else if((cmd == this->command_tilt_) && (this->ventilation_position_ >= 0.0f)) {
    this->start_special_movement(this->ventilation_position_);
  }
}

void EleroCover::move_to_intermediate() {
  this->commands_to_send_.push(this->command_intermediate_);
  if(this->intermediate_position_ >= 0.0f)
    this->start_special_movement(this->intermediate_position_);
}

void EleroCover::move_to_ventilation() {
  this->commands_to_send_.push(this->command_tilt_);
  if(this->ventilation_position_ >= 0.0f)
    this->start_special_movement(this->ventilation_position_);
}

bool EleroCover::is_at_target() {
  // We return false as we don't want to send a stop command for completely open or
  // close - this is handled by the cover
  if((this->target_position_ == COVER_OPEN) || (this->target_position_ == COVER_CLOSED))
    return false;

  switch (this->current_operation) {
    case COVER_OPERATION_OPENING:
      return this->position >= this->target_position_;
    case COVER_OPERATION_CLOSING:
      return this->position <= this->target_position_;
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
  if((this->open_duration_ > 0) && (this->close_duration_ > 0))
    traits.set_supports_position(true);
  else
    traits.set_supports_position(false);
  traits.set_supports_toggle(true);
  traits.set_is_assumed_state(true);
  traits.set_supports_tilt(this->supports_tilt_);
  return traits;
}

void EleroCover::set_rx_state(uint8_t state) {
  ESP_LOGV(TAG, "Got state: 0x%02x for blind 0x%06x", state, this->command_.blind_addr);
  // Bring the estimated position up to date first
  this->recompute_position();
  float pos = this->position;
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
  if((pos != this->position) || (op != this->current_operation) || (current_tilt != this->tilt)) {
    this->position = pos;
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
    this->publish_state();
    this->last_publish_ = millis();
  }
}

void EleroCover::increase_counter() {
  if(this->command_.counter == 0xff)
    this->command_.counter = 1;
  else
    this->command_.counter += 1;
}

void EleroCover::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    this->start_movement(COVER_OPERATION_IDLE);
  }
  if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    this->target_position_ = pos;
    if((pos > this->position) || (pos == COVER_OPEN)) {
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
      if(this->position == COVER_CLOSED || this->last_operation_ == COVER_OPERATION_CLOSING) {
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

  this->current_operation = dir;
  this->movement_start_ = millis();
  this->last_recompute_time_ = millis();
  this->publish_state();
}

void EleroCover::recompute_position() {
  if(this->current_operation == COVER_OPERATION_IDLE)
    return;
  if((this->open_duration_ == 0) || (this->close_duration_ == 0))
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
  this->position += dir * (now - this->last_recompute_time_) / action_dur;
  this->position = clamp(this->position, 0.0f, 1.0f);

  this->last_recompute_time_ = now;

}

} // namespace elero
} // namespace esphome
