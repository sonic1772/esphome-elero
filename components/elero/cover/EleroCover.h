#pragma once

#include "esphome/core/component.h"
#include "esphome/core/automation.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/elero/elero.h"
#include <queue>

namespace esphome {
namespace elero {

class EleroCover : public cover::Cover, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override;
  
  cover::CoverTraits get_traits() override;
  
  void set_elero_parent(Elero *parent) { this->parent_ = parent; }
  void set_blind_address(uint32_t address) { this->command_.blind_addr = address; }
  void set_channel(uint8_t channel) { this->command_.channel = channel; }
  void set_remote_address(uint32_t remote) { this->command_.remote_addr = remote; }
  void set_payload_1(uint8_t payload) { this->command_.payload[0] = payload; }
  void set_payload_2(uint8_t payload) { this->command_.payload[1] = payload; }
  void set_hop(uint8_t hop) { this->command_.hop = hop; }
  void set_pckinf_1(uint8_t pckinf) { this->command_.pck_inf[0] = pckinf; }
  void set_pckinf_2(uint8_t pckinf) { this->command_.pck_inf[1] = pckinf; }
  void set_command_up(uint8_t cmd) { this->command_up_ = cmd; }
  void set_command_down(uint8_t cmd) { this->command_down_ = cmd; }
  void set_command_stop(uint8_t cmd) { this->command_stop_ = cmd; }
  void set_command_check(uint8_t cmd) { this->command_check_ = cmd; }
  void set_command_tilt(uint8_t cmd) { this->command_tilt_ = cmd; }
  void set_command_intermediate(uint8_t cmd) { this->command_intermediate_ = cmd; }
  void set_intermediate_position(float pos) { this->intermediate_position_ = pos; }
  void set_ventilation_position(float pos) { this->ventilation_position_ = pos; }
  void set_lamella_duration(uint32_t dur) { this->lamella_duration_ = dur; }
  void set_poll_offset(uint32_t offset) { this->poll_offset_ = offset; }
  void set_close_duration(uint32_t dur) { this->close_duration_ = dur; }
  void set_open_duration(uint32_t dur) { this->open_duration_ = dur; }
  void set_poll_interval(uint32_t intvl) { this->poll_intvl_ = intvl; }
  uint32_t get_blind_address() { return this->command_.blind_addr; }
  uint32_t get_remote_address() { return this->command_.remote_addr; }
  void set_supports_tilt(bool tilt) { this->supports_tilt_ = tilt; }
  void set_rx_state(uint8_t state);
  void handle_commands(uint32_t now);
  void recompute_position();
  void start_movement(cover::CoverOperation op);
  bool is_at_target();
  // Command seen on air from the original remote (same remote address as ours)
  void handle_remote_command(uint8_t cmd);
  // Drive to the positions stored in the motor (Zwischenposition / Lueftungsposition)
  void move_to_intermediate();
  void move_to_ventilation();
  
 protected:
  void control(const cover::CoverCall &call) override;
  void increase_counter();
  void track_movement(cover::CoverOperation op, float target);
  void start_special_movement(float target);
  void set_idle_and_publish();
  bool has_durations_();
  float lamella_raw_();
  float to_display_(float raw);
  float to_raw_(float display);
  void publish_(bool save = true);
  bool try_special_position_(float display_target);

  t_elero_command command_ = {
    .counter = 1,
  };
  Elero *parent_;
  uint32_t last_poll_{0};
  uint32_t last_command_{0};
  uint32_t poll_offset_{0};
  uint32_t movement_start_{0};
  uint32_t open_duration_{0};
  uint32_t close_duration_{0};
  uint32_t last_publish_{0};
  uint32_t last_recompute_time_{0};
  uint32_t poll_intvl_{0};
  float target_position_{0};
  bool supports_tilt_{false};
  uint8_t command_up_{0x20};
  uint8_t command_down_{0x40};
  uint8_t command_check_{0x00};
  uint8_t command_stop_{0x10};
  uint8_t command_tilt_{0x24};
  uint8_t command_intermediate_{0x44};
  // Positions stored in the motor as shown in Home Assistant, < 0 = not configured
  float intermediate_position_{-1.0f};
  float ventilation_position_{-1.0f};
  // Raw target of a movement the motor stops by itself (special position), < 0 = none
  float special_target_{-1.0f};
  // Travel-time based position (0 = closed, 1 = open). `position` is what is
  // published and may be mapped (see lamella_duration)
  float raw_position_{0.0f};
  uint32_t lamella_duration_{0};
  uint32_t end_reached_time_{0};
  std::queue<uint8_t> commands_to_send_;
  uint8_t send_retries_{0};
  uint8_t send_packets_{0};
  cover::CoverOperation last_operation_{cover::COVER_OPERATION_OPENING};
};

} // namespace elero
} // namespace esphome

