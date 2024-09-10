#pragma once
#include "esphome/core/defines.h"
#include "esphome/core/component.h"
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_BUTTON
#include "esphome/components/button/button.h"
#endif
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

#include <map>

namespace esphome {
namespace seeed_mr60fda2 {

static const uint8_t DATA_BUF_MAX_SIZE = 16;
static const uint8_t FRAME_BUF_MAX_SIZE = 37;
static const uint8_t LEN_TO_HEAD_CKSUM = 8;
static const uint8_t LEN_TO_DATA_FRAME = 9;

static const uint8_t FRAME_HEADER_BUFFER = 0x01;
static const uint16_t IS_FALL_TYPE_BUFFER = 0x0E02;
static const uint16_t PEOPLE_EXIST_TYPE_BUFFER = 0x0F09;
static const uint16_t RUSULT_INSTALL_HEIGHT = 0x0E04;
static const uint16_t RUSULT_PARAMETERS = 0x0E06;
static const uint16_t RUSULT_HEIGHT_THRESHOLD = 0x0E08;
static const uint16_t RUSULT_SENSITIVITY = 0x0E0A;

enum FrameLocation {
  LOCATE_FRAME_HEADER,
  LOCATE_ID_FRAME1,
  LOCATE_ID_FRAME2,
  LOCATE_LENGTH_FRAME_H,
  LOCATE_LENGTH_FRAME_L,
  LOCATE_TYPE_FRAME1,
  LOCATE_TYPE_FRAME2,
  LOCATE_HEAD_CKSUM_FRAME,  // Header checksum: [from the first byte to the previous byte of the HEAD_CKSUM bit]
  LOCATE_DATA_FRAME,
  LOCATE_DATA_CKSUM_FRAME,  // Data checksum: [from the first to the previous byte of the DATA_CKSUM bit]
  LOCATE_PROCESS_FRAME,
};

static const float INSTALL_HEIGHT[7] = {2.4f, 2.5f, 2.6f, 2.7f, 2.8f, 2.9f, 3.0f};
static const float HEIGHT_THRESHOLD[7] = {0.0f, 0.1f, 0.2f, 0.3f, 0.4f, 0.5f, 0.6f};
static const uint8_t SENSITIVITY[3] = {3, 15, 30};

class MR60FDA2Component : public Component,
                          public uart::UARTDevice {  // The class name must be the name defined by text_sensor.py
#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(people_exist)
#endif
#ifdef USE_BUTTON
  SUB_BUTTON(get_radar_parameters)
  SUB_BUTTON(reset_radar)
#endif
#ifdef USE_SELECT
  SUB_SELECT(install_height)
  SUB_SELECT(height_threshold)
  SUB_SELECT(sensitivity)
#endif
#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(is_fall)
#endif

 protected:
  uint8_t current_frame_locate_;
  uint8_t current_frame_buf[FRAME_BUF_MAX_SIZE];
  uint8_t current_data_buf[DATA_BUF_MAX_SIZE];
  uint16_t current_frame_id_;
  size_t current_frame_len_;
  size_t current_data_frame_len_;
  uint16_t current_frame_type_;
  uint32_t current_install_height_;
  uint32_t current_height_threshold_;
  uint32_t current_sensitivity_;

  bool validateChecksum(const uint8_t *data, size_t len, uint8_t expected_checksum);
  uint8_t calculateChecksum(const uint8_t *data, size_t len);
  void splitFrame(uint8_t buffer);
  void processFrame();
  void send_query_(uint8_t *query, size_t string_length);
  void float_to_bytes(float value, unsigned char *bytes);
  void int_to_bytes(uint32_t value, unsigned char *bytes);

 public:
  float get_setup_priority() const override { return esphome::setup_priority::LATE; }
  void setup() override;
  void dump_config() override;
  void loop() override;
  void set_install_height(uint8_t index);
  void set_height_threshold(uint8_t index);
  void set_sensitivity(uint8_t index);
  void get_radar_parameters();
  void reset_radar();
};

}  // namespace seeed_mr60fda2
}  // namespace esphome
