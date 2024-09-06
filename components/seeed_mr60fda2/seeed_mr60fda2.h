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
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/helpers.h"

#include <map>

namespace esphome {
namespace seeed_mr60fda2 {

static const uint8_t DATA_BUF_MAX_SIZE = 16;
static const uint8_t FRAME_BUF_MAX_SIZE = 37;
static const uint8_t SIZE_FRAME_HEADER = 8;
static const uint8_t HEAD_CKSUM_LEN = 8;

static const uint8_t FRAME_HEADER_BUFFER = 0x01;
static const uint16_t IS_FALL_TYPE_BUFFER = 0x0E02;
static const uint16_t PEOPLE_EXIST_TYPE_BUFFER = 0x0F09;

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

static const char *const INSTALL_HEIGHT[7] = {"2.4m", "2.5m", "2.6m", "2.7m", "2.8m", "2.9m", "3.0m"};
static const char *const HEIGHT_THRESHOLD[7] = {"0m", "0.1m", "0.2m", "0.3m", "0.4m", "0.5m", "0.6m"};
static const char *const SENSITIVITY[3] = {"1", "2", "3"};

class MR60FDA2Component : public Component,
                          public uart::UARTDevice {  // The class name must be the name defined by text_sensor.py
#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(is_fall)
  SUB_BINARY_SENSOR(people_exist)
#endif
#ifdef USE_BUTTON
  SUB_BUTTON(get_radar_parameters)
#endif
#ifdef USE_SELECT
  SUB_SELECT(set_install_height)
  SUB_SELECT(set_height_threshold)
  SUB_SELECT(set_sensitivity)
#endif

 protected:
  uint8_t current_frame_locate_;
  uint8_t current_frame_buf[FRAME_BUF_MAX_SIZE];
  uint8_t current_data_buf[DATA_BUF_MAX_SIZE];
  size_t current_frame_len_;
  uint16_t current_frame_id_;
  uint16_t current_data_frame_len_;
  uint16_t current_frame_type_;

  bool validateChecksum(const uint8_t *data, size_t len, uint8_t expected_checksum);
  uint8_t calculateChecksum(const uint8_t *data, size_t len);
  void splitFrame(uint8_t buffer);
  void processFrame();
  // void send_query_(uint8_t *query, size_t string_length);

 public:
  float get_setup_priority() const override { return esphome::setup_priority::LATE; }
  void setup() override;
  void dump_config() override;
  void loop() override;
};

}  // namespace seeed_mr60fda2
}  // namespace esphome
