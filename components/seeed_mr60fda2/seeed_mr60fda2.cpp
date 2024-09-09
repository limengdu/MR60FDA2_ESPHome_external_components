#include "esphome/core/log.h"
#include "seeed_mr60fda2.h"

#include <utility>

namespace esphome {
namespace seeed_mr60fda2 {

static const char *const TAG = "seeed_mr60fda2";

// Prints the component's configuration data. dump_config() prints all of the component's configuration
// items in an easy-to-read format, including the configuration key-value pairs.
void MR60FDA2Component::dump_config() {
  ESP_LOGCONFIG(TAG, "MR60FDA2:");
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR(" ", "Is Fall Binary Sensor", this->is_fall_binary_sensor_);
  LOG_BINARY_SENSOR(" ", "People Exist Binary Sensor", this->people_exist_binary_sensor_);
#endif
#ifdef USE_BUTTON
  LOG_BUTTON(" ", "Get Radar Parameters Button", this->get_radar_parameters_button_);
#endif
#ifdef USE_SELECT
  LOG_SELECT(" ", "Install Height Select", this->install_height_select_);
  LOG_SELECT(" ", "Height Threshold Select", this->height_threshold_select_);
  LOG_SELECT(" ", "Sensitivity Select", this->sensitivity_select_);
#endif
}

// Initialisation functions
void MR60FDA2Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up MR60FDA2...");
  this->check_uart_settings(115200);

  this->current_frame_locate_ = LOCATE_FRAME_HEADER;
  this->current_frame_id_ = 0;
  this->current_frame_len_ = 0;
  this->current_data_frame_len_ = 0;
  this->current_frame_type_ = 0;

  memset(this->current_frame_buf, 0, FRAME_BUF_MAX_SIZE);
  memset(this->current_data_buf, 0, DATA_BUF_MAX_SIZE);

  ESP_LOGCONFIG(TAG, "Set up MR60FDA2 complete");
}

// main loop
void MR60FDA2Component::loop() {
  uint8_t byte;
  // uint8_t array[7];

  // Is there data on the serial port
  while (this->available()) {
    // this->read_array(&array, 7)
    this->read_byte(&byte);
    this->splitFrame(byte);  // split data frame
  }
}

/**
 * @brief Calculate the checksum for a byte array.
 *
 * This function calculates the checksum for the provided byte array using an
 * XOR-based checksum algorithm.
 *
 * @param data The byte array to calculate the checksum for.
 * @param len The length of the byte array.
 * @return The calculated checksum.
 */
uint8_t MR60FDA2Component::calculateChecksum(const uint8_t *data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  checksum = ~checksum;
  return checksum;
}

/**
 * @brief Validate the checksum of a byte array.
 *
 * This function validates the checksum of the provided byte array by comparing
 * it to the expected checksum.
 *
 * @param data The byte array to validate.
 * @param len The length of the byte array.
 * @param expected_checksum The expected checksum.
 * @return True if the checksum is valid, false otherwise.
 */
bool MR60FDA2Component::validateChecksum(const uint8_t *data, size_t len, uint8_t expected_checksum) {
  return calculateChecksum(data, len) == expected_checksum;
}

void MR60FDA2Component::splitFrame(uint8_t buffer) {
  switch (this->current_frame_locate_) {
    case LOCATE_FRAME_HEADER:  // starting buffer
      if (FRAME_HEADER_BUFFER == buffer) {
        this->current_frame_len_ = 1;
        this->current_frame_buf[this->current_frame_len_ - 1] = buffer;
        this->current_frame_locate_++;
        // ESP_LOGD(TAG, "%x", this->current_frame_buf[this->current_frame_len_ - 1]);
      } else {
        this->current_frame_locate_ = LOCATE_FRAME_HEADER;
        // ESP_LOGD(TAG, "FRAME_HEADER_BUFFER ERROR buffer:%x", buffer);
      }
      break;
    case LOCATE_ID_FRAME1:
      this->current_frame_id_ = buffer << 8;
      this->current_frame_len_++;
      this->current_frame_buf[this->current_frame_len_ - 1] = buffer;
      this->current_frame_locate_++;
      // ESP_LOGD(TAG, "%x", this->current_frame_buf[this->current_frame_len_ - 1]);
      break;
    case LOCATE_ID_FRAME2:
      this->current_frame_id_ += buffer;
      this->current_frame_len_++;
      this->current_frame_buf[this->current_frame_len_ - 1] = buffer;
      this->current_frame_locate_++;
      // ESP_LOGD(TAG, "%x", this->current_frame_buf[this->current_frame_len_ - 1]);
      break;
    case LOCATE_LENGTH_FRAME_H:
      this->current_data_frame_len_ = buffer << 8;
      this->current_frame_len_++;
      this->current_frame_buf[this->current_frame_len_ - 1] = buffer;
      this->current_frame_locate_++;
      // ESP_LOGD(TAG, "%x", this->current_frame_buf[this->current_frame_len_ - 1]);
      break;
    case LOCATE_LENGTH_FRAME_L:
      this->current_data_frame_len_ += buffer;
      if (this->current_data_frame_len_ > DATA_BUF_MAX_SIZE) {
        // ESP_LOGD(TAG, "DATA_FRAME_LEN ERROR: %d", this->current_data_frame_len_);
        this->current_frame_locate_ = LOCATE_FRAME_HEADER;
      } else {
        this->current_frame_len_++;
        this->current_frame_buf[this->current_frame_len_ - 1] = buffer;
        this->current_frame_locate_++;
        // ESP_LOGD(TAG, "%x", this->current_frame_buf[this->current_frame_len_ - 1]);
      }
      break;
    case LOCATE_TYPE_FRAME1:
      this->current_frame_type_ = buffer << 8;
      this->current_frame_len_++;
      this->current_frame_buf[this->current_frame_len_ - 1] = buffer;
      this->current_frame_locate_++;
      // ESP_LOGD(TAG, "%x", this->current_frame_buf[this->current_frame_len_ - 1]);
      break;
    case LOCATE_TYPE_FRAME2:
      this->current_frame_type_ += buffer;
      if ((this->current_frame_type_ == IS_FALL_TYPE_BUFFER) ||
          (this->current_frame_type_ == PEOPLE_EXIST_TYPE_BUFFER)) {
        this->current_frame_len_++;
        this->current_frame_buf[this->current_frame_len_ - 1] = buffer;
        this->current_frame_locate_++;
        // ESP_LOGD(TAG, "%x", this->current_frame_buf[this->current_frame_len_ - 1]);
      } else {
        // ESP_LOGD(TAG, "CURRENT_FRAME_TYPE NOT FOUND: %x", this->current_frame_type_);
        this->current_frame_locate_ = LOCATE_FRAME_HEADER;
      }
      break;
    case LOCATE_HEAD_CKSUM_FRAME:
      this->current_frame_len_++;
      this->current_frame_buf[this->current_frame_len_ - 1] = buffer;
      if (this->validateChecksum(this->current_frame_buf, this->current_frame_len_,
                                 this->current_frame_buf[current_frame_len_ - 1])) {
        this->current_frame_locate_++;
        ESP_LOGD(TAG, "GET HEAD_CKSUM_FRAME: %x%x", this->current_frame_buf[this->current_frame_len_ - 2], this->current_frame_buf[this->current_frame_len_ - 1]);
      } else {
        ESP_LOGD(TAG, "HEAD_CKSUM_FRAME ERROR: %x%x", this->current_frame_buf[this->current_frame_len_ - 2], this->current_frame_buf[this->current_frame_len_ - 1]);
        this->current_frame_locate_ = LOCATE_FRAME_HEADER;
      }
      break;
    case LOCATE_DATA_FRAME:
      this->current_frame_len_++;
      this->current_frame_buf[this->current_frame_len_ - 1] = buffer;
      if (this->current_frame_len_ - HEAD_CKSUM_LEN == this->current_data_frame_len_) {
        this->current_frame_locate_++;
        // ESP_LOGD(TAG, "%x", this->current_frame_buf[this->current_frame_len_ - 1]);
        // ESP_LOGD(TAG, "9");
      }
      if (this->current_frame_len_ > FRAME_BUF_MAX_SIZE) {
        ESP_LOGD(TAG, "PRACTICE_DATA_FRAME_LEN ERROR: %d", this->current_frame_len_ - HEAD_CKSUM_LEN);
        this->current_frame_locate_ = LOCATE_FRAME_HEADER;
      }
      break;
    case LOCATE_DATA_CKSUM_FRAME:
      this->current_frame_len_++;
      this->current_frame_buf[this->current_frame_len_ - 1] = buffer;
      if (validateChecksum(this->current_frame_buf, this->current_frame_len_,
                           this->current_frame_buf[this->current_frame_len_ - 1])) {
        this->current_frame_locate_++;
        this->processFrame();
        // ESP_LOGD(TAG, "%x", this->current_frame_buf[this->current_frame_len_ - 1]);
        // ESP_LOGD(TAG, "10");
      } else {
        ESP_LOGD(TAG, "DATA_CKSUM_FRAME ERROR: %x", this->current_frame_buf[current_frame_len_ - 1]);
        this->current_frame_locate_ = LOCATE_FRAME_HEADER;
      }
      break;
    default:
      break;
  }
}

void MR60FDA2Component::processFrame() {
  switch (this->current_frame_type_) {
    case IS_FALL_TYPE_BUFFER:
      if (this->is_fall_binary_sensor_ != nullptr) {
        this->is_fall_binary_sensor_->publish_state(this->current_frame_buf[HEAD_CKSUM_LEN]);
        this->current_frame_locate_ = LOCATE_FRAME_HEADER;
        ESP_LOGD(TAG, "Succeed get fall info");
      }
      break;
    case PEOPLE_EXIST_TYPE_BUFFER:
      if (this->people_exist_binary_sensor_ != nullptr) {
        this->people_exist_binary_sensor_->publish_state(this->current_frame_buf[HEAD_CKSUM_LEN]);
        this->current_frame_locate_ = LOCATE_FRAME_HEADER;
        ESP_LOGD(TAG, "Succeed get people exist info");
      }
      break;
    default:
      break;
  }
}

// // Sending data frames
// void MR60FDA2Component::send_query_(uint8_t *query, size_t string_length) { this->write_array(query, string_length);
// }

// // Send Heartbeat Packet Command
// void MR60FDA2Component::get_heartbeat_packet() {
//   uint8_t send_data_len = 10;
//   uint8_t send_data[10] = {0x53, 0x59, 0x01, 0x01, 0x00, 0x01, 0x0F, 0x00, 0x54, 0x43};
//   send_data[7] = get_frame_crc_sum(send_data, send_data_len);
//   this->send_query_(send_data, send_data_len);
// }

}  // namespace seeed_mr60fda2
}  // namespace esphome
