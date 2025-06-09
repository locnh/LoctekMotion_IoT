#include "desk_height_sensor.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace loctekmotion_desk_height {

static const char *const TAG = "loctekmotion_desk_height";

// ========== PRIVATE METHOD IMPLEMENTATIONS ==========
void DeskHeightSensor::setup() {
    // Initialize UART
    this->set_baud_rate(9600);
    this->set_rx_buffer_size(64);
    this->set_stop_bits(1);
    this->set_data_bits(8);
    this->set_parity(UART_CONFIG_PARITY_NONE);
    this->reset_state();
}

void DeskHeightSensor::reset_state() {
    msg_len_ = 0;
    msg_type_ = 0;
    is_valid_packet_ = false;
    std::fill(history_.begin(), history_.end(), 0);
}

/**
 * @brief Convert a 7-segment display pattern byte to its corresponding digit
 * 
 * This function decodes the segment pattern from a 7-segment display byte.
 * The segments are mapped to bits in the following order (LSB to MSB):
 *
 *    -- a --    Bit 0: segment a (bottom)
 *   |       |   Bit 1: segment b (lower right)
 *   f       b   Bit 2: segment c (upper right)
 *   |       |   Bit 3: segment d (top)
 *    -- g --    Bit 4: segment e (upper left)
 *   |       |   Bit 5: segment f (lower left)
 *   e       c   Bit 6: segment g (middle)
 *   |       | 
 *    -- d --
 *
 * For example, the digit '0' is represented by lighting up segments a-f (0b0111111).
 * The decimal point (DP) is ignored as it's masked out (bit 7).
 * 
 * @param segment_byte The byte containing the segment pattern (bit 7 = DP, bits 6-0 = segments g-a)
 * @return int8_t The decoded digit (0-9), 10 for hyphen (-), or -1 for invalid pattern
 */
int8_t DeskHeightSensor::segment_to_digit(uint8_t segment_byte) {
    // Mask out the decimal point bit (MSB) to get just the segment pattern
    uint8_t pattern = segment_byte & 0x7F;
    
    // Convert 7-segment pattern to digit (0-10, where 10 is '-')
    switch (pattern) {
        case 0b0111111: return 0;  // abcdef
        case 0b0000110: return 1;  // bc
        case 0b1011011: return 2;  // abdeg
        case 0b1001111: return 3;  // abcdg
        case 0b1100110: return 4;  // bcfg
        case 0b1101101: return 5;  // acdfg
        case 0b1111101: return 6;  // acdefg
        case 0b0000111: return 7;  // abc
        case 0b1111111: return 8;  // abcdefg
        case 0b1101111: return 9;  // abcdfg
        case 0b0100000: return 10; // g (hyphen)
        default:        return -1; // Invalid pattern
    }
}

void DeskHeightSensor::process_height_value(uint8_t third_digit) {
    // Convert each segment to its numeric value
    const int8_t hundreds = segment_to_digit(history_[1]);
    const int8_t tens = segment_to_digit(history_[0]);
    const int8_t ones = segment_to_digit(third_digit);
    
    // Validate digit conversions
    if (hundreds < 0 || tens < 0 || ones < 0) {
        ESP_LOGD(TAG, "Invalid segment pattern in height value");
        return;
    }
    
    // Skip if tens digit is a hyphen (treated as invalid for height)
    if (tens == 10) {
        ESP_LOGD(TAG, "Skipping height update - hyphen in tens place");
        return;
    }
    
    // Calculate height in cm (using integer math first for precision)
    int32_t height_x10;
    
    if (has_decimal_point(history_[0])) {
        // Decimal point is after tens place (e.g., "12.3" -> 12.3 cm)
        // hundreds = 1, tens = 2, ones = 3 -> 12.3 cm
        height_x10 = (hundreds * 100) + (tens * 10) + ones;  // 1*100 + 2*10 + 3 = 123
    } else {
        // No decimal point (e.g., "123" -> 123.0 cm)
        // hundreds = 1, tens = 2, ones = 3 -> 123.0 cm
        height_x10 = (hundreds * 1000) + (tens * 100) + (ones * 10);  // 1*1000 + 2*100 + 3*10 = 1230
    }
    
    // Convert to float (dividing by 10 to get cm from tenths of cm)
    current_height_ = height_x10 / 10.0f;
    
    ESP_LOGD(TAG, "Desk height updated: %.1f cm", current_height_);
}

void DeskHeightSensor::loop() {
    uint8_t incoming_byte;
    
    while (this->available() > 0) {
        if (!this->read_byte(&incoming_byte)) {
            continue;
        }
        
        // Shift history buffer (using rotate is efficient for small buffers)
        std::rotate(history_.rbegin(), history_.rbegin() + 1, history_.rend());
        history_[0] = incoming_byte;
        
        // Check for packet start
        if (incoming_byte == PACKET_START_BYTE) {
            reset_state();
            continue;
        }
        
        // Only process if we have a valid packet start in history
        if (history_[1] == PACKET_START_BYTE) {
            // Second byte is message length
            msg_len_ = incoming_byte;
            is_valid_packet_ = false;  // Reset until we validate the packet
        } 
        else if (history_[2] == PACKET_START_BYTE) {
            // Third byte is message type
            msg_type_ = incoming_byte;
        } 
        else if (history_[3] == PACKET_START_BYTE) {
            // Fourth byte is first height digit
            if (msg_type_ == HEIGHT_MESSAGE_TYPE && 
                (msg_len_ == 7 || msg_len_ == 10)) {
                is_valid_packet_ = (incoming_byte != 0 && segment_to_digit(incoming_byte) >= 0);
                if (!is_valid_packet_) {
                    ESP_LOGD(TAG, "Invalid first height digit: 0x%02X", incoming_byte);
                }
            } else {
                is_valid_packet_ = false;
            }
        }
        else if (history_[4] == PACKET_START_BYTE && is_valid_packet_) {
            // Fifth byte is second height digit
            // No action needed, just store in history for final processing
        }
        else if (incoming_byte == PACKET_END_BYTE && is_valid_packet_) {
            // End of packet - process the complete height value
            process_height_value(history_[0]);
            
            // Publish if height has changed (with hysteresis to prevent noise)
            const float height_change = fabsf(current_height_ - last_published_height_);
            if (height_change >= 0.1f) {  // 1mm threshold
                publish_state(current_height_);
                last_published_height_ = current_height_;
                ESP_LOGD(TAG, "Published new height: %.1f cm", current_height_);
            }
        }
    }
}

void DeskHeightSensor::dump_config() {
    ESP_LOGCONFIG(TAG, "LoctekMotion Desk Height Sensor:");
    LOG_UPDATE_INTERVAL(this);
}

} // namespace loctekmotion_desk_height
} // namespace esphome
