#include "desk_height_sensor.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace loctekmotion_desk_height {

static const char *const TAG = "loctekmotion_desk_height";

// ========== PRIVATE METHOD IMPLEMENTATIONS ==========

void DeskHeightSensor::setup() {
    // Nothing to do here for now
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
    
    // Calculate height in cm
    float height = (hundreds * 100.0f) + (tens * 10.0f) + ones;
    
    // Apply decimal point if present in the tens digit
    if (has_decimal_point(history_[0])) {
        height /= 10.0f;
    }
    
    // Update the current height
    current_height_ = height;
    ESP_LOGD(TAG, "Desk height updated: %.1f cm", current_height_);
}

void DeskHeightSensor::loop() {
    uint8_t incoming_byte;
    
    while (this->available() > 0) {
        if (!this->read_byte(&incoming_byte)) {
            continue;
        }
        
        // Shift history buffer
        std::rotate(history_.rbegin(), history_.rbegin() + 1, history_.rend());
        history_[0] = incoming_byte;
        
        // Check for packet start
        if (incoming_byte == PACKET_START_BYTE) {
            reset_state();
            continue;
        }
        
        // Process packet based on position in history
        if (history_[1] == PACKET_START_BYTE) {
            // Second byte is message length
            msg_len_ = incoming_byte;
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
            }
        }
        else if (history_[4] == PACKET_START_BYTE && is_valid_packet_) {
            // Fifth byte is second height digit
            // No action needed, just store in history for final processing
        }
        else if (incoming_byte == PACKET_END_BYTE && is_valid_packet_) {
            // End of packet - process the complete height value
            process_height_value(history_[0]);
            
            // Publish if height has changed
            if (current_height_ != last_published_height_) {
                publish_state(current_height_);
                last_published_height_ = current_height_;
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
