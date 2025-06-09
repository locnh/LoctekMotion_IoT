#include "desk_height_sensor.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include <bitset>

namespace esphome {
namespace loctekmotion_desk_height {

static const char *const TAG = "loctekmotion_desk_height.sensor";

// Protocol constants
static constexpr uint8_t PACKET_START_BYTE = 0x9b;
static constexpr uint8_t PACKET_END_BYTE = 0x9d;
static constexpr uint8_t HEIGHT_MESSAGE_TYPE = 0x12;
static constexpr size_t HISTORY_BUFFER_SIZE = 5;

// ========== UTILITY METHODS ==========
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
int hex_to_int(uint8_t segment_byte) {
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

/**
 * Checks if the byte indicates a decimal point is active
 * @param segment_byte The byte to check
 * @return true if the decimal point is active, false otherwise
 */
bool has_decimal_point(uint8_t segment_byte) { 
    return (segment_byte & 0x80) == 0x80; 
}

// ========== PACKET PARSING ==========
void DeskHeightSensor::reset_state() {
    this->msg_len = 0;
    this->msg_type = 0;
    this->is_valid_packet = false;
    std::fill(std::begin(this->history), std::end(this->history), 0);
}

void DeskHeightSensor::process_packet_byte(uint8_t byte) {
    // Shift history buffer
    for (int i = HISTORY_BUFFER_SIZE - 1; i > 0; i--) {
        this->history[i] = this->history[i - 1];
    }
    this->history[0] = byte;
    
    // Check for packet start
    if (byte == PACKET_START_BYTE) {
        this->reset_state();
        return;
    }
    
    // Process packet data based on position in history
    if (this->history[1] == PACKET_START_BYTE) {
        this->msg_len = byte;  // Second byte is message length
    } else if (this->history[2] == PACKET_START_BYTE) {
        this->msg_type = byte;  // Third byte is message type
    } else if (this->history[3] == PACKET_START_BYTE) {
        // Fourth byte is first height digit (for height messages)
        if (this->msg_type == HEIGHT_MESSAGE_TYPE && 
            (this->msg_len == 7 || this->msg_len == 10)) {
            this->is_valid_packet = (byte != 0 && hex_to_int(byte) != 0);
        }
    } else if (this->history[4] == PACKET_START_BYTE && this->is_valid_packet) {
        // Fifth byte is second height digit
        // No action needed, just store in history for final processing
    } else if (this->history[5] == PACKET_START_BYTE && this->is_valid_packet) {
        // Sixth byte is third height digit - process complete height value
        process_height_value(byte);
    }
}

void DeskHeightSensor::process_height_value(uint8_t third_digit) {
    const int height1 = hex_to_int(this->history[1]) * 100;  // Hundreds
    const int height2 = hex_to_int(this->history[0]) * 10;   // Tens
    const int height3 = hex_to_int(third_digit);             // Units
    
    // Skip if the tens digit is a hyphen (value 10 * 10 = 100)
    if (height2 != 100) {
        float final_height = height1 + height2 + height3;
        
        // Apply decimal point if needed (from the tens digit)
        if (has_decimal_point(this->history[0])) {
            final_height /= 10.0f;
        }
        
        this->value = final_height;
        ESP_LOGD(TAG, "Desk height updated: %.1f cm", this->value);
    }
}

// ========== MAIN LOOP ==========
void DeskHeightSensor::loop() {
    uint8_t incoming_byte;
    
    while (this->available() > 0) {
        if (this->read_byte(&incoming_byte)) {
            process_packet_byte(incoming_byte);
            
            // Check for packet end and publish if we have a new value
            if (incoming_byte == PACKET_END_BYTE && 
                this->value.has_value() && 
                this->value != this->last_published_value) {
                this->publish_state(*this->value);
                this->last_published_value = *this->value;
            }
        }
    }
}

void DeskHeightSensor::dump_config() {
    LOG_SENSOR("", "LoctekMotion Desk Height Sensor", this);
    LOG_UPDATE_INTERVAL(this);
}

} // namespace loctekmotion_desk_height
} // namespace esphome
