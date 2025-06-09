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
        default:        return -1; // Invalid pattern // Note: if 0b0100000 is segment 'f', comment should be '// f (hyphen)'
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
void DeskHeightSensor::setup() {
    // Initialization, if any, can go here.
    // UART setup is handled by the base UARTDevice class.
    this->reset_state(); // Ensure a clean state on setup.
}

void DeskHeightSensor::reset_state() {
    this->msg_len_ = 0;
    this->msg_type_ = 0;
    this->is_valid_packet_ = false;
    // Do NOT clear history_ here, it's a sliding window.
}

void DeskHeightSensor::process_packet_byte(uint8_t byte) {
    // Shift history buffer
    for (int i = HISTORY_BUFFER_SIZE - 1; i > 0; i--) {
        this->history_[i] = this->history_[i - 1];
    }
    this->history_[0] = byte;
    
    // Check for packet start
    if (byte == PACKET_START_BYTE) {
        // Reset packet metadata, history_[0] now holds the START_BYTE
        this->reset_state(); 
        return;
    }
    
    // Process packet data based on position in history
    if (this->history_[1] == PACKET_START_BYTE) {
        this->msg_len_ = byte;  // Second byte is message length
    } else if (this->history_[2] == PACKET_START_BYTE) {
        this->msg_type_ = byte;  // Third byte is message type
    } else if (this->history_[3] == PACKET_START_BYTE) {
        // Fourth byte is first height digit (for height messages)
        if (this->msg_type_ == HEIGHT_MESSAGE_TYPE &&
            (this->msg_len_ == 7 || this->msg_len_ == 10)) {
            this->is_valid_packet_ = (byte != 0 && hex_to_int(byte) != 0);
            if (!this->is_valid_packet_) {
                ESP_LOGD(TAG, "Invalid D1: byte=0x%02X, val=%d", byte, hex_to_int(byte));
            }
        }
    } else if (this->history_[4] == PACKET_START_BYTE && this->is_valid_packet_) {
        // Current byte is D2 (fifth byte after START_BYTE)
        // No specific action, D2 is stored in history_[0]
    } else if (this->history_[5] == PACKET_START_BYTE && this->is_valid_packet_) {
        // Current byte is D3 (sixth byte after START_BYTE) - process complete height value
        process_height_value(byte);
    }
}

void DeskHeightSensor::process_height_value(uint8_t d3_byte_arg) {
    // When this is called:
    // d3_byte_arg is history_[0] (current byte, D3)
    // history_[1] is D2
    // history_[2] is D1
    const int d1_val = hex_to_int(this->history_[2]); // Hundreds
    const int d2_val = hex_to_int(this->history_[1]); // Tens
    const int d3_val = hex_to_int(d3_byte_arg);       // Units
    
    // Validate digits: D1 must be 1-9. D2, D3 must be 0-9. D2 can be 10 (hyphen).
    if (d1_val >= 1 && d1_val <= 9 && d2_val >= 0 && d2_val <= 10 && d3_val >= 0 && d3_val <= 9) {
        if (d2_val == 10) { // Tens digit is a hyphen
            ESP_LOGD(TAG, "Hyphen in tens place (D2 from 0x%02X), skipping height update.", this->history_[1]);
            return;
        }
        float final_height = (d1_val * 100) + (d2_val * 10) + d3_val;
        
        // Apply decimal point if needed (from the D2 byte)
        if (has_decimal_point(this->history_[1])) {
            final_height /= 10.0f;
        }
        
        this->value_data_ = final_height;
        this->has_value_ = true;
        ESP_LOGD(TAG, "Desk height updated: %.1f cm", this->value_data_);
    }
    else {
      ESP_LOGD(TAG, "Invalid digits for height: D1=%d (0x%02X), D2=%d (0x%02X), D3=%d (0x%02X)",
                d1_val, this->history_[2], d2_val, this->history_[1], d3_val, d3_byte_arg);
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
                this->has_value_ &&
                (!this->has_last_published_value_ || this->value_data_ != this->last_published_value_data_)) {
                this->publish_state(this->value_data_);
                this->last_published_value_data_ = this->value_data_;
                this->has_last_published_value_ = true;
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
