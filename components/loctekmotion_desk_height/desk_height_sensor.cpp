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
 * Converts a 7-segment display byte to its numeric value
 * @param segment_byte The byte representing the 7-segment display
 * @return The numeric value (0-10) where 10 represents a hyphen (-)
 */
int hex_to_int(uint8_t segment_byte) {
    const std::bitset<8> segments(segment_byte);
    
    // Check for each digit pattern (0-9 and -)
    if (segments[0] && segments[1] && segments[2] && segments[3] && segments[4] && segments[5] && !segments[6]) return 0;
    if (!segments[0] && segments[1] && segments[2] && !segments[3] && !segments[4] && !segments[5] && !segments[6]) return 1;
    if (segments[0] && segments[1] && !segments[2] && segments[3] && segments[4] && !segments[5] && segments[6]) return 2;
    if (segments[0] && segments[1] && segments[2] && segments[3] && !segments[4] && !segments[5] && segments[6]) return 3;
    if (!segments[0] && segments[1] && segments[2] && !segments[3] && !segments[4] && segments[5] && segments[6]) return 4;
    if (segments[0] && !segments[1] && segments[2] && segments[3] && !segments[4] && segments[5] && segments[6]) return 5;
    if (segments[0] && !segments[1] && segments[2] && segments[3] && segments[4] && segments[5] && segments[6]) return 6;
    if (segments[0] && segments[1] && segments[2] && !segments[3] && !segments[4] && !segments[5] && !segments[6]) return 7;
    if (segments[0] && segments[1] && segments[2] && segments[3] && segments[4] && segments[5] && segments[6]) return 8;
    if (segments[0] && segments[1] && segments[2] && segments[3] && !segments[4] && segments[5] && segments[6]) return 9;
    if (!segments[0] && !segments[1] && !segments[2] && !segments[3] && !segments[4] && !segments[5] && segments[6]) return 10; // Hyphen (-)
    
    return 0; // Default to 0 for invalid patterns
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
