#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include <array>

namespace esphome {
namespace loctekmotion_desk_height {

// Protocol constants
static constexpr uint8_t PACKET_START_BYTE = 0x9b;
static constexpr uint8_t PACKET_END_BYTE = 0x9d;
static constexpr uint8_t HEIGHT_MESSAGE_TYPE = 0x12;
static constexpr size_t HISTORY_BUFFER_SIZE = 5;

/**
 * @brief Sensor class for reading desk height from LoctekMotion desk controller.
 * 
 * This class reads serial data from a LoctekMotion desk controller and extracts
 * the current height information from the 7-segment display data.
 */
class DeskHeightSensor : public sensor::Sensor,
                        public Component,
                        public uart::UARTDevice {
public:
    // ========== ESPHOME STANDARD METHODS ==========
    float get_setup_priority() const override { 
        return esphome::setup_priority::DATA; 
    }

    void setup() override;
    void loop() override;
    void dump_config() override;

private:
    // ========== PRIVATE METHODS ==========
    /// Convert 7-segment display byte to numeric value (0-10, where 10 is '-')
    static int8_t segment_to_digit(uint8_t segment_byte);
    
    /// Check if the decimal point is active in a segment byte
    static bool has_decimal_point(uint8_t segment_byte) { 
        return (segment_byte & 0x80) == 0x80; 
    }
    
    /// Process a complete height value from the packet
    void process_height_value(uint8_t third_digit);
    
    /// Reset the internal state for a new packet
    void reset_state();

    // ========== MEMBER VARIABLES ==========
    std::array<uint8_t, HISTORY_BUFFER_SIZE> history_ = {{0}};
    
    // Packet parsing state
    uint8_t msg_len_ = 0;
    uint8_t msg_type_ = 0;
    bool is_valid_packet_ = false;
    
    // Height tracking
    float current_height_ = 0.0f;
    float last_published_height_ = -1.0f;  // -1 means never published
};

} // namespace loctekmotion_desk_height
} // namespace esphome
