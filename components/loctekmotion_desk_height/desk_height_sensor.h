#pragma once

#include <array>

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace loctekmotion_desk_height {

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
    // ========== CONFIGURATION ==========
    float get_setup_priority() const override { 
        return esphome::setup_priority::DATA; 
    }

    // ========== COMPONENT LIFECYCLE ==========
    void setup() override;
    void loop() override;
    void dump_config() override;

private:
    // ========== PACKET PROCESSING ==========
    /// Reset the internal state for a new packet
    void reset_state();
    
    /// Process a single byte of the incoming packet
    void process_packet_byte(uint8_t byte);
    
    /// Process the complete height value from the packet
    void process_height_value(uint8_t third_digit);

    // ========== CHILD LOCK DETECTION ==========
    /// Check if the current history buffer contains the child lock pattern (LoC)
    bool is_child_lock_active() const;
    
    // ========== MEMBER VARIABLES ==========
    // Needs to be 6 to access history_[5] for PACKET_START_BYTE check when processing 6th data byte (D3)
    static constexpr size_t HISTORY_BUFFER_SIZE = 6;
    
    /// Last calculated height value (in cm)
    float value_data_{0.0f};
    bool has_value_{false};
    bool child_lock_active_{false};
    
    /// Last published height value (to avoid duplicate updates)
    float last_published_value_data_{0.0f};
    bool has_last_published_value_{false};
    
    /// Buffer to track the last few bytes for packet parsing
    std::array<uint8_t, HISTORY_BUFFER_SIZE> history_ = {0};
    
    /// Current message length from packet header
    uint8_t msg_len_ = 0;
    
    /// Current message type from packet header
    uint8_t msg_type_ = 0;
    
    /// Flag indicating if current packet is valid
    bool is_valid_packet_ = false;
};

} // namespace loctekmotion_desk_height
} // namespace esphome
