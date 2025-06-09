"""
LoctekMotion Desk Height Sensor component for ESPHome.

This component reads the height of a LoctekMotion desk from its display controller
over UART and exposes it as a sensor in Home Assistant.
"""
from __future__ import annotations

import esphome.codegen as cg
from esphome.components import sensor, uart
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_DISTANCE,
    ICON_ARROW_EXPAND_VERTICAL,
    STATE_CLASS_MEASUREMENT,
    UNIT_CENTIMETER,
)

# Component metadata
CODEOWNERS = ["@locnh"]
DEPENDENCIES = ["uart"]

# Namespace and class definition
loctekmotion_ns = cg.esphome_ns.namespace("loctekmotion_desk_height")
DeskHeightSensor = loctekmotion_ns.class_(
    "DeskHeightSensor", sensor.Sensor, cg.Component, uart.UARTDevice
)

# Configuration schema
CONFIG_SCHEMA = sensor.sensor_schema(
    DeskHeightSensor,
    unit_of_measurement=UNIT_CENTIMETER,
    icon=ICON_ARROW_EXPAND_VERTICAL,
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
    device_class=DEVICE_CLASS_DISTANCE,
).extend(uart.UART_DEVICE_SCHEMA)

# UART configuration validation
FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema(
    "loctekmotion_desk_height",
    baud_rate=9600,
    require_tx=False,      # Only RX is needed as we only read height data
    require_rx=True,
    data_bits=8,
    parity=None,
    stop_bits=1,
)


async def to_code(config: dict) -> None:
    """Generate code for the desk height sensor.
    
    Args:
        config: The configuration dictionary from ESPHome.
    """
    var = cg.new_Pvariable(config[CONF_ID])
    await sensor.register_sensor(var, config)
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
