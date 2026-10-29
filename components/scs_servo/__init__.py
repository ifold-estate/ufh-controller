"""Feetech SCS-series bus servo (e.g. Waveshare SC15) over a half-duplex UART adapter."""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, uart
from esphome.const import (
    CONF_ID,
    CONF_POSITION,
    CONF_TEMPERATURE,
    CONF_VOLTAGE,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_VOLT,
)

CODEOWNERS = ["@ifold-estate"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor"]
MULTI_CONF = True

CONF_SERVO_ID = "servo_id"
CONF_LOAD = "load"
CONF_SPEED = "speed"
CONF_RESPONSE_TIMEOUT = "response_timeout"

scs_servo_ns = cg.esphome_ns.namespace("scs_servo")
SCSServo = scs_servo_ns.class_("SCSServo", cg.PollingComponent, uart.UARTDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SCSServo),
            cv.Optional(CONF_SERVO_ID, default=1): cv.int_range(min=0, max=253),
            cv.Optional(
                CONF_RESPONSE_TIMEOUT, default="20ms"
            ): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_POSITION): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                icon="mdi:valve",
            ),
            cv.Optional(CONF_SPEED): sensor.sensor_schema(
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_LOAD): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                state_class=STATE_CLASS_MEASUREMENT,
                icon="mdi:weight",
            ),
            cv.Optional(CONF_VOLTAGE): sensor.sensor_schema(
                unit_of_measurement=UNIT_VOLT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_VOLTAGE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=0,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.polling_component_schema("1s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_servo_id(config[CONF_SERVO_ID]))
    cg.add(var.set_response_timeout(config[CONF_RESPONSE_TIMEOUT]))
    for key, setter in (
        (CONF_POSITION, "set_position_sensor"),
        (CONF_SPEED, "set_speed_sensor"),
        (CONF_LOAD, "set_load_sensor"),
        (CONF_VOLTAGE, "set_voltage_sensor"),
        (CONF_TEMPERATURE, "set_temperature_sensor"),
    ):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, setter)(sens))
