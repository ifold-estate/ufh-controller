import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_MAX_VALUE, CONF_MIN_VALUE, CONF_STEP

from . import SCSServo, scs_servo_ns

CONF_SCS_SERVO_ID = "scs_servo_id"

SCSGoalNumber = scs_servo_ns.class_("SCSGoalNumber", number.Number, cg.Component)

CONFIG_SCHEMA = number.number_schema(
    SCSGoalNumber, icon="mdi:valve", entity_category="config"
).extend(
    {
        cv.GenerateID(CONF_SCS_SERVO_ID): cv.use_id(SCSServo),
        cv.Optional(CONF_MIN_VALUE, default=0): cv.int_range(0, 1023),
        cv.Optional(CONF_MAX_VALUE, default=1023): cv.int_range(0, 1023),
        cv.Optional(CONF_STEP, default=1): cv.positive_int,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = await number.new_number(
        config,
        min_value=config[CONF_MIN_VALUE],
        max_value=config[CONF_MAX_VALUE],
        step=config[CONF_STEP],
    )
    await cg.register_component(var, config)
    parent = await cg.get_variable(config[CONF_SCS_SERVO_ID])
    cg.add(var.set_parent(parent))
