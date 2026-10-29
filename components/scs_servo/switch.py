import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch

from . import SCSServo, scs_servo_ns

CONF_SCS_SERVO_ID = "scs_servo_id"

SCSTorqueSwitch = scs_servo_ns.class_("SCSTorqueSwitch", switch.Switch, cg.Component)

CONFIG_SCHEMA = (
    switch.switch_schema(SCSTorqueSwitch, icon="mdi:engine", entity_category="config")
    .extend({cv.GenerateID(CONF_SCS_SERVO_ID): cv.use_id(SCSServo)})
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)
    parent = await cg.get_variable(config[CONF_SCS_SERVO_ID])
    cg.add(var.set_parent(parent))
