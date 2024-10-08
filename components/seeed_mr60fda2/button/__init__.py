import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_UPDATE,
    ENTITY_CATEGORY_NONE,
    DEVICE_CLASS_RESTART,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from .. import CONF_MR60FDA2_ID, MR60FDA2Component, mr60fda2_ns

GetRadarParametersButton = mr60fda2_ns.class_("GetRadarParametersButton", button.Button)
ResetRadarButton = mr60fda2_ns.class_("ResetRadarButton", button.Button)

CONF_GET_RADAR_PARAMETERS = "get_radar_parameters"
CONF_RESET_RADAR = "reset_radar"

CONFIG_SCHEMA = {
    cv.GenerateID(CONF_MR60FDA2_ID): cv.use_id(MR60FDA2Component),
    cv.Optional(CONF_GET_RADAR_PARAMETERS): button.button_schema(
        GetRadarParametersButton,
        device_class=DEVICE_CLASS_UPDATE,
        entity_category=ENTITY_CATEGORY_NONE,
    ),
    cv.Optional(CONF_RESET_RADAR): button.button_schema(
        ResetRadarButton,
        device_class=DEVICE_CLASS_RESTART,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
}


async def to_code(config):
    mr60fda2_component = await cg.get_variable(config[CONF_MR60FDA2_ID])
    if get_radar_parameters_config := config.get(CONF_GET_RADAR_PARAMETERS):
        b = await button.new_button(get_radar_parameters_config)
        await cg.register_parented(b, config[CONF_MR60FDA2_ID])
        cg.add(mr60fda2_component.set_get_radar_parameters_button(b))
    if reset_radar_config := config.get(CONF_RESET_RADAR):
        b = await button.new_button(reset_radar_config)
        await cg.register_parented(b, config[CONF_MR60FDA2_ID])
        cg.add(mr60fda2_component.set_reset_radar_button(b))
