"""Generate the small browser dialect from the firmware's pymavlink installation."""
import json
from pathlib import Path
from pymavlink.dialects.v20 import ardupilotmega as mav

names = ['HEARTBEAT', 'SET_MODE', 'PARAM_SET', 'PARAM_VALUE', 'ATTITUDE',
         'GLOBAL_POSITION_INT', 'SERVO_OUTPUT_RAW', 'RC_CHANNELS_OVERRIDE',
         'COMMAND_LONG', 'COMMAND_INT', 'COMMAND_ACK', 'NAV_CONTROLLER_OUTPUT', 'MISSION_COUNT', 'MISSION_REQUEST_INT',
         'MISSION_REQUEST', 'MISSION_ITEM_INT', 'MISSION_ITEM', 'MISSION_ACK',
         'MISSION_CURRENT', 'MISSION_ITEM_REACHED', 'MISSION_SET_CURRENT',
         'VFR_HUD', 'STATUSTEXT', 'SIM_STATE', 'HOME_POSITION', 'EKF_STATUS_REPORT', 'NAMED_VALUE_FLOAT',
         'DISTANCE_SENSOR']
definitions = {}
for cls in mav.mavlink_map.values():
    if cls.msgname not in names:
        continue
    offset = 0
    fields = []
    sizes = {'float':4, 'double':8, 'uint64_t':8, 'int64_t':8, 'uint32_t':4,
             'int32_t':4, 'uint16_t':2, 'int16_t':2, 'uint8_t':1, 'int8_t':1, 'char':1}
    for i, name in enumerate(cls.ordered_fieldnames):
        kind = cls.fieldtypes[cls.fieldnames.index(name)]
        count = cls.array_lengths[i] or 1
        fields.append([name, kind, offset, count])
        offset += sizes[kind] * count
    definitions[cls.msgname] = {'id':cls.id,'crc':cls.crc_extra,'length':offset,'fields':fields}
Path(__file__).resolve().parents[2].joinpath('generated/mavlink/dialect.json').write_text(json.dumps(definitions, indent=2)+'\n')
