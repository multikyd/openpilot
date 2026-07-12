#!/usr/bin/env python3
from collections import namedtuple


def generate():
  parts = []
  parts.append("""
VERSION ""


NS_ :
    NS_DESC_
    CM_
    BA_DEF_
    BA_
    VAL_
    CAT_DEF_
    CAT_
    FILTER
    BA_DEF_DEF_
    EV_DATA_
    ENVVAR_DATA_
    SGTYPE_
    SGTYPE_VAL_
    BA_DEF_SGTYPE_
    BA_SGTYPE_
    SIG_TYPE_REF_
    SIG_GROUP_
    SIG_VALTYPE_
    SIGTYPE_VALTYPE_
    BO_TX_BU_
    BA_DEF_REL_
    BA_REL_
    BA_DEF_DEF_REL_
    BU_SG_REL_
    BU_EV_REL_
    BU_BO_REL_
    SG_MUL_VAL_

BS_:

BU_: XXX RADAR
""")

  for a in range(0x235, 0x249):
    parts.append(f"""
BO_ {a} CORNER_RADAR_235_OBJECTS_{a:x}: 32 RADAR
 SG_ OBJ_QUAL_LEVEL : 24|7@1+ (1,0) [0|100] "%" XXX
 SG_ OBJ_REL_POS_X : 64|13@1+ (0.05,0) [0|409.55] "m" XXX
 SG_ OBJ_REL_POS_Y : 78|12@1+ (0.05,-102.4) [-102.4|102.35] "m" XXX
 SG_ OBJ_REL_VEL_X : 91|12@1+ (0.05,-100) [-100|104.75] "m/s" XXX
 SG_ OBJ_REL_VEL_Y : 104|10@1+ (0.05,-25) [-25|26.15] "m/s" XXX
 SG_ OBJ_REL_ACCEL_X : 115|9@1- (0.05,0) [-12.8|12.75] "m/s^2" XXX
""")

  return {"hyundai_canfd_corner_radar_235.dbc": "".join(parts)}
