# 50524700001E0005003C000000B80B550078000000000000000000000000000000000000000000

# This program decodes a configuration

# usage: checkcfg.py  [hex config string]

import struct
import serial
from time import sleep
import sys
import re


def dump_config(cfg):
    print("unit_id: %d" % cfg[0])

    if (cfg[1]<0):
        print("letdown_delay: %d seconds after power on" % abs(cfg[1]))
    else:
        print("letdown_delay: %d seconds after launch detect" % cfg[1])

    print("letdown_duration: %d seconds" % cfg[2])

    if (cfg[3]):
        print("max_flight_duration: %d seconds" % cfg[3])
    else:
        print("max_flight_duration: no limit")

    if (cfg[4]==0):
        print("cut_pressure: no limit")
    else:
        print("cut_pressure: %d Pa" % cfg[4])

    print("cut_duration: %d milliseconds" % cfg[5])

    print("rise_rate_threshold: %d" % cfg[6])

    if (cfg[7]>0):
        print("update_interval_satellite: %d" % cfg[7])
    else:
        print("update_interval_satellite: NO UPDATES")

    if (cfg[8]!=0):
        print("max_distance: %d meters" % cfg[8])
    else:
        print("max_distance: ignore")

    if (cfg[9]!=0):
        print("min_latitude: %f degrees N" % (cfg[9]/1000000))
    else:
        print("min_latitude: ignore")

    if (cfg[10]!=0):
        print("max latitude: %f degrees N" % (cfg[10]/1000000))
    else:
        print("max latitude: ignore")

    if (cfg[11]!=0):
        print("min longitude: %f degrees W" % (cfg[11]/1000000))
    else:
        print("min longitude: ignore")

    if (cfg[12]!=0):
        print("max longitude: %f degrees W" % (cfg[12]/1000000))
    else:
        print("max longitude: ignore")

def unpack_config_struct(buffer):
    config = struct.unpack('< HhHHHHHHIiiii', buffer)
    return config


# Press the green button in the gutter to run the script.
if __name__ == '__main__':

    cfgRegex = re.compile(b'\*\*\* EEPROM ([0-9A-Fa-f]{72})')  # regex to find configuration from BAD output

    print("")
    if (len(sys.argv)>1):
        configuration = sys.argv[1]
    else:
        configuration = "00001E0005003C000000B80B550078000000000000000000000000000000000000000000" # 505247
        configuration = "0000E2FF020000000000B80B55002C010000000080CC06020051250280A328FA002847FA"

    dump = bytes.fromhex(configuration)  # convert the ascii hex to bytes
    #print("bytes: " + str(dump))
    if(dump):
        cfg = unpack_config_struct(dump)   # unpack the bytes of the struct
        print("Configuration as read")
        print("---------------------")
        dump_config(cfg)  # dump the configuration in readable format


# See PyCharm help at https://www.jetbrains.com/help/pycharm/
"""
struct eeprom_config {
  uint16_t unit_id;
  int16_t letdown_delay; // (seconds; positive: letdown X seconds after launch detect; negative: letdown, letdown X seconds after power on)
  uint16_t letdown_duration; // (seconds, 0=disable)
  uint16_t max_flight_duration; // seconds (0=ignore)
  uint16_t cut_pressure; //  Pa.  (0=ignore)
  uint16_t cut_duration; // (milliseconds)
  uint16_t rise_rate_threshold; //  (digital units, see test data)
  uint16_t update_interval_satellite; // (seconds, 0=no update)
  uint32_t max_distance;  // (meters, 0=disable)
  int32_t min_latitude;  // (millionths of degrees)
  int32_t max_latitude;  // (millionths of degrees)
  int32_t min_longitude; // (millionths of degrees)
  int32_t max_longitude; // (millionths of degrees)
};   'HhHHHHHHHIiiii'
"""