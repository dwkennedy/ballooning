# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import struct
import serial
from time import sleep


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.

def build_config_struct():

    unit_id = 0x0101
    letdown_delay = 30000
    letdown_duration = 30000
    max_flight_duration = 0
    cut_pressure = 0
    cut_duration = 30000
    rise_rate_threshold = 0xFF
    update_interval_satellite = 60
    max_distance = 0
    min_latitude = 0
    max_latitude = 0
    min_longitude = 0
    max_longitude = 0x7FFFFFFF

    config = struct.pack('< HHHHHHHHIiiii',
                         unit_id,
                         letdown_delay,
                         letdown_duration,
                         max_flight_duration,
                         cut_pressure,
                         cut_duration,
                         rise_rate_threshold,
                         update_interval_satellite,
                         max_distance,
                         min_latitude,
                         max_latitude,
                         min_longitude,
                         max_longitude)
    return config

# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    config_bytes = b'PRG' + build_config_struct()

    try:
        serialPort = serial.Serial(port="COM20", baudrate=19200,
                               bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
    except Exception as e:
        print(e)
        quit(1)

    while serialPort.in_waiting:
        print(serialPort.read(serialPort.in_waiting)) # purge buffer
        sleep(0.100)

    print("Connect cable and turn on device now")

    #wait for first character
    while (serialPort.in_waiting<=0):
        None
    # purge all arriving characters
    while (serialPort.in_waiting):
        #print(serialPort.read(serialPort.in_waiting))
        print(serialPort.readline())
        sleep(0.100)

    sleep(0.100)

    for i in range(1,2):
        print("sending " + str(config_bytes))
        serialPort.write(config_bytes)
        for i in range(1, 20):
            foo = serialPort.readline()
            if (foo):
                print(foo)
                if (foo==b'OK\r\n'):
                    serialPort.write(b'END')
                if (foo==b'ERR\r\n'):
                    print("retry " + str(config_bytes))
                    serialPort.write(config_bytes)
                sleep(0.100)
            else:
                sleep(0.5)

    for i in range(1,10):
        foo=serialPort.readline() # (serialPort.in_waiting)
        if(foo):
            print(foo)
            sleep(0.100)
        else:
            sleep(1)

    #print("->" + str(config_bytes))
    #serialPort.write(config_bytes)
    #for i in range(1,6):
    #    print(serialPort.readline())
    #    sleep(0.100)


    #serialPort.write(b'END')
    #for i in range(1,6):
    #    print(serialPort.readline())
    #    sleep(0.100)


# See PyCharm help at https://www.jetbrains.com/help/pycharm/
"""
struct eeprom_config {
  uint16_t unit_id;
  uint16_t letdown_delay; // (milliseconds after launch detect)
  uint16_t letdown_duration; // (milliseconds, 0=disable)
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
};   'HHHHHHHHHIiiii'
"""