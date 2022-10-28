# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import struct
import serial
from time import sleep
import sys


def print_hi(name):
    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.

def build_config_struct():

    unit_id = 1235
    letdown_delay = -120  # positive: seconds after launch detect: negative, seconds after power on
    letdown_duration = 5  # seconds
    max_flight_duration = 180  # SECONDS, 0=ignore
    cut_pressure = 0  # Pascals, 0=ignore
    cut_duration = 5000  # milliseconds
    rise_rate_threshold = 85  # Pa/sec * uncalibrated factor: NWC elevator is 100
    update_interval_satellite = 120  # SECONDS, 0 = no update
    max_distance = 0  # meters, 0=ignore
    min_latitude = 0  # millionths of degree, ie 35.123456 = 35123456, 0=ignore
    max_latitude = 0
    min_longitude = 0
    max_longitude = 0 # 0x7FFFFFFE

    config = struct.pack('< HhHHHHHHIiiii',
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

    if (len(sys.argv)<2):
        port = "COM20"
    else:
        port = sys.argv[1]

    print("Hex string for Rockblock: " + config_bytes.hex())

    try:
        serialPort = serial.Serial(port=port, baudrate=19200,
                               bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
    except Exception as e:
        print(e)
        print("Check serial port in device manager!")
        quit(1)

    while serialPort.in_waiting:
        print(serialPort.read(serialPort.in_waiting)) # purge buffer
        sleep(0.100)

    print("Connect cable and turn on device now")

    #wait for first character
    while (serialPort.in_waiting<=0):
        None

    sleep(2.0)
    # purge all arriving characters
    while (serialPort.in_waiting):
        #print(serialPort.read(serialPort.in_waiting))
        print(serialPort.readline().decode('UTF-8').rstrip())
        #sleep(0.100)

    try:
        for i in range(1,10):
            print("sending " + str(config_bytes))
            serialPort.write(config_bytes)
            for j in range(1,10):
                foo = serialPort.readline()
                if (foo):
                    print(foo.decode('UTF-8').rstrip())
                    if (foo==b'OK\r\n'):
                        raise StopIteration
                    if (foo==b'ERR\r\n'):
                        sleep(0.1)
                        break
    except StopIteration:
        serialPort.write(b'END')
        print("Programming success!")

    for i in range(1,10):
        foo=serialPort.readline() # (serialPort.in_waiting)
        if(foo):
            print(foo.decode('UTF-8').rstrip())
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