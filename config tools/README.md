This is a python program to send programming sequences over a serial port to the device
requirement: pip install pyserial

Use a usb to 3.3V TTL FTDI serial cable like https://www.sparkfun.com/products/9717

For example, if the FTDI serial device is connected to COM20:

```text
$ python configure.py COM20

C:\Users\micro\anaconda3\python.exe "C:/Users/micro/Documents/ballooning/config tools/configure.py" 
unit_id: 1237
letdown_delay: 60 seconds after power on
letdown_duration: 1 seconds
max_flight_duration: 120 seconds
cut_pressure: no limit
cut_duration: 3000 milliseconds
rise_rate_threshold: 85
update_interval_satellite: 120
max_distance: ignore
min_latitude: ignore
max latitude: ignore
min longitude: ignore
max longitude: ignore

Hex string for RockBLOCK: d504c4ff010078000000b80b550078000000000000000000000000000000000000000000
```
The "Hex string for RockBlock" is the hex message used
to send a configuration to a device over the satellite connection.  This could be useful to change the frequency of transmissions (or
shut them off entirely) after the device has landed to save message charges.  Or, the cut down parameters could
be changed in flight depending on wind conditions aloft.

Continuing with the program output:
```text
Connect cable and turn on device now
tx: b'PRG\xd5\x04\xc4\xff\x01\x00x\x00\x00\x00\xb8\x0bU\x00x\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'

*** Start setup()
*** MOTOR ON
*** MOTOR OFF
*** CUTTER ON
*** CUTTER OFF
tx: b'PRG\xd5\x04\xc4\xff\x01\x00x\x00\x00\x00\xb8\x0bU\x00x\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
*** EEPROM 00001E0005003C000000B80B550078000000000000000000000000000000000000000000
*** Wait for serial cmd
ERR
tx: b'PRG\xd5\x04\xc4\xff\x01\x00x\x00\x00\x00\xb8\x0bU\x00x\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
*** PRG D504C4FF010078000000B80B550078000000000000000000000000000000000000000000
OK
Programming success!
END

*** CFG D504C4FF010078000000B80B550078000000000000000000000000000000000000000000

Configuration as read
---------------------
unit_id: 1237
letdown_delay: 60 seconds after power on
letdown_duration: 1 seconds
max_flight_duration: 120 seconds
cut_pressure: no limit
cut_duration: 3000 milliseconds
rise_rate_threshold: 85
update_interval_satellite: 120
max_distance: ignore
min_latitude: ignore
max latitude: ignore
min longitude: ignore
max longitude: ignore

```
In the example above, the desired configuration to be sent to the device is printed, followed by a trace of the programming dialog.
You should see an "OK" response after the CFG string is sent.  

After the device is programmed it reads the EEPROM and returns the device configuration (*** CFG XXXXX...)  The configuration string is then
decoded and displayed so you can verify that the configuration has been sucessfully updated.  I suggest printing the configuration and keeping
a copy with the device so you know how it's programmed without having to hook it up and/or reprogram it.

