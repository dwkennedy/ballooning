This is a python program to send programming sequences over a serial port to the device
requirement: pip install pyserial

Use a usb to 3.3V TTL FTDI serial cable like https://www.sparkfun.com/products/9717

For example, if the FTDI serial device is connected to COM20:

$ python configure.py COM20

Hex string for Rockblock: 5052470101e2ff05003c00000088135500780000000000000000000000000000000000feffff7f
Connect cable and turn on device now

.
.
.


You should see an "OK" response after the CFG string is sent.  The hex string is used to send a configuration
to a device over the satellite connection.  This could be useful to change the frequency of transmissions (or
shut them off entirely) after the device has landed to save message charges.  Also, cut down parameters can
be changed in flight.
