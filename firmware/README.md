note: I monkeyed with the Adafruit MPRLS driver to make a function to return integer pressure in uint32_t (Pascals).  range is around 87,000 to 108,500 Pa at sea level so this fits in a 32 bit integer

I modified the oversampling rate in the sparkfun MLR3115A2 driver; the sampling rate is ~700 ms at 128 samples, 6ms at 1 sample, 32x is about 200 ms.  I'm shooting for a 330ms sample period so I went with 32

The application is in ballooning_state_engine, the other files are test code only

compile notes:
1.8.x version of Arduino IDE (2.x should work but I haven't tested it well)
Add adafruit MPL3115A2 library (and extra required Adafruit base library)
Add library CRC16 by Rob Tillart (version 0.3.3)
Add library TinyGPSplus
Add board definition URL http://www.bamfordresearch.com/files/package_jb23_insystemmcu_index.json
install Zadig using these instructions https://forum.arduino.cc/t/avrisp-mkii-driver/632191/3
   select libusbK for avrisp mkii, restart arduino IDE
Select ATMEGA328P as board type
select AVRISP mkii as programmer; serial port shouldn't matter
