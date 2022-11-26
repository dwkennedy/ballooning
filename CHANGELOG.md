11/25/2022

* replace redundant unit_id with number of satellites used in fix and hdop

* add CRC to configuration and beacon packets in device firmware

* update server software to examine and verify CRC from received RockBLOCK messages

* add check for motor stalling during letdown sequence, turns off motor when battery voltage hits critical level to prevent boot loops

* tested motor stalling, motor stall won't reset board at room temp (and weak batteries).  needs more testing at very cold battery temperatures

* added SOS led flashing if bad condition detected at boot (plug in serial cable for more info)
    1.  CRC error in EEPROM configuration (should never happen)
    2.  low battery (replace with fresh batteries, Energizer, Duracell, or Panasonic only)
    3.  cutter overcurrent or open circuit  (check cutter winding and connections)
    4.  motor overcurrent or open circuit  (motor jammed, wire disconnected or broken)
    5.  pressure sensor error (check SMD soldering, or reconfigure for timed let-down initiation)

* updated configure.py for new firmware with CRC, checkcfg.py no longer needed (there is a flag to enable/disable programming in configure.py.  configure.py always reads device configuration and attempts to upload to database)
