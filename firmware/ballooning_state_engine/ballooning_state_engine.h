struct sat_message {
  uint16_t unit_id; // (0xFFFF = unset)
  uint8_t state;    // 0=prelaunch, 3=flight, etc
  uint8_t second;   // 0-59
  uint8_t minute;   // 0-59
  uint8_t hour;     // 0-23
  uint8_t day;      // 1-31
  uint8_t month;    // 1-12
  uint8_t year;     //(2000 plus 0-255)
  int32_t latitude;   // N, millionths of degrees
  int32_t longitude;  // E, millionths of degrees
  int16_t altitude;    // up to 32767 meters, 107,503 ft
  int16_t course;   // 100ths of a degree
  int16_t speed;    // 100ths of knot
  uint32_t pressure;   // Pa 0-172369
  int16_t temperature;      // in tenths of deg C.  0-100 -> 0-1000
  int16_t humidity; // in tenths of percent 0-100% -> 0-1000
  uint16_t batt_voltage;  // 0-1023; 1023 = 10V.  3:1 voltage divider on input voltage
};

struct eeprom_config {
  uint16_t unit_id;
  int16_t letdown_delay; // (seconds;  positive number: letdown X seconds after launch detect; negative letdown, letdown X seconds after power on)
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
};
