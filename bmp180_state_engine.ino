/*
  detect balloon launch by thresholding rise rate
  Doug Kennedy
  for BMP180 or similar sensor using adafruit BMP805 library
  Output 13 (LED)
  ON - rise detected, positive slope exceeds threshold
  FLASH - sink detected, negative slope exceeds threshold
  OFF - between rise/sink thresholds
*/

/*
   rise_rate = sum (filter[i]*sample[(i+n)%N)  
     filter 16 bit (+/- 32768) 16 bits  +/- 64 so 7 bits
     sample 32 bit (0-120000)  18 bits
     N 33                       5 bits
*/

//  https://github.com/SlashDevin/NeoSWSerial
//  https://github.com/naszly/Arduino-StaticSerialCommands
//  https://github.com/stevemarple/MicroNMEA
//  http://www.bamfordresearch.com/files/package_jb23_insystemmcu_index.json

// if using BMP180 pressure sensor
//#define BMP180

// if using MPR pressure sensor
#define MPR

// if using FAKE pressure for testing
//#define FAKE_PRESSURE

#ifdef BMP180
#include <Adafruit_BMP085.h>
// Connect VCC of the BMP180 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
#endif

#ifdef MPR
#include <Wire.h>
#include <SparkFun_MicroPressure.h>
#endif 

#include <StaticSerialCommands.h>
// #include <SoftwareSerial.h>
// #define _SS_MAX_RX_BUFF 128 // RX buffer size
#include <NeoSWSerial.h>
#include <MicroNMEA.h>
#include <EEPROM.h>
#include <math.h>

// debug state engine
#define DEBUG (1)

// debug sensor and filter
#define DEBUG_SENSOR (0)

// debug plot the rise_rate
#define DEBUG_PLOT (0)

// print out average loop time in ms, current loop time
#define DEBUG_SAMPLE_INTERVAL (0)

// uncomment to calculate filter coefficents and dump to serial
//#define DEBUG_FILTER (1)

// Motor port
#define MOTOR (2)

// hot-wire cutter port
#define CUTTER (3)

// GPS LED port
#define LED_GPS (4)

// length of filter (N must be odd)
#define N (33)

// make this interval between samples; filter total time is N*T
#define T (333)

// filter delay in milliseconds
#define FILTER_DELAY (N*T)

// sink rate to trigger event - not presently used; could be used to detect balloon pop.  less than -100 could be freefall
#define SINK_RATE_THRESHOLD (-100)

// parameters to adjust for flight
// address/serial number of unit
static unsigned int unit_id;

// how many seconds before activation of the let down motor
static unsigned int letdown_delay;

// how long to run the motor (seconds)
static unsigned int letdown_duration;

// when to terminate the flight (seconds)
static unsigned int max_flight_duration;

// pressure at which to cut, mbar
static unsigned int cut_pressure;

// how long to cut (seconds)
static unsigned int cut_duration;

// rise rate to trigger event. should be higher than noise but less than balloon rise rate. depends on sample rate and filter
// elevator in NWC peaks out around 60.  noise level is +/- 10
static unsigned int rise_rate_threshold;

// how often to send GPS packet
static unsigned int update_interval;

// max distance balloon can travel before cut (over surface of earth, in meters)
static unsigned long max_distance;

// geofencing min/max lat/long
static long min_latitude;
static long max_latitude;
static long min_longitude;
static long max_longitude;

NeoSWSerial gpsSerial (10, 9);
// SoftwareSerial gpsSerial (10, 9);

#ifdef BMP180
Adafruit_BMP085 bmp;
#endif

#ifdef MPR
/*
 * Initialize Constructor
 * Optional parameters:
 *  - EOC_PIN: End Of Conversion (defualt: -1)
 *  - RST_PIN: Reset (defualt: -1)
 *  - MIN_PSI: Minimum Pressure (default: 0 PSI)
 *  - MAX_PSI: Maximum Pressure (default: 25 PSI)
 */
//SparkFun_MicroPressure mpr(EOC_PIN, RST_PIN, MIN_PSI, MAX_PSI);
SparkFun_MicroPressure mpr; // Use default values with reset and EOC pins unused
#endif

char nmeaBuffer[85];  // a buffer big enough to hold largest expected NMEA sentence
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

static long launch_lat;
static long launch_lon;
static long launch_alt;

// vector of filter coefficients
#ifdef DEBUG_FILTER
int filter[N];
#else
const int filter[N] = {53,50,46,43,40,36,33,30,26,23,20,16,13,10,6,3,
                       0,-3,-6,-10,-13,-16,-20,-23,-26,-30,-33,-36,-40,-43,-46,-50,-53};
#endif

// circular buffer of N samples
long sample[N];
long base_pressure;

// index of circular buffer, 0 to N-1
int n = 0;

// state engine labels
enum state
{
  PRELAUNCH,
  LETDOWN_INIT,
  LETDOWN_ACTIVE,
  FLIGHT,
  CUT_INIT,
  CUT_ACTIVE,
  POST_FLIGHT
};
typedef enum state state; /* also a typedef of same identifier */

state active_state = PRELAUNCH;  // initial state

int LED_period = 2000;  // interval to blink LED
int LED_duration = 50;  // duration of LED blink
const int GPS_LED_period = 500;  // interval to blink GPS locked LED
const int GPS_LED_duration = 250;  // duration to blink GPS locked LED

unsigned long launch_time = 0;  // so we can time letdown and flight time

void cmd_help(SerialCommands& sender, Args& args);
void cmd_ping(SerialCommands& sender, Args& args);
void cmd_show(SerialCommands& sender, Args& args);
void cmd_cut(SerialCommands& sender, Args& args);
void cmd_letdown(SerialCommands& sender, Args& args);
void cmd_update(SerialCommands& sender, Args& args);
void cmd_set(SerialCommands& sender, Args& args);
void cmd_set_unit_id(SerialCommands& sender, Args& args);
void cmd_set_letdown_delay(SerialCommands& sender, Args& args);
void cmd_set_letdown_duration(SerialCommands& sender, Args& args);
void cmd_set_max_flight_duration(SerialCommands& sender, Args& args);
void cmd_set_cut_pressure(SerialCommands& sender, Args& args);
void cmd_set_cut_duration(SerialCommands& sender, Args& args);
void cmd_set_rise_rate_threshold(SerialCommands& sender, Args& args);
void cmd_set_update_interval(SerialCommands& sender, Args& args);
void cmd_set_max_distance(SerialCommands& sender, Args& args);
void cmd_set_min_latitude(SerialCommands& sender, Args& args);
void cmd_set_max_latitude(SerialCommands& sender, Args& args);
void cmd_set_min_longitude(SerialCommands& sender, Args& args);
void cmd_set_max_longitude(SerialCommands& sender, Args& args);

Command subCommands [] {
  COMMAND(cmd_set_unit_id, "unit_id", ArgType::Int, nullptr, "unique address"),
  COMMAND(cmd_set_letdown_delay, "letdown_delay", ArgType::Int, nullptr, "activation after launch, secs"),
  COMMAND(cmd_set_letdown_duration, "letdown_duration", ArgType::Int, nullptr, "time to lower, secs"),
  COMMAND(cmd_set_max_flight_duration, "max_flight_duration", ArgType::Int, nullptr, "elapsed time to terminate flight, secs"),
  COMMAND(cmd_set_cut_pressure, "cut_pressure", ArgType::Int, nullptr, "pressure to terminate flight, mbar"),
  COMMAND(cmd_set_cut_duration, "cut_duration", ArgType::Int, nullptr, "how long to activate cutter, secs"),
  COMMAND(cmd_set_rise_rate_threshold, "rise_rate_threshold", ArgType::Int, nullptr, "let-down trigger sensitivity (~500-1000)"),
  COMMAND(cmd_set_update_interval, "update_interval", ArgType::Int, nullptr, "how often to send beacon, secs"),
  COMMAND(cmd_set_max_distance, "max_distance", ArgType::Int, nullptr, "distance from launch to terminate flight, meters"),
  COMMAND(cmd_set_min_latitude, "min_latitude", ArgType::Int, nullptr, "minimum latitude geofence, degrees X 10^6 N"),
  COMMAND(cmd_set_max_latitude, "max_latitude", ArgType::Int, nullptr, "maximum latitude geofence, degrees X 10^6 N"),
  COMMAND(cmd_set_min_longitude, "min_longitude", ArgType::Int, nullptr, "minimum longitude geofence, degrees X 10^6 E"),
  COMMAND(cmd_set_max_longitude, "max_longitude", ArgType::Int, nullptr, "maximum longitude geofence, degrees X 10^6 E"),
};

Command commands[] {
  COMMAND(cmd_help, "help", nullptr, "list cmds"),
  COMMAND(cmd_ping, "ping", ArgType::String, nullptr, "link test"),
  COMMAND(cmd_show, "show", nullptr, "show params"),
  COMMAND(cmd_set, "set", subCommands, "set params"),
  COMMAND(cmd_letdown, "letdown", ArgType::Int, nullptr, "actuate letdown <int> seconds"),
  COMMAND(cmd_cut, "cut", ArgType::Int, nullptr, "actuate cutter unit_id <int>"),
  COMMAND(cmd_update, "update", ArgType::Int, nullptr, "temp change update rate"),
};

void cmd_help(SerialCommands& sender, Args& args) {
  sender.listCommands();
}

void cmd_update(SerialCommands& sender, Args& args) {
  update_interval = args[0].getInt();
}

void cmd_ping(SerialCommands& sender, Args& args) {
  sender.getSerial().print(F("pong "));
  sender.getSerial().println(args[0].getString());
}

void cmd_cut(SerialCommands& sender, Args& args) {
  if (args[0].getInt() == unit_id) {
    sender.getSerial().println(F("cutter activated"));
    active_state = CUT_INIT;
  } else {
    sender.getSerial().println(F("unit_id mismatch"));
  }
}

// this command is only for ground testing, as puts you in flight mode
// could be used if the pressure sensor goes nuts and the letdown doesn't trip
void cmd_letdown(SerialCommands& sender, Args& args) {
  sender.getSerial().println(F("letdown activating in 3 secs"));
  active_state = LETDOWN_INIT;
  launch_time = millis();
  if (nmea.isValid()) {
    launch_lat = nmea.getLatitude();  // millionths of degrees
    launch_lon = nmea.getLongitude(); // millionths of degrees
    nmea.getAltitude(launch_alt);  // altitude MSL
  }
  letdown_duration = args[0].getInt();
  letdown_delay = 3;
  LED_period = 500;  // long slow blink
  LED_duration = 450;
  digitalWrite(CUTTER, LOW);  // turn off cutter just in case
}

void cmd_show(SerialCommands& sender, Args& args) {
  sender.getSerial().print(F("unit_id: "));
  sender.getSerial().println(unit_id);
  sender.getSerial().print(F("letdown_delay: "));
  sender.getSerial().println(letdown_delay);
  sender.getSerial().print(F("letdown_duration: "));
  sender.getSerial().println(letdown_duration);
  sender.getSerial().print(F("max_flight_duration: "));
  sender.getSerial().println(max_flight_duration);
  sender.getSerial().print(F("cut_pressure: "));
  sender.getSerial().println(cut_pressure);
  sender.getSerial().print(F("cut_duration: "));
  sender.getSerial().println(cut_duration);
  sender.getSerial().print(F("rise_rate_threshold: "));
  sender.getSerial().println(rise_rate_threshold);
  sender.getSerial().print(F("update_interval: "));
  sender.getSerial().println(update_interval);
  sender.getSerial().print(F("max_distance: "));
  sender.getSerial().println(max_distance);
  sender.getSerial().print(F("min_latitude: "));
  sender.getSerial().println(min_latitude);
  sender.getSerial().print(F("max_latitude: "));
  sender.getSerial().println(max_latitude);
  sender.getSerial().print(F("min_longitude: "));
  sender.getSerial().println(min_longitude);
  sender.getSerial().print(F("max_longitude: "));
  sender.getSerial().println(max_longitude);
  sender.getSerial().print(F("active_state: "));
  sender.getSerial().println(active_state);

}

void cmd_set(SerialCommands& sender, Args& args) {
  sender.listAllCommands(subCommands, sizeof(subCommands) / sizeof(Command));
}

void cmd_set_unit_id(SerialCommands& sender, Args& args) {
  unsigned int number = args[0].getInt();
  EEPROM.put(0 * sizeof(unsigned int), (unsigned int)number); // save serial number
  unit_id = number;  // set serial in ram as well
  sender.getSerial().print(F("unit_id="));
  sender.getSerial().println(number);
}

void cmd_set_letdown_delay(SerialCommands& sender, Args& args) {
  unsigned int number = args[0].getInt();
  EEPROM.put(1 * sizeof(unsigned int), (unsigned int)number); // save number
  letdown_delay = number;  // set in ram as well
  sender.getSerial().print(F("letdown_delay="));
  sender.getSerial().println(number);
}

void cmd_set_letdown_duration(SerialCommands& sender, Args& args) {
  auto number = args[0].getInt();
  EEPROM.put(2 * sizeof(unsigned int), (unsigned int)number); // save number
  letdown_duration = number;  // set in ram as well
  sender.getSerial().print(F("letdown_duration="));
  sender.getSerial().println(number);
}

void cmd_set_max_flight_duration(SerialCommands& sender, Args& args) {
  auto number = args[0].getInt();
  EEPROM.put(3 * sizeof(unsigned int), (unsigned int)number); // save number
  max_flight_duration = number;  // set in ram as well
  sender.getSerial().print(F("max_flight_duration="));
  sender.getSerial().println(number);
}

void cmd_set_cut_pressure(SerialCommands& sender, Args& args) {
  auto number = args[0].getInt();
  EEPROM.put(4 * sizeof(unsigned int), (unsigned int)number); // save number
  cut_pressure = number;  // set in ram as well
  sender.getSerial().print(F("cut_pressure="));
  sender.getSerial().println(number);
}

void cmd_set_cut_duration(SerialCommands& sender, Args& args) {
  auto number = args[0].getInt();
  EEPROM.put(5 * sizeof(unsigned int), (unsigned int)number); // save number
  cut_duration = number;  // set in ram as well
  sender.getSerial().print(F("cut_duration="));
  sender.getSerial().println(number);
}

void cmd_set_rise_rate_threshold(SerialCommands& sender, Args& args) {
  auto number = args[0].getInt();
  EEPROM.put(6 * sizeof(unsigned int), (unsigned int)number); // save number
  rise_rate_threshold = number;  // set in ram as well
  sender.getSerial().print(F("rise_rate_threshold="));
  sender.getSerial().println(number);
}

void cmd_set_update_interval(SerialCommands& sender, Args& args) {
  auto number = args[0].getInt();
  EEPROM.put(7 * sizeof(unsigned int), (unsigned int)number); // save number
  update_interval = number;  // set in ram as well
  sender.getSerial().print(F("update_interval="));
  sender.getSerial().println(number);
}

void cmd_set_max_distance(SerialCommands& sender, Args& args) {
  unsigned long number = args[0].getInt();
  EEPROM.put(8 * sizeof(unsigned int) + 0 * sizeof(long), (unsigned long)number); // save unsigned long distance
  max_distance = number;  // set in ram as well
  sender.getSerial().print(F("max_distance="));
  sender.getSerial().println(number);
}

void cmd_set_min_latitude(SerialCommands& sender, Args& args) {
  long number = args[0].getInt();
  EEPROM.put(8 * sizeof(unsigned int) + 1 * sizeof(long), (long)number); // save long min_latitude
  min_latitude = number;  // set in ram as well
  sender.getSerial().print(F("min_latitude="));
  sender.getSerial().println(number);
}

void cmd_set_max_latitude(SerialCommands& sender, Args& args) {
  long number = args[0].getInt();
  EEPROM.put(8 * sizeof(unsigned int) + 2 * sizeof(long), (long)number); // save long max_latitude
  max_latitude = number;  // set in ram as well
  sender.getSerial().print(F("max_latitude="));
  sender.getSerial().println(number);
}


void cmd_set_min_longitude(SerialCommands& sender, Args& args) {
  long number = args[0].getInt();
  EEPROM.put(8 * sizeof(unsigned int) + 3 * sizeof(long), (long)number); // save long min_longitude
  min_longitude = number;  // set in ram as well
  sender.getSerial().print(F("min_longitude="));
  sender.getSerial().println(number);
}

void cmd_set_max_longitude(SerialCommands& sender, Args& args) {
  long number = args[0].getInt();
  EEPROM.put(8 * sizeof(unsigned int) + 4 * sizeof(long), (long)number); // save long max_longitude
  max_longitude = number;  // set in ram as well
  sender.getSerial().print(F("max_longitude="));
  sender.getSerial().println(number);
}

char buffer[80];  // a buffer big enough to hold serial commands or reminder prompts
SerialCommands serialCommands(Serial, commands, sizeof(commands) / sizeof(Command),
                              buffer, sizeof(buffer));

// distance between points on earth in meters
// lat/lon in millionths of degrees (divide by 1,000,000 for degrees)
double haversine(double lat1, double lon1, double lat2, double lon2) {
  const double earth_radius = 6371009;  // earth radius in meters
  //const double pi180 = 3.1415926535897932384626433832795/180.0/1000000.0;
  const double pi180 = 0.000000017453292519943295769236907684886127134428718885417254560971;

  // convert millionths of degrees to radians
  //double dlat = (double)(lat2 - lat1) * pi180;
  double dlon = (double)(lon2 - lon1) * pi180;
  lat1 = lat1 * pi180;
  lat2 = lat2 * pi180;
  double dlat = lat2 - lat1;
  
  double dist = 2 * earth_radius * asin(
                  sqrt( pow(sin( (dlat / 2)), 2)
                        + pow(sin( (dlon / 2)), 2)
                        * cos(lat1) * cos(lat2)));
  
  return (dist); // great circle distance in km
}


void setup() {
  
  // configure control ports and make sure they're off
  pinMode(MOTOR, OUTPUT);  // motor control port
  digitalWrite(MOTOR, LOW);  // motor off
  pinMode(CUTTER, OUTPUT);  // cutter control port
  digitalWrite(CUTTER, LOW);  // cutter off

  // set up GPS LED
  pinMode(LED_GPS, OUTPUT);
  // set up output indicator LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_GPS, HIGH);
  delay(150);
  digitalWrite(LED_BUILTIN, HIGH);  // blip blip on boot
  digitalWrite(LED_GPS, LOW);  // blip blip on boot
  delay(150);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LED_GPS, HIGH);
  delay(150);
  digitalWrite(LED_BUILTIN, HIGH);  // blip #2
  digitalWrite(LED_GPS, LOW);  // blip blip on boot
  delay(150);
  digitalWrite(LED_BUILTIN, LOW);

  // set serial port baud rates
  Serial.begin(9600);  // hardware serial is connected to sat radio and/or programming FTDI cable
  gpsSerial.begin(9600);

  if (DEBUG) {
    Serial.println(F("*** Start setup()"));
  }
  
  // calculate filter coefficients
  //  (could be done statically if N is fixed)
#ifdef DEBUG_FILTER
    Serial.print(F("filter[] = {"));

    for (int i = 0; i < N; i++) {
      //filter[i] = (double)( -(((double)((double)i-(((double)N-1L)/2L))/(double)N)) );
      //filter[i] = 1;
      // filter coeffs with double -- works
      // filter[i] = -(12.0 * (double)i-6.0 * ((double)N-1.0)) / ((double)N * ((double)N*(double)N - 1.0)); // - (12*i - 6*(N-1))   /   N*(N^2-1)
      // filter[i] = 10000*filter[i];  // scale filter coefficients, delta Pressure / sample period
      // try filter coeefs with longs, 1000000 is in there to ensure the coefficients don't get wiped out due to lack of precision
      filter[i] = (-10000.0 * (12.0 * (double)i - 6.0 * ((double)N - 1.0))) / ((double)N * ((double)N * (double)N - 1.0));
      if (i<(N-1)) {
        Serial.print(filter[i]);
        Serial.print(F(","));
      }
    }

    Serial.print(filter[N-1]);
    Serial.println(F("};"));
#endif

  // read parameters from EEPROM
  // parameters to adjust for flight

  // unit address/ serial number
  EEPROM.get(0 * sizeof(unsigned int), unit_id);

  // check for uninitialized EEPROM
  if (unit_id == 65535) { // not initialized, so lets initialize it
    EEPROM.put(1 * sizeof(unsigned int), (unsigned int)30); // letdown_delay
    EEPROM.put(2 * sizeof(unsigned int), (unsigned int)30); // letdown_duration
    EEPROM.put(3 * sizeof(unsigned int), (unsigned int)0); // max_flight_duration
    EEPROM.put(4 * sizeof(unsigned int), (unsigned int)0); // cut_pressure
    EEPROM.put(5 * sizeof(unsigned int), (unsigned int)30); // cut_duration
    EEPROM.put(6 * sizeof(unsigned int), (unsigned int)40); // rise_rate_threshold
    EEPROM.put(7 * sizeof(unsigned int), (unsigned int)60); // update_interval
    EEPROM.put(8 * sizeof(unsigned int) + 0 * sizeof(long), (unsigned long)0); // max_distance
    EEPROM.put(9 * sizeof(unsigned int) + 1 * sizeof(long), (long)0); // min_latitude
    EEPROM.put(10* sizeof(unsigned int) + 2 * sizeof(long), (long)0); // max_latitude
    EEPROM.put(11* sizeof(unsigned int) + 3 * sizeof(long), (long)0); // min_longitude
    EEPROM.put(12* sizeof(unsigned int) + 4 * sizeof(long), (long)0); // max_longitude
    EEPROM.put(0 * sizeof(unsigned int), (unsigned int)0);  // unit_id 0=UNSET
    unit_id = 0;
  }

  // how many seconds before activation of the let down motor
  EEPROM.get(1 * sizeof(unsigned int), letdown_delay);

  // how long to run the motor (seconds)
  EEPROM.get(2 * sizeof(unsigned int), letdown_duration);

  // when to terminate the flight (seconds)
  EEPROM.get(3 * sizeof(unsigned int), max_flight_duration);

  // pressure at which to cut, mbar
  EEPROM.get(4 * sizeof(unsigned int), cut_pressure);

  // how long to cut (seconds)
  EEPROM.get(5 * sizeof(unsigned int), cut_duration);

  // rise rate to trigger event.
  EEPROM.get(6 * sizeof(unsigned int), rise_rate_threshold);

  // update interval
  EEPROM.get(7 * sizeof(unsigned int), update_interval);

  // update geofencing
  EEPROM.get( 8 * sizeof(unsigned int) + 0 * sizeof(long), max_distance);
  EEPROM.get( 9 * sizeof(unsigned int) + 1 * sizeof(long), min_latitude);
  EEPROM.get(10 * sizeof(unsigned int) + 2 * sizeof(long), max_latitude);
  EEPROM.get(11 * sizeof(unsigned int) + 3 * sizeof(long), min_longitude);
  EEPROM.get(12 * sizeof(unsigned int) + 4 * sizeof(long), max_longitude);

#ifdef BMP180
  while (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085/180 sensor, check wiring");
    for (int i = 0; i < 4; i++) {
      digitalWrite(LED_BUILTIN, HIGH);  // blink LED 4 times to indicate sensor problem
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
    }
    delay(400);  // give an extra time between the 4 pulses
  }
#endif

#ifdef MPR
  /* The micropressure sensor uses default settings with the address 0x18 using Wire.

     The mircropressure sensor has a fixed I2C address, if another address is used it
     can be defined here. If you need to use two micropressure sensors, and your
     microcontroller has multiple I2C buses, these parameters can be changed here.

     E.g. mpr.begin(ADDRESS, Wire1)

     Will return true on success or false on failure to communicate. */
  Wire.begin();
  while(!mpr.begin())
  {
    Serial.println("Cannot connect to MicroPressure sensor.");
    for (int i = 0; i < 4; i++) {
      digitalWrite(LED_BUILTIN, HIGH);  // blink LED 4 times to indicate sensor problem
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
    }
    delay(400);  // give an extra time between the 4 pulses
  }
#endif

  if (DEBUG) {
    serialCommands.getSerial().println(F("*** Connected to pressure sensor"));
  }

  if (DEBUG_PLOT) {  // print out a bunch of zeros for the sake of simple arduino serial plotter
    for (int i = 0; i < 500; i++) {
      Serial.println("0 0");
    }
  }

  /** if (0) {  // wierd stuff
    base_pressure = 0;  // calculate base pressure for plotting later, average of several readings
    for (int i=0; i<N*2; i++) {
      // WHY DO I DO THIS WEIRD THING ABOUT SAMPLE[I/2]????? maybe to get more samples for base pressure?
    #ifdef BMP180
      //sample[i/2] = bmp.readPressure()/(double)100.0;  // read pressure in Pa, convert to mbar because we like mbar
      sample[i/2] = (long)bmp.readPressure();  // read pressure in Pa, mbar * 100
    #else
      sample[i/2] = (long)97400;
    #endif
      base_pressure += sample[i/2];  // base_pressure is only used for arduino simple serial plotter
      //sample[i] = 0;
      //Serial.println(sample[i/2]); // pressure in mbar
      //delay(T);
    }
    base_pressure /= N*2;
    } **/

  // pre-fill sample array with pressures
  base_pressure = 0;
  for (int i = 0; i < N; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
#ifdef BMP180
    sample[i] = bmp.readPressure();  // read pressure in Pa 
#endif
#ifdef MPR
    sample[i] = mpr.readPressure(PA);
    //for (int s=0; s<6; s++) {
    //  sample[i] += mpr.readPressure(PA);  // read pressure in Pa
    //}
    //sample[i] = sample[i]/7;
#endif
#ifdef FAKE_PRESSURE
    sample[i] = 97400;  // fake pressure
#endif

    base_pressure += sample[i];  // base_pressure is only used for arduino simple serial plotter

    delay((T-8)/2);  // sample at about the same rate as normal
    digitalWrite(LED_BUILTIN, LOW);
    delay((T-8)/2);
  }
  base_pressure /= N;  // base_pressure is average of N readings

  if (DEBUG) {
    Serial.print(F("*** base_pressure="));
    Serial.println(base_pressure);
    Serial.println(F("*** End setup()"));
  }

  n = 0;  // oldest sample, first to be replaced in buffer; n is the index into the circular buffer of pressures

  // set initial state of controller
  active_state = PRELAUNCH;
  // LED blinky parameters for initial state
  LED_period = 1000;
  LED_duration = 50;

  // initialize launch location
  launch_lat = 0;
  launch_lon = 0;
  launch_alt = 0;

  // here we test the haversine function with known values to generate a known result
  /* if (0) {
    const double lat1 = 51500700;
    const double lon1 = 124600;
    const double lat2 = 40689200;
    const double lon2 = 74044500;
    Serial.print(F("haversine test (5574.84): "));
    Serial.println(haversine(lat1, lon1, lat2, lon2));
  } */

}

void loop() {
  long current_pressure;
  long pressure_sample;
  long rise_rate;
  static long rise_rate_running_sum;
  static unsigned long cut_time;
  static unsigned long last_update_millis;
  static unsigned long last_sample_millis;
  static unsigned long loop_timer;
  static unsigned long loop_total;
  static unsigned int loop_count;

  loop_timer=millis();
  
  // read command from satellite radio
  //    if cut command received, active_state = CUT_INIT
  //    if PING command received, return PONG response
  //    see command[] array for full list; help displays list
  serialCommands.readSerial();

  // read GPS string/update GPS structure here
  while (gpsSerial.available()) {
    if (nmea.process(gpsSerial.read())) {
      // do something if a new NMEA sentence has arrived
      // serialCommands.getSerial().println(nmea.getSecond());
    }
  }

  // sample pressure periodically by detecting rollover of period
  unsigned long this_sample_millis = millis() % T;
  if (this_sample_millis < last_sample_millis) {

  // read new pressure into circular buffer position n
  // increment n
  // set rise_rate to 0
  // starting at n (oldest sample), continuing to n+(N-1) mod N (aka n-1)
  //    calculate filter*sample, add to rise_rate (FIR filter)
  //  scale rise_rate to mbar/sec??
  // that's the rise/fall rate. rise is positive, fall negative

#ifdef BMP180
    pressure_sample = bmp.readPressure();  // pressure in Pa (~97000 Pa in Norman, OK)
#endif
#ifdef MPR
    pressure_sample = mpr.readPressure(PA);
#endif
#ifdef FAKE_PRESSURE
    pressure_sample = 97400;
#endif
    sample[n] = pressure_sample;

    // increment the circular buffer index
    n++;
    if (n >= N) {
      n = 0;
    }

    // apply linear regression filter to calculate rise rate
    rise_rate = 0;
    current_pressure = 0;
    for (int i = 0; i < N; i++) {
      rise_rate += (filter[i] * sample[ (n + i) % N ]);
      current_pressure += sample[i];
      if (0) {  // debug code
        Serial.print("filter[");
        Serial.print(i);
        Serial.print("]: (");
        Serial.print(filter[i]);
        Serial.print(") * ");
        Serial.print("sample[");
        Serial.print( (n + i) % N );
        Serial.print("]: (");
        Serial.print(sample[(n + i) % N]);
        Serial.print(") = ");
        Serial.println(filter[i] * sample[ (n + i) % N]);
      }
    }
    if (0) {  // more debug code
      Serial.print(F("rise_rate: "));
      Serial.println(rise_rate);
    }

    //rise_rate_running_sum += rise_rate;
  
    current_pressure /= N;  // moving average of pressure
    rise_rate = rise_rate>>10 ;  // rise_rate scaled to make up for integer filter coefficients

    if (DEBUG_PLOT) {
      Serial.print(current_pressure-base_pressure);
      Serial.print(F(" "));
      Serial.print(rise_rate);
      Serial.println(F(""));
    }

    if (DEBUG_SENSOR) {
      Serial.print(F("AvePres: "));
      Serial.print(current_pressure);
      Serial.print(F("\t"));
      Serial.print(F("PressSamp: "));
      Serial.print(pressure_sample);
      Serial.print(F("\t"));
      Serial.print(F("RiseRate: "));
      Serial.println(rise_rate);
      //Serial.print(F("\tSum: "));
      //Serial.println(rise_rate_running_sum);
    }
  }
  
  // update pressure sample time so we can detect overflow on next loop iteration
  last_sample_millis = this_sample_millis;

  unsigned long this_update_millis = (millis() % (1000 * update_interval));
  // send status message periodically via HW serial / satellite
  if ( this_update_millis < last_update_millis) {
    serialCommands.getSerial().print(millis() / 1000);
    serialCommands.getSerial().print(F(","));
    serialCommands.getSerial().print(active_state);
    serialCommands.getSerial().print(F(","));
    int c = nmea.getMinute();
    if (c < 10) {
      serialCommands.getSerial().print(F("0")); // leading zero
    }
    serialCommands.getSerial().print(c);
    serialCommands.getSerial().print(F(":"));
    c = nmea.getSecond();
    if (c < 10) {
      serialCommands.getSerial().print(F("0")); // leading zero
    }
    serialCommands.getSerial().print(c);
    serialCommands.getSerial().print(F(","));
    serialCommands.getSerial().print(current_pressure);
    serialCommands.getSerial().print(F(","));
    serialCommands.getSerial().print(rise_rate);
    serialCommands.getSerial().print(F(","));
    serialCommands.getSerial().print(nmea.getLatitude());
    serialCommands.getSerial().print(F(","));
    serialCommands.getSerial().print(nmea.getLongitude());
    serialCommands.getSerial().print(F(","));
    if (nmea.isValid()) {
      serialCommands.getSerial().println(haversine(launch_lat, launch_lon, nmea.getLatitude(), nmea.getLongitude()));
    } else {
      serialCommands.getSerial().println(F("nan"));  // no GPS lock, so no accurate distance available
    }

  }
  
  // update update time so we can detect overflow on next loop iteration
  last_update_millis = this_update_millis;
  
  // LED update
  digitalWrite(LED_BUILTIN, (millis() % LED_period) < LED_duration);

  // GPS LED update; blink if valid, otherwise off
  digitalWrite(LED_GPS, ( nmea.isValid() && (millis() % GPS_LED_period) < GPS_LED_duration));

  switch (active_state) {
    case PRELAUNCH:  // short flash, 1Hz
      if (rise_rate > rise_rate_threshold) {
        launch_time = (millis() - FILTER_DELAY/2);
        LED_period = 500;  // long slow blink
        LED_duration = 450;
        active_state = LETDOWN_INIT;
        if (DEBUG) {
          Serial.print(F("LAUNCH ESTIMATE: "));
          Serial.println(launch_time/1000);
          Serial.print(F("LAUNCH DETECT: "));
          Serial.println(millis()/1000);
          Serial.print(F("LAUNCH LATITUDE: "));
          Serial.println(launch_lat);
          Serial.print(F("LAUNCH LONGITUDE: "));
          Serial.println(launch_lon);
        }
        // turn off GPS valid LED, because at this point it's too late to get a good launch coordinate.
        //   ... and the device is flying away before the GPS was locked.  OOPS
      } else {
        // update launch location if we have a lock
        // updating many times while in prelaunch mode means we have many bites at the apple to get a good gps location
        // altitude will be the altitude at which the filter detects a launch (in the air) but not far off
        if (nmea.isValid()) {
          launch_lat = nmea.getLatitude();  // millionths of degrees
          launch_lon = nmea.getLongitude(); // millionths of degrees
          nmea.getAltitude(launch_alt);  // altitude MSL
        }
      }
      break;

    case LETDOWN_INIT:  // long flashes 2Hz
      if ( ((millis() - (unsigned long)launch_time)) > (unsigned long)letdown_delay * 1000) {
        digitalWrite(CUTTER, LOW);
        digitalWrite(MOTOR, HIGH);
        LED_period = 100;
        LED_duration = 50;
        active_state = LETDOWN_ACTIVE;
        if (DEBUG) {
          Serial.print(F("LETDOWN ON: "));
          Serial.println(millis()/1000);
        }
      }
      break;

    case LETDOWN_ACTIVE:  // letting down, 10Hz flashes
      if ( (millis() - (unsigned long)launch_time) > ((unsigned long)letdown_delay * 1000 + (unsigned long)letdown_duration * 1000) ) {
        digitalWrite(MOTOR, LOW);
        LED_period = 2000;
        LED_duration = 50;
        active_state = FLIGHT;
        if (DEBUG) {
          Serial.print(F("LETDOWN STOPPED: "));
          Serial.println(millis()/1000);
        }
      }
      break;

    case FLIGHT:  // short flash, 0.5Hz
      // check time limit; set max_flight_duration to 0 to disable time initiated cut
      if ((max_flight_duration > 0) && ( (millis() - (unsigned long)launch_time) > (unsigned long)max_flight_duration * 1000 )) {
        active_state = CUT_INIT;
        break;
      }
      // check pressure ceiling; set cut_pressure to 0 to disable pressure initiated cut
      // if ((cut_pressure > 0) && (current_pressure < (long)cut_pressure * 100)) {
      if ((cut_pressure > 0) && (current_pressure < (long)cut_pressure*100)) {
        active_state = CUT_INIT;
        break;
      }
      // check geofence here
      if (nmea.isValid()) {
        // compute distance and compare to max distance downrange
        if ( max_distance>0 && (haversine(launch_lat, launch_lon, nmea.getLatitude(), nmea.getLongitude()) > max_distance)) {
          active_state = CUT_INIT;
          break;
        }
        if ( max_latitude!=0 && (nmea.getLatitude() > max_latitude)) {
          active_state = CUT_INIT;
          break; 
        }
        if ( min_latitude!=0 && (nmea.getLatitude() < min_latitude)) {
          active_state = CUT_INIT;
          break; 
        }
        if ( max_longitude!=0 && (nmea.getLongitude() > max_longitude)) {
          active_state = CUT_INIT;
          break; 
        }
        if ( max_longitude!=0 && (nmea.getLongitude() < min_longitude)) {
          active_state = CUT_INIT;
          break; 
        }
        
        // compare lat/lon to lat/lon min and max
      }
      break;

    case CUT_INIT:
      digitalWrite(MOTOR, LOW);
      digitalWrite(CUTTER, HIGH);
      cut_time = millis();
      LED_period = 200;
      LED_duration = 100;
      active_state = CUT_ACTIVE;
      if (DEBUG) {
        Serial.print(F("CUTDOWN ON: "));
        Serial.println(millis()/1000);
      }
      break;

    case CUT_ACTIVE:  // short flash, 5 Hz.
      if ( (millis() - (unsigned long)cut_time) > (unsigned long)cut_duration * 1000) {
        digitalWrite(CUTTER, LOW);
        LED_period = 4000;
        LED_duration = 100;
        active_state = POST_FLIGHT;
        if (DEBUG) {
          Serial.print(F("CUTDOWN OFF: "));
          Serial.println(millis()/1000);
        }
      }
      break;

    case POST_FLIGHT:  // short flash, 0.25Hz
      digitalWrite(CUTTER, LOW);
      digitalWrite(MOTOR, LOW);
      break;

    default:
      break;
  }

  //if (DEBUG_SENSOR) {
  //  Serial.println(F(""));
  //}

  delay(1);
  //unsigned long foo = millis()-loop_timer;
  //if (foo > 3) {
  //  Serial.println(millis()-loop_timer);
  //}
  
}
