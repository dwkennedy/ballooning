/*
  detect balloon launch by thresholding rise rate
  Doug Kennedy
  for BMP180 or similar sensor
  ... and a bunch of other stuff
*/

/* subset of commands for satellite
 *  PNG{stuff}             return binary string {stuff}
 *  PRG{struct config}     program EEPROM with config structure
 *  LET{uint16_t time)     activate MOTOR for time milliseconds
 *  CUT{unit_id}           initiate cutting immediately if unit_id matches
 */

//  Board manager URL for programming bare ATMEGA328P
//   http://www.bamfordresearch.com/files/package_jb23_insystemmcu_index.json
//  Adafruit library for Honeywell MPR pressure sensor that I modified
//   https://github.com/adafruit/Adafruit_MPRLS
//  Sparkfun iridium SBD
//   https://github.com/sparkfun/SparkFun_IridiumSBD_I2C_Arduino_Library


// if using BMP180 pressure sensor
//#define BMP180

// if using MPRLS pressure sensor
//#define MPR

// if using MPL3115A2 pressure sensor
#define MPL3115A2

// if using FAKE pressure for testing
//#define FAKE_PRESSURE

#ifdef MPR
#include <Wire.h>
//#include <SparkFun_MicroPressure.h>
/*
 * Initialize Constructor
 * Optional parameters:
 *  - EOC_PIN: End Of Conversion (defualt: -1)
 *  - RST_PIN: Reset (defualt: -1)
 *  - MIN_PSI: Minimum Pressure (default: 0 PSI)
 *  - MAX_PSI: Maximum Pressure (default: 25 PSI)
 */
//SparkFun_MicroPressure mpr(EOC_PIN, RST_PIN, MIN_PSI, MAX_PSI);
//SparkFun_MicroPressure mpr; // Use default values with reset and EOC pins unused

#include "Doug_Adafruit_MPRLS.h"
#define RESET_PIN 5  // set to any GPIO pin # to hard-reset on begin()
#define EOC_PIN -1   // set to any GPIO pin to read end-of-conversion by pin
Adafruit_MPRLS mpr = Adafruit_MPRLS(RESET_PIN, EOC_PIN);
#endif 

#ifdef MPL3115A2
#include <Adafruit_MPL3115A2.h>
Adafruit_MPL3115A2 baro;
#endif

#include <SoftwareSerial.h>
//#include <NeoSWSerial.h>
// #define _SS_MAX_RX_BUFF 128 // RX buffer size
#include <IridiumSBD.h>
//#include <MicroNMEA.h>
#include <TinyGPSPlus.h> // NMEA parsing: http://arduiniana.org

#include <EEPROM.h>
#define EEPROM_BASE_ADDR 40
#include <math.h>
#include "ballooning_state_engine.h"

// debug setup() and state engine
#define DEBUG

// debug iridium sdb
//#define DEBUG_SDB

// debug sensor and filter
//#define DEBUG_SENSOR

// debug plot the rise_rate
//#define DEBUG_PLOT

// print out average loop time in ms, current loop time
//#define DEBUG_LOOP_INTERVAL

// uncomment to calculate filter coefficents and dump to serial
//#define DEBUG_FILTER

// Motor port
#define MOTOR (5)

// hot-wire cutter port
#define CUTTER (4)

// RED LED port
#define LED_RED (13)
// GREEN/GPS LED port
#define LED_GREEN (12)

// battery voltage analog input
#define BATT_SENSE A3
// voltage at which to skip turn-on motor/cutter pulse
#define BATT_VOLTAGE_THRESHOLD (8.5)

// length of filter (N must be odd)
#define N (33)

// make this interval between pressure samples; filter total time is N*T
#define T (800)

// filter delay in milliseconds; we trigger before filter gets filled up with rising values
//  so estimate at half of the filter total time
#define FILTER_DELAY (N*T/2)

// sink rate to trigger event - not presently used; could be used to detect balloon pop.  less than -100 could be freefall
#define SINK_RATE_THRESHOLD (-100)

// parameters to adjust for flight.  see ballooning_state_engine.h
struct eeprom_config config;

// how often to send GPS packet to console
static const uint16_t update_interval_console = 5;

/*
// address/serial number of unit
uint16_t unit_id;

// how many seconds before activation of the let down motor
static unsigned int letdown_delay;

// how long to run the motor (seconds)
static unsigned int cut_duration;

// when to terminate the flight (seconds)
static unsigned int max_flight_duration;

// pressure at which to cut, mbar
static unsigned int cut_pressure;

// how long to cut (seconds)
static unsigned int cut_duration;

// rise rate to trigger event. should be higher than noise but less than balloon rise rate. depends on sample rate and filter
// elevator in NWC peaks out around 60.  noise level is +/- 10
static unsigned int rise_rate_threshold;

// max distance balloon can travel before cut (over surface of earth, in meters)
static unsigned long max_distance;

// geofencing min/max lat/long
static long min_latitude;
static long max_latitude;
static long min_longitude;
static long max_longitude;
*/

// lines on rockblock labelled backwards; TX on RB is wired to TX on 328P
#define satSerial Serial
#define RING_PIN (2)
#define SLEEP_PIN (3)
SoftwareSerial gpsSerial (6,7); // RX, TX
SoftwareSerial consoleSerial (10,9); // RX, TX 
//IridiumSBD modem(satSerial, SLEEP_PIN, RING_PIN);  // Declare the IridiumSBD object
IridiumSBD modem(satSerial, -1, RING_PIN);  // Declare the IridiumSBD object
TinyGPSPlus gps;  // Declare the TinyGPSPlus object

// vector of filter coefficients
#ifdef DEBUG_FILTER
int filter[N];
#else
const int filter[N] = {53,50,46,43,40,36,33,30,26,23,20,16,13,10,6,3,
                       0,-3,-6,-10,-13,-16,-20,-23,-26,-30,-33,-36,-40,-43,-46,-50,-53};
#endif

// circular buffer of N samples
unsigned long sample[N];
unsigned long base_pressure;
// index of circular buffer, 0 to N-1
uint16_t n = 0;

// state engine labels
enum state
{
  PRELAUNCH,
  LETDOWN_INIT,
  LETDOWN_ACTIVE,
  FLIGHT,
  CUT_INIT,
  CUT_ACTIVE,
  POST_FLIGHT,
  SETUP,
};

typedef enum state state; /* also a typedef of same identifier */

state active_state = SETUP;  // initial state

bool loopEnabled = false; // turn on/off SBD callback

uint16_t LED_period = 2000;  // interval to blink LED
uint16_t LED_duration = 50;  // duration of LED blink
const uint16_t GPS_LED_period = 2000;  // interval to blink GPS locked LED
const uint16_t GPS_LED_duration = 2100;  // duration to blink GPS locked LED

static uint32_t launch_time = 0;  // so we can time letdown and flight time.  static is not needed in global vars?
float launch_lat;
float launch_lon;
float launch_alt;
bool gps_isvalid;  // flag to indiate valid lock or not

uint16_t batt_voltage;
uint32_t current_pressure;
uint32_t pressure_sample;
float temperature_sample;
int32_t rise_rate;

uint8_t MT_buffer[96];  // buffer for mobile terminated messages (to rockblock)
size_t MT_buffer_size; // = sizeof(MT_buffer);
uint8_t status;  // return status of satellite commands
int16_t x;  // rando temp variable 
uint32_t timer;  // store millis() value for timing
  
// declare the reset function so we can restart program
void(* resetFunc) (void) = 0;

// print 255 as FF, 10 as 0A, etc
void print_hex_char(Stream &out, uint8_t c) {
  if ( c < 0x10 ) { // add leading zero to hex numbers < 0x10. 0x01, 0x02... 0x0F
       out.write('0');
    }
    out.print(c, HEX);
}

// print a whole buffer as hex
void print_hex_buffer(Stream &out, uint8_t* c, uint16_t count) {
  for(int i=0; i<count; i++) {
    print_hex_char(out, c[i]);
  }
}

uint8_t process_cmd(uint8_t buffer[], size_t buffer_size) {

    #ifdef DEBUG_PROCESS_CMD
    print_hex_buffer(consoleSerial,buffer,buffer_size);
    consoleSerial.write(':');
    consoleSerial.println(buffer_size);
    #endif
    
    // CUT command:  CUT (3 bytes)
    //if ((buffer_size == 5) && !strncmp(buffer, "CUT", 3) && !strncmp(buffer+3, config.unit_id, 2)) {
    if ((buffer_size == 3) && !strncmp(buffer, "CUT", 3) ) {
      #ifdef DEBUG
      consoleSerial.print(F("*** CUT "));
      consoleSerial.println(config.cut_duration);
      #endif
      // do cut stuff here
      if (active_state == SETUP) {
        timer = millis();
        while ( (millis()-timer) < config.cut_duration) {
          digitalWrite(LED_RED, LOW);
          digitalWrite(LED_GREEN, LOW);
          digitalWrite(CUTTER, HIGH);
        }
        digitalWrite(CUTTER, LOW);
      } else {
        active_state = CUT_INIT;
      }
      return(1);  // good command status
    }

    // LET command: LET (3 bytes)
    else if ((buffer_size == 3) && !strncmp(buffer, "LET", 3)) {
      // do letdown stuff here  
      #ifdef DEBUG
      consoleSerial.print(F("*** LET "));
      consoleSerial.println(config.letdown_duration);
      #endif
      if (active_state == SETUP) {
        timer = millis();
        while ( ((millis()-timer)/(uint32_t)1000) < config.letdown_duration) {
          digitalWrite(LED_RED, LOW);
          digitalWrite(LED_GREEN, LOW);
          digitalWrite(MOTOR, HIGH);
        }
        digitalWrite(MOTOR, LOW);
      } else {
        active_state = LETDOWN_INIT;
      }
      return(1);
    }
    
    // PRG commmand:  store configuration structure to EEPROM
    else if ((buffer_size == (36+3)) && !strncmp(buffer, "PRG", 3)) {
      #ifdef DEBUG
      consoleSerial.print(F("*** PRG "));
      print_hex_buffer(consoleSerial, buffer+3, buffer_size-3);
      consoleSerial.println();
      #endif
      // write config to EEPROM
      //EEPROM.put(EEPROM_BASE_ADDR, (struct eeprom_config)(buffer+3) );
      memcpy((uint8_t *)&config, buffer+3, sizeof(config));  // update config in memory
      EEPROM.put(EEPROM_BASE_ADDR, config);  // write new config to EEPROM
      // EEPROM.get(EEPROM_BASE_ADDR, config);  // read back from EEPROM
      return(1);
    }

    // UPD command:  change satellite update interval to xx seconds
    else if ((buffer_size == 5) && !strncmp(buffer, "UPD", 3)) {
      config.update_interval_satellite = (uint16_t)*(buffer+3);
      #ifdef DEBUG
      consoleSerial.print(F("*** UPD "));
      consoleSerial.print(config.update_interval_satellite);
      consoleSerial.println();
      #endif
      return(1);
    }

    else if ((buffer_size==3) && !strncmp(buffer, "END", 3)) {
      return(2);   // exit serial command mode
    }
    
    return(0);  // no valid command found
}

void error_flash(uint8_t flashes, uint8_t repeats) {
  for (uint8_t repeat = 0; repeat < repeats; repeat++) {
    for (uint8_t count = 0; count < flashes; count++) {
        digitalWrite(LED_RED, HIGH);  // blink LED to indicate problem
        delay(100);
        digitalWrite(LED_RED, LOW);
        delay(100);
    }
    delay(300);  // extra space between repetition of code
  }
}

float read_batt_voltage() {
  return(analogRead(BATT_SENSE)/77.57575758);
}

void setup() {

  // configure control ports and make sure they're off
  pinMode(MOTOR, OUTPUT);  // motor control port
  digitalWrite(MOTOR, LOW);  // motor off
  pinMode(CUTTER, OUTPUT);  // cutter control port
  digitalWrite(CUTTER, LOW);  // cutter off

  // set up GPS LED
  pinMode(LED_GREEN, OUTPUT);
  digitalWrite(LED_GREEN, HIGH);
  // set up output indicator LED
  pinMode(LED_RED, OUTPUT);
  digitalWrite(LED_RED, HIGH);

  // iridium modem ring signal
  pinMode(RING_PIN, INPUT);

  // iridium modem sleep signal; +5V = NOT ASLEEP
  pinMode(SLEEP_PIN, OUTPUT);
  digitalWrite(SLEEP_PIN, HIGH);

  analogReference(EXTERNAL); // use AREF for reference voltage

#ifdef MPR
  // pressure sensor reset
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, LOW); // initiate reset
#endif

  active_state = SETUP;  // we in setup mode now
  
  // set serial port baud rates
  satSerial.begin(19200);  // hardware serial is connected to satellite radio
  consoleSerial.begin(19200);  // software serial to debugging console
  consoleSerial.listen();
  
    // avoid double setup and wait for LED test
  delay(6000);
  
  #ifdef DEBUG
    consoleSerial.println(F("\r\n*** Start setup()"));
  #endif
  
  // turn off all LED, on since boot for 5 sec (lamp test)
  digitalWrite(LED_RED, LOW);  // turn on RED LED
  digitalWrite(LED_GREEN, LOW);   // turn on GREEN LED
  
  #ifdef DEBUG
  if ( read_batt_voltage() > BATT_VOLTAGE_THRESHOLD ) { 
    consoleSerial.print(F("*** BATT "));
    consoleSerial.println(read_batt_voltage());
    consoleSerial.println(F("*** MOTOR ON"));
    digitalWrite(MOTOR, HIGH);  // turn on motor
    delay(50);
    consoleSerial.print(F("*** BATT "));
    consoleSerial.println(read_batt_voltage());
    digitalWrite(MOTOR, LOW);  // turn off motor
    consoleSerial.println(F("*** MOTOR OFF"));
    consoleSerial.println(F("*** CUTTER ON"));
    digitalWrite(CUTTER, HIGH);  // turn on cutter
    delay(100);
    consoleSerial.print(F("*** BATT "));
    consoleSerial.println(read_batt_voltage());
    digitalWrite(CUTTER, LOW);  // turn off cutter
    consoleSerial.println(F("*** CUTTER OFF"));
  }
  
  #endif

  #ifdef MPR
  digitalWrite(RESET_PIN, HIGH);  // take pressure sensor out of reset
  #endif
  //delay(1000);
  

  
  // read parameters from EEPROM-- things we can adjust for flight

  //   force initial config by making unit_id 0xFFFF
  //EEPROM.put(EEPROM_BASE_ADDR, (uint16_t)0xFFFF);
  
  #ifdef DEBUG
  //consoleSerial.println("*** reading EEPROM");
  #endif
  EEPROM.get(EEPROM_BASE_ADDR, config); 
  #ifdef DEBUG
  consoleSerial.print(F("*** EEPROM "));
  print_hex_buffer(consoleSerial,(uint8_t *)&config, sizeof(struct eeprom_config));
  consoleSerial.println();
  #endif

/*  EEPROM.get(EEPROM_BASE_ADDR, config.unit_id);
  #ifdef DEBUG
  consoleSerial.print("*** unit_id: ");
  consoleSerial.println(config.unit_id, HEX);
  #endif 
*/
  
  // check for uninitialized EEPROM
  if (config.unit_id == 0xFFFF) { // not initialized, so lets initialize it
    #ifdef DEBUG
    consoleSerial.println(F("*** Initializing EEPROM"));
    #endif
    config.unit_id = 0;
    config.letdown_delay = 30;  // seconds; positive: delay after launch detect, negative: delay after power on
    config.cut_duration = 3000;  // milliseconds
    config.max_flight_duration = 60;
    config.cut_pressure = 0;
    config.letdown_duration = 5;  //seconds
    config.rise_rate_threshold = 85;
    config.update_interval_satellite = 120;
    config.max_distance = 0;
    config.min_latitude = 0;
    config.max_latitude = 0;
    config.min_longitude = 0;
    config.max_longitude = 0; 
    EEPROM.put(EEPROM_BASE_ADDR, config);  // write config to EEPROM
  } 
  
  // check for commands on serial port;
  #ifdef DEBUG
  consoleSerial.println(F("*** Wait for serial cmd"));
  #endif
  for(uint8_t i=0; i<10; i++) {
    digitalWrite(LED_RED, i%2);  // blink LEDs during programming phase
    digitalWrite(LED_GREEN, !(i%2));
    
    uint8_t counter = 0;
    timer = millis();  // timer for end of message
    while( ((millis()-timer) < 1000L) ) {
      if ( (x = consoleSerial.read()) > -1) {
        //consoleSerial.print(F("$"));  // echo character for debug
        timer = millis();  // reset timer
        MT_buffer[counter++] = (uint8_t)x;   // load each serial rx character into cmd buffer until timeout.  use sat buffer for this purpose
      }
    }
    if (counter>0) {  // if some bytes were read
      if (0) {
        print_hex_buffer(consoleSerial,MT_buffer,counter);
        consoleSerial.println();
        delay(100);
      }
      status = process_cmd(MT_buffer, counter);  // do the thing
      if (status==0) {
        consoleSerial.println(F("ERR"));  // process command returned error status
        //print_hex_buffer(consoleSerial, MT_buffer, counter);
      } else if (status==1) {
        consoleSerial.println(F("OK"));  // good command receive
        //print_hex_buffer(consoleSerial, MT_buffer, counter);
      } else if (status==2) {
        consoleSerial.println(F("END"));  // end command received
        //print_hex_buffer(consoleSerial, (uint8_t)&config, sizeof(struct eeprom_config));  // config struct
        break;
      }
    }
  } 

  digitalWrite(LED_RED, LOW);  // make sure leds are off
  digitalWrite(LED_GREEN, LOW);

  #ifdef DEBUG
     consoleSerial.print(F("*** CFG "));
     print_hex_buffer(consoleSerial, (uint8_t *) &config, sizeof(eeprom_config));
     consoleSerial.println();
  #endif
  
  gpsSerial.begin(9600);  // software serial to gps receiver
  gpsSerial.listen();

#ifdef MPL3115A2
  #ifdef DEBUG
  consoleSerial.println(F("*** Start MPL3115A2"));
  #endif
  if (!baro.begin()) {
    consoleSerial.println(F("*** Cannot connect to MPL3115A2, resetting"));
    error_flash(3,3);
    resetFunc();  // go back to setup()
  }
  baro.setMode(MPL3115A2_BAROMETER);
#endif

//  #ifdef DEBUG
//  consoleSerial.println(F("*** Connected to pressure sensor"));
//  #endif
  
  // Begin satellite modem operation
  #ifdef DEBUG
  consoleSerial.println(F("*** Start Iridium modem"));
  #endif
  loopEnabled = false;  // disable SBD callback during setup
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);  // high power
  //modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);  // for low power
  status = modem.begin();
  if (status != ISBD_SUCCESS)
  {
    consoleSerial.print(F("*** Iridium begin failed: error "));
    consoleSerial.println(status);
    if (status == ISBD_NO_MODEM_DETECTED) {
      consoleSerial.println(F("*** No modem detected: check wiring."));
      error_flash(2,3);
      resetFunc();
    }
  }

  #ifdef DEBUG
  // Get the IMEI
  char IMEI[16];
  status = modem.getIMEI(IMEI, sizeof(IMEI));
  if (status != ISBD_SUCCESS)
  {
     consoleSerial.print(F("*** getIMEI failed: error "));
     consoleSerial.println(status);
     return;
  }
  consoleSerial.print(F("*** IMEI is "));
  consoleSerial.println(IMEI);
  #endif
  
  #ifdef DEBUG_SDB
  // Print the firmware revision
  char version[12];
  status = modem.getFirmwareVersion(version, sizeof(version));
  if (status != ISBD_SUCCESS)
  {
     consoleSerial.print(F("*** FirmwareVersion failed: error "));
     consoleSerial.println(status);
     return;
  }
  consoleSerial.print(F("*** Firmware Version is "));
  consoleSerial.println(version);

  // Check the signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  int signalQuality;
  status = modem.getSignalQuality(signalQuality);
  if (status != ISBD_SUCCESS)
  {
    consoleSerial.print(F("*** getSignalQuality failed: error "));
    consoleSerial.println(status);
    return;
  }

  consoleSerial.print(F("*** Signal quality "));
  consoleSerial.print(signalQuality);
  consoleSerial.println(F(" out of 5"));

  #endif
  
  // calculate filter coefficients
  //  (could be done statically if N is fixed)
  #ifdef DEBUG_FILTER
    consoleSerial.print(F("filter[] = {"));

    for (int i = 0; i < N; i++) {
      //filter[i] = (double)( -(((double)((double)i-(((double)N-1L)/2L))/(double)N)) );
      //filter[i] = 1;
      // filter coeffs with double -- works
      // filter[i] = -(12.0 * (double)i-6.0 * ((double)N-1.0)) / ((double)N * ((double)N*(double)N - 1.0)); // - (12*i - 6*(N-1))   /   N*(N^2-1)
      // filter[i] = 10000*filter[i];  // scale filter coefficients, delta Pressure / sample period
      // try filter coeefs with longs, 1000000 is in there to ensure the coefficients don't get wiped out due to lack of precision
      filter[i] = (-10000.0 * (12.0 * (double)i - 6.0 * ((double)N - 1.0))) / ((double)N * ((double)N * (double)N - 1.0));
      if (i<(N-1)) {
        consoleSerial.print(filter[i]);
        consoleSerial.print(F(","));
      }
    }

    consoleSerial.print(filter[N-1]);
    consoleSerial.println(F("};"));
  #endif



#ifdef BMP180
  while (!bmp.begin()) {
    consoleSerial.println("Could not find a valid BMP085/180 sensor, check wiring");
    error_flash(3,3);
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
  //#if defined(WIRE_HAS_TIMEOUT)
  //Wire.setWireTimeout(3000 /* us */, true /* reset_on_timeout */);
  //#endif
  while(!mpr.begin())
  {
    consoleSerial.println(F("*** Cannot connect to MPRLS sensor, resetting"));
    error_flash(3,3);
    resetFunc();  // go back to setup()
  }
#endif



  #ifdef DEBUG_PLOT   // print out a bunch of zeros for the sake of simple arduino serial plotter
  for (int i = 0; i < 500; i++) {
    consoleSerial.println("0 0");
  }
  #endif

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
      //consoleSerial.println(sample[i/2]); // pressure in mbar
      //delay(T);
    }
    base_pressure /= N*2;
    } **/

  // pre-fill sample array with pressures
  #ifdef DEBUG
  consoleSerial.print(F("*** Initializing digital filter"));
  #endif
  base_pressure = 0;
  for (int i = 0; i < N; i++) {
    digitalWrite(LED_RED, HIGH);
  #ifdef BMP180
    sample[i] = bmp.readPressure();  // read pressure in Pa 
  #endif
  #ifdef FAKE_PRESSURE
    sample[i] = 97400;  // fake pressure
  #endif

  #ifdef MPR
    while ( (sample[i] = mpr.readIntPressure()) == 0L) {
      delay(50);
      digitalWrite(LED_RED, LOW);  // bad sample, turn off LED and try again
    }
  #endif MPR  

  #ifdef MPL3115A2
    sample[i] = baro.getPressure()*100;
  #endif
  
    base_pressure += sample[i];  // base_pressure is only used for arduino simple serial plotter

    //delay((T-8)/4);  // sample at about 2x the same rate as normal
    digitalWrite(LED_RED, LOW);
    //delay((T-8)/4);
    #ifdef DEBUG
    consoleSerial.print(F("."));
    #endif
  }

  #ifdef DEBUG
  consoleSerial.println();
  #endif
  
  base_pressure /= N;  // base_pressure is average of N readings
  #ifdef DEBUG
    consoleSerial.print(F("*** base_pressure="));
    consoleSerial.println(base_pressure);
    consoleSerial.println(F("*** End setup()"));
    consoleSerial.println(F("time,state,ring,pressure,rise_rate,lat,lon,distance,gps_rx,voltage"));
  #endif

  n = 0;  // index to oldest sample, first to be replaced in buffer; n is the index into the circular buffer of pressures

  // set initial state of controller
  active_state = PRELAUNCH;
  // LED blinky parameters for initial state
  LED_period = 1000;
  LED_duration = 50;

  // initialize launch location
  launch_lat = 0;
  launch_lon = 0;
  launch_alt = 0;

  // enable SBD callback
  loopEnabled = true;

  // setup timer for LED blinking
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

void loop() {

  static uint32_t last_update_millis, this_update_millis;
  struct sat_message beacon;
  
  // send/receive sat message here, which will repeatedly call ISBDCallback while idle

  //  manually do callback at least once before sending a message; probe at least once per loop iteration. 
  ISBDCallback();  
  
  this_update_millis = (millis() % ((uint32_t)1000*(uint32_t)config.update_interval_satellite));

  // when it's update time or there is a message waiting for RX, build the TX status message : 
  // example message: *** SDB TX: 000B21107916E8B8192A07531FA6E1B99101D7E100000 size 31

  // send message if:  1) ring pin active, 2) message waiting, 3)  satellite update time elapsed
  if ( (!digitalRead(RING_PIN))
        || (modem.getWaitingMessageCount() > 0)
        || (config.update_interval_satellite && (this_update_millis < last_update_millis))) {

    #ifdef DEBUG
    if (!digitalRead(RING_PIN)) {
        consoleSerial.println(F("*** SDB RING"));
    }
    #endif
    beacon.unit_id = config.unit_id;
    beacon.state = active_state;
    beacon.second = gps.time.second();
    beacon.minute = gps.time.minute();
    beacon.hour = gps.time.hour();
    beacon.day = gps.date.day();
    beacon.month = gps.date.month();
    beacon.year = (uint8_t)(gps.date.year()-2000);
    beacon.latitude = (uint32_t)(gps.location.lat()*(uint32_t)1000000);
    beacon.longitude = (uint32_t)(gps.location.lng()*(uint32_t)1000000);
    beacon.altitude = (int32_t)gps.altitude.meters();
    beacon.course = (int16_t)(gps.course.value());  // 100ths of degree
    beacon.speed = (int16_t)(gps.speed.value()); // 100ths of knot
    beacon.pressure = current_pressure;
    beacon.temperature = (int16_t)(temperature_sample*10); //current_temp*10;
    beacon.humidity = 0*10; //current_humidity*10;
    beacon.batt_voltage = analogRead(BATT_SENSE);
    
    #ifdef DEBUG
    consoleSerial.print(F("*** SDB TX: "));
    uint8_t *pointer = (uint8_t *)&beacon;
    print_hex_buffer(consoleSerial, pointer, sizeof(beacon));
    consoleSerial.print(F(" size "));
    consoleSerial.print(sizeof(beacon));
    consoleSerial.println();
    #endif

    // sendReceiveSBDBinary(const uint8_t *txData, size_t txDataSize, uint8_t *rxBuffer, size_t &rxBufferSize)
    if (modem.isConnected()) { // Check that the Qwiic Iridium is connected
      //modem.enableSuperCapCharger(true); // Enable the super capacitor charger
      //consoleSerial.println("super cap charger on");
      //while (!modem.checkSuperCapCharger()) ; // Wait for the capacitors to charge
      //consoleSerial.println("super cap charged");
      //modem.enable9603Npower(true); // Enable power for the 9603N
      //consoleSerial.println("modem turned on");
      //modem.begin(); // Wake up the modem
      MT_buffer_size = sizeof(MT_buffer); // always reset this before receive; message size is returned in this variable
      status = modem.sendReceiveSBDBinary((uint8_t *)&beacon, sizeof(beacon), MT_buffer, MT_buffer_size); // TX/RX a message in binary
      #ifdef DEBUG
      if ((MT_buffer_size>0) && (status == 0)) {
        consoleSerial.print(F("*** SDB RX (hex): "));
        print_hex_buffer(consoleSerial, MT_buffer, MT_buffer_size);
        consoleSerial.print(F(" size "));
        consoleSerial.println(MT_buffer_size, HEX);
        consoleSerial.print(F("*** SDB RX (asc): "));
        for(int i=0; i<MT_buffer_size; i++) {
            if ( isprint(MT_buffer[i])) { // print RX message printable characters
                consoleSerial.write(MT_buffer[i]);
            } else {
                consoleSerial.write('(');
                print_hex_char(consoleSerial,MT_buffer[i]);
                consoleSerial.write(')');
            }
        }
        consoleSerial.print(F(" size "));
        consoleSerial.println(MT_buffer_size);
      }
      consoleSerial.print(F("*** SBD TX/RX status: "));
      consoleSerial.println(status);
      // Clear the Mobile Originated message buffer to avoid re-sending the message during subsequent loops
      consoleSerial.println(F("*** SBD Clearing the MO buffer"));
      #endif
      status = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
      #ifdef DEBUG
      if (status != ISBD_SUCCESS) {
          Serial.print(F("*** SBD clearBuffers failed: error "));
          Serial.println(status);
      }
      
      #endif
      //modem.sleep(); // Put the modem to sleep
      //modem.enable9603Npower(false); // Disable power for the 9603N
      //modem.enableSuperCapCharger(false); // Disable the super capacitor charger
      //modem.enable841lowPower(true); // Enable the ATtiny841's low power mode (optional)
    }

    #ifdef DEBUG
    consoleSerial.println(F("*** SDB TX/RX finished"));
    #endif

    // process incoming message

    if ((MT_buffer_size > 0) && (status == 0)) {
      //PNG command:  return bytes 
      if (!strncmp(MT_buffer, "PNG", 3)) {
        //  send MT_buffer back
        #ifdef DEBUG
          consoleSerial.println(F("*** PNG received, returning message"));
        #endif
        status = modem.sendSBDBinary((uint8_t *)&MT_buffer[3], MT_buffer_size-3); // TX/RX a message in binary
        #ifdef DEBUG
          consoleSerial.print(F("*** SBD TX status: "));
          consoleSerial.println(status);
          // Clear the Mobile Originated message buffer to avoid re-sending the message during subsequent loops
          consoleSerial.println(F("*** SBD Clearing the MO buffer"));
        #endif
        status = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
        #ifdef DEBUG
          if (status != ISBD_SUCCESS) {
            Serial.print(F("*** SBD clearBuffers failed: error "));
            Serial.println(status);
          }
        #endif
      } else {
        if ((MT_buffer_size == 3) && !strncmp(MT_buffer, "CFG", 3)) {
          // send CFG+configuration string back   
          #ifdef DEBUG
            consoleSerial.print(F("*** CFG received, returning "));
            print_hex_buffer(consoleSerial, (uint8_t *) &config, sizeof(eeprom_config));
            consoleSerial.println();
          #endif
          //memcpy((uint8_t *)&config, MT_buffer+3, sizeof(config));  // update MT_buffer to include current config
          memcpy(MT_buffer+3, (uint8_t *)&config, sizeof(config));  // update MT_buffer to include current config
          // forgot to send CFG header, please fix later DWK
          status = modem.sendSBDBinary((uint8_t *)&MT_buffer[3], MT_buffer_size+sizeof(config));  // TX CFG+binary configuration structure
          #ifdef DEBUG
            consoleSerial.print(F("*** SBD TX status: "));
            consoleSerial.println(status);
            // Clear the Mobile Originated message buffer to avoid re-sending the message during subsequent loops
            consoleSerial.println(F("*** SBD Clearing the MO buffer"));
          #endif
          status = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
          #ifdef DEBUG
            if (status != ISBD_SUCCESS) {
              Serial.print(F("*** SBD clearBuffers failed: error "));
              Serial.println(status);
            }
          #endif
        } else {
          process_cmd(MT_buffer, MT_buffer_size);        
        }
      }
    }
  }

  last_update_millis = this_update_millis;  // last_update_millis is continually updated; when this_update_millis rolls over we'll get the next message
                                            // if update_interval_satellite is 120, you'll get message attempts at 120, 240, ..., n*120, ...
}

bool ISBDCallback() {

  static long rise_rate_running_sum;
  static unsigned long cut_time;
  static unsigned long last_update_millis, this_update_millis;  // serial debug update interval
  static unsigned long last_sample_millis, this_sample_millis;  // pressure sampling interval
  //static unsigned long loop_total;
  //static unsigned int loop_count;

  #ifdef DEBUG_LOOP_INTERVAL
  static unsigned long loop_timer;
  loop_timer=millis();
  #endif

  if (!loopEnabled) {  // this is here to disable the loop while talking to satellite in setup()
    return true;
  }

  // read GPS string/update GPS structure here
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    //consoleSerial.print(c);
    if (gps.encode(c)) {
      // process new gps info here, new sentence received
    }
  }

  //  is last gps message valid + recently updated?
  gps_isvalid = (gps.location.isValid() && (gps.location.age() < 5000));

  // sample pressure periodically by detecting rollover of period ///////////////////////////////////////////////////////////////////////////////////
  this_sample_millis = millis() % T;
  if (this_sample_millis < last_sample_millis) {

  // read new pressure into circular buffer position n
  // increment n
  // set rise_rate to 0
  // starting at n (oldest sample), continuing to n+(N-1) mod N (aka n-1)
  //    calculate filter*sample, add to rise_rate (FIR filter)
  //  scale rise_rate to mbar/sec??
  // that's the rise/fall rate. rise is positive, fall negative

#ifdef FAKE_PRESSURE
    pressure_sample = 97400;
#endif
#ifdef MPR
    if ( (pressure_sample = mpr.readIntPressure()) == 0L) {
      pressure_sample = sample[((n-1) % N)];  // sample failed, so replace with similar sample
      LED_period = 100;  // make LED complain about bad pressure
      LED_duration = 50;
    }
#endif
#ifdef MPL3115A2
    pressure_sample = baro.getPressure()*100;
    temperature_sample = baro.getTemperature();  // temp in deg C
#endif

    sample[n] = pressure_sample;

    // increment the circular buffer index
    n++;
    n = n % N;

    // apply linear regression filter to calculate rise rate
    rise_rate = 0;
    current_pressure = 0;
    for (int i = 0; i < N; i++) {
      rise_rate += (filter[i] * sample[ (n + i) % N ]);
      current_pressure += sample[i];
      #ifdef DEBUG_FILTER
        consoleSerial.print("filter[");
        consoleSerial.print(i);
        consoleSerial.print("]: (");
        consoleSerial.print(filter[i]);
        consoleSerial.print(") * ");
        consoleSerial.print("sample[");
        consoleSerial.print( (n + i) % N );
        consoleSerial.print("]: (");
        consoleSerial.print(sample[(n + i) % N]);
        consoleSerial.print(") = ");
        consoleSerial.println(filter[i] * sample[ (n + i) % N]);
      #endif
    }
    #ifdef DEBUG_FILTER
      consoleSerial.print(F("rise_rate: "));
      consoleSerial.println(rise_rate);
    #endif

    //rise_rate_running_sum += rise_rate;
  
    current_pressure /= N;  // moving average of pressure
    rise_rate = rise_rate>>10 ;  // rise_rate scaled to make up for integer filter coefficients

    #ifdef DEBUG_PLOT
      consoleSerial.print(current_pressure-base_pressure);
      consoleSerial.print(F(" "));
      consoleSerial.print(rise_rate);
      consoleSerial.println(F(""));
    #endif

    #ifdef DEBUG_SENSOR
      consoleSerial.print(F("AvePres: "));
      consoleSerial.print(current_pressure);
      consoleSerial.print(F("\t"));
      consoleSerial.print(F("PressSamp: "));
      consoleSerial.print(pressure_sample);
      consoleSerial.print(F("\t"));
      consoleSerial.print(F("RiseRate: "));
      consoleSerial.println(rise_rate);
      //consoleSerial.print(F("\tSum: "));
      //consoleSerial.println(rise_rate_running_sum);
    #endif
  }
  
  // update pressure sample time so we can detect overflow on next loop iteration
  last_sample_millis = this_sample_millis;

  // send status message periodically via debug serial ///////////////////////////////////////////////////////////////////////////////////////////

  this_update_millis = (millis() % ((uint32_t)1000 * update_interval_console));
  if ( update_interval_console && (this_update_millis < last_update_millis)) {
    consoleSerial.print(millis() / (uint32_t)1000);
    consoleSerial.print(F(","));
    consoleSerial.print(active_state);
    consoleSerial.print(F(","));
    consoleSerial.print((!modem.hasRingAsserted())?"IDLE,":"RING,");
    /*int c = gps.time.minute();
    if (c < 10) {
      consoleSerial.print(F("0")); // leading zero
    }
    consoleSerial.print(c);
    consoleSerial.print(F(":"));
    c = gps.time.second();
    if (c < 10) {
      consoleSerial.print(F("0")); // leading zero
    }
    consoleSerial.print(c);
    consoleSerial.print(F(",")); */
    consoleSerial.print(current_pressure);
    consoleSerial.print(F(","));
    //consoleSerial.print(pressure_sample);
    //consoleSerial.print(F(","));
    consoleSerial.print(rise_rate);
    consoleSerial.print(F(","));
    //consoleSerial.print(temperature_sample);
    //consoleSerial.print(F(","));
    consoleSerial.print(gps.location.lat(),6);
    consoleSerial.print(F(","));
    consoleSerial.print(gps.location.lng(),6);
    consoleSerial.print(F(","));
    //consoleSerial.print(gps.altitude.meters());
    //consoleSerial.print(F(","));
    //consoleSerial.print((float)gps.course.value()/100.0);
    //consoleSerial.print(F(","));
    //consoleSerial.print((float)gps.speed.value()/100.0);
    //consoleSerial.print(F(","));
    if (gps_isvalid) {
      consoleSerial.print(TinyGPSPlus::distanceBetween(gps.location.lat(),
          gps.location.lng(), launch_lat, launch_lon));
    //  consoleSerial.println(haversine(launch_lat, launch_lon, nmea.getLatitude(), nmea.getLongitude()));
    } else {
      consoleSerial.print(F("NaN"));  // no GPS lock, so no accurate distance available
    }
    consoleSerial.print(F(","));
    consoleSerial.print(gps.passedChecksum());
    consoleSerial.print(F(","));
    consoleSerial.println(analogRead(BATT_SENSE)/77.57575758); // (4*(analogRead(BATT_SENSE)/1024)*3.3); // 4:1 voltage divider, 3.3 analog reference
  }
  
  // update update time so we can detect overflow on next loop iteration
  last_update_millis = this_update_millis;

  switch (active_state) {
    case PRELAUNCH:  // short flash, 1Hz
      if ((config.letdown_delay >= 0) && (rise_rate > config.rise_rate_threshold)) {
        // detect launch due to pressure drop
        launch_time = (millis() - FILTER_DELAY);
        LED_period = 500;  // long slow blink
        LED_duration = 450;
        active_state = LETDOWN_INIT;
        #ifdef DEBUG
          consoleSerial.print(F("LAUNCH DETECT AT: "));
          consoleSerial.println(millis()/(uint32_t)1000);
          consoleSerial.print(F("TIME ESTIMATE: "));
          consoleSerial.println(launch_time/(uint32_t)1000);
          consoleSerial.print(F("PRESS: "));
          consoleSerial.println(current_pressure);
          consoleSerial.print(F("RISE RATE: "));
          consoleSerial.println(rise_rate);
          consoleSerial.print(F("LATITUDE: "));
          consoleSerial.println(launch_lat,6);
          consoleSerial.print(F("LONGITUDE: "));
          consoleSerial.println(launch_lon,6);
        #endif
      } else if ((config.letdown_delay < 0) && ((millis()/1000L) > (-1*config.letdown_delay))) {
        launch_time = 0;  // for negative letdown_delay, we delay letdown from "launch" at zero millis()
        LED_period = 500;  // long slow blink
        LED_duration = 450;
        active_state = LETDOWN_INIT;
        #ifdef DEBUG
          consoleSerial.print(F("LAUNCH BY TIMER AT: "));
          consoleSerial.println(millis()/(uint32_t)1000);
          consoleSerial.print(F("LATITUDE: "));
          consoleSerial.println(launch_lat,6);
          consoleSerial.print(F("LONGITUDE: "));
          consoleSerial.println(launch_lon,6);
        #endif
      } else {
        // update launch location if we have a lock
        // updating many times while in prelaunch mode means we have many bites at the apple to get a good gps location
        // altitude will be the altitude at which the filter detects a launch (in the air) but not far off
        if (gps_isvalid) {
          launch_lat = gps.location.lat();  // millionths of degrees
          launch_lon = gps.location.lng();  // millionths of degrees
          launch_alt = gps.altitude.meters();  // altitude MSL in meters
        }
      }
      break;

    case LETDOWN_INIT:  // long flashes 2Hz
      if ( (int32_t)((millis() - launch_time)/1000L) > (int32_t)config.letdown_delay ) {  // if config.letdown_delay is negative, this will succeed immediately
        digitalWrite(CUTTER, LOW);
        digitalWrite(MOTOR, HIGH);
        LED_period = 100;
        LED_duration = 50;
        active_state = LETDOWN_ACTIVE;
        #ifdef DEBUG
          consoleSerial.print(F("MOTOR ON: "));
          consoleSerial.println(millis()/(uint32_t)1000);
        #endif
      }
      break;

    case LETDOWN_ACTIVE:  // letting down, 10Hz flashes
      if ( ((millis() - launch_time)/1000L) > ((abs(config.letdown_delay) + config.letdown_duration)) ) {
        digitalWrite(MOTOR, LOW);
        LED_period = 2000;
        LED_duration = 200;
        active_state = FLIGHT;
        #ifdef DEBUG
          consoleSerial.print(F("MOTOR OFF: "));
          consoleSerial.println(millis()/(uint32_t)1000);
        #endif
      }
      break;

    case FLIGHT:  // short flash, 0.5Hz
      // check time limit; set max_flight_duration to 0 to disable time initiated cut
      if ((config.max_flight_duration > 0) && ( ((millis() - launch_time)/1000L) > config.max_flight_duration)) {
        active_state = CUT_INIT;
        break;
      }
      // check pressure ceiling; set cut_pressure to 0 to disable pressure initiated cut
      // if ((cut_pressure > 0) && (current_pressure < (long)cut_pressure * 100)) {
      if ((config.cut_pressure) && (current_pressure < config.cut_pressure)) {
        active_state = CUT_INIT;
        break;
      }
      
      // check geofence here
      if (gps_isvalid) {

        // try to detect/correct launch w/o valid gps
        if ( launch_lat == 0 ) {
          launch_lat = gps.location.lat();  // millionths of degrees
          launch_lon = gps.location.lng();  // millionths of degrees
          launch_alt = gps.altitude.meters();  // altitude MSL in meters
        }
        // compute distance and compare to max distance downrange
        if ( config.max_distance && (TinyGPSPlus::distanceBetween(gps.location.lat(),
          gps.location.lng(), launch_lat, launch_lon) > config.max_distance)) {
          active_state = CUT_INIT;
          break;
        // check min/max lat/lon to see if limits breached
        }
        if ( config.max_latitude!=0 && ( ((uint32_t)(gps.location.lat()*(uint32_t)1000000)) > config.max_latitude)) {
          active_state = CUT_INIT;
          break; 
        }
        if ( config.min_latitude!=0 && ( ((uint32_t)(gps.location.lat()*(uint32_t)1000000)) < config.min_latitude)) {
          active_state = CUT_INIT;
          break; 
        }
        if ( config.max_longitude!=0 && ( ((uint32_t)(gps.location.lng()*(uint32_t)1000000)) > config.max_longitude)) {
          active_state = CUT_INIT;
          break; 
        }
        if ( config.max_longitude!=0 && ( ((uint32_t)(gps.location.lng()*(uint32_t)1000000)) < config.min_longitude)) {
          active_state = CUT_INIT;
          break; 
        }
      }
      break;

    case CUT_INIT:
      digitalWrite(MOTOR, LOW);
      digitalWrite(CUTTER, HIGH);
      cut_time = millis();
      LED_period = 200;
      LED_duration = 100;
      active_state = CUT_ACTIVE;
      #ifdef DEBUG
        consoleSerial.print(F("CUTDOWN ON: "));
        consoleSerial.println(millis()/(uint32_t)1000);
      #endif
      break;

    case CUT_ACTIVE:  // short flash, 5 Hz.
      if ( (millis() - cut_time) > (config.cut_duration)) {
        digitalWrite(CUTTER, LOW);
        LED_period = 4000;
        LED_duration = 200;
        active_state = POST_FLIGHT;
        #ifdef DEBUG
          consoleSerial.print(F("CUTDOWN OFF: "));
          consoleSerial.println(millis()/(uint32_t)1000);
        #endif
      }
      break;

    case POST_FLIGHT:  // short flash, 0.25Hz
      digitalWrite(CUTTER, LOW);
      digitalWrite(MOTOR, LOW);
      break;

    default:
      break;
  }
  
  #ifdef DEBUG_LOOP_INTERVAL
  if (millis()-loop_timer > 3) {
    consoleSerial.println(millis()-loop_timer);
  }
  #endif

  return (true);   // returning false will terminate the SDB messages transmission early
}

#ifdef DEBUG_SDB
//void ISBDConsoleCallback(IridiumSBD *device, char c)
//{
//  consoleSerial.write(c);
//}

void ISBDDiagsCallback(IridiumSBD *device, char c)
{
  consoleSerial.write(c);
}
#endif

// ISR for LED blinking, called once a millisecond
SIGNAL(TIMER0_COMPA_vect) {
  uint32_t currentMillis = millis();
  
  // LED update
  digitalWrite(LED_RED, (currentMillis % LED_period) < LED_duration);

  // GPS LED update; blink if valid, otherwise off
  digitalWrite(LED_GREEN, ( gps_isvalid && (currentMillis % GPS_LED_period) < GPS_LED_duration));
  
}


// My beautiful function that isn't needed because TinyGPS++ already has it
// distance between points on earth in meters
// lat/lon in millionths of degrees (divide by 1,000,000 for degrees)
/*double haversine(double lat1, double lon1, double lat2, double lon2) {
  const double earth_radius = 6371009;  // earth radius in meters
  //const double pi180 = 3.1415926535897932384626433832795/180.0/1000000.0;
  // PI/180/1000000   convert millionths of degrees to radians
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
*/
