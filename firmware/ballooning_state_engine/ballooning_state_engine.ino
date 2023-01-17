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


//////////////////////// VARIOUS DEBUG LEVELS ////////////////////

// debug setup() and state engine; pretty much necessary now
#define DEBUG

// even more debug, ususally stuff we'd see after launch (ie only during ground test of functions)
//#define DEBUG_MAX

// echo satellite messages to serial
#define DEBUG_SBD

// debug iridium sbd with lots of messages
//#define DEBUG_SBD_MAX

// debug sensor and filter
//#define DEBUG_SENSOR

// print out average loop time in ms, current loop time
//#define DEBUG_LOOP_INTERVAL

// uncomment to calculate filter coefficents and dump to serial
//#define DEBUG_FILTER


//////////////////// YOUR CHOICE OF PRESSURE SENSOR //////////////////////
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

/*
 * Initialize Constructor
 * Optional parameters:
 *  - EOC_PIN: End Of Conversion (defualt: -1)
 *  - RST_PIN: Reset (defualt: -1)
 *  - MIN_PSI: Minimum Pressure (default: 0 PSI)
 *  - MAX_PSI: Maximum Pressure (default: 25 PSI)
 */

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
#include <IridiumSBD.h>
#include <TinyGPSPlus.h> // NMEA parsing: http://arduiniana.org
#include <CRC16.h>  // functions to compute/check CRC
CRC16 crc16;

#include <EEPROM.h>
// EEPROM_BASE_ADDR lets you spread the configuration load around the EEPROM so you don't wear a hole in it
#define EEPROM_BASE_ADDR 40
#include <math.h>
#include "ballooning_state_engine.h"

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
#define BATT_VOLTAGE_THRESHOLD (8.0)
// voltage at which to disable motor
#define BATT_VOLTAGE_OVERCURRENT (6.0)

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

// lines on rockblock labelled as DCE/modem; TX on RB is wired to TX on 328P
#define satSerial Serial
#define RING_PIN (2)
#define SLEEP_PIN (3)  // sleep ping requires some cicuitry to tie it to ground to put the iridium modem asleep
SoftwareSerial gpsSerial (6,7); // RX, TX
SoftwareSerial consoleSerial (10,9); // RX, TX 
//IridiumSBD modem(satSerial, SLEEP_PIN, RING_PIN);  // Declare the IridiumSBD object
IridiumSBD modem(satSerial, -1, RING_PIN);  // Declare the IridiumSBD object, sleep pin didn't work as advertised :(
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

state active_state = SETUP;  // initial states
state cut_method = SETUP; 
satellite_state active_satellite_state = SEND_IDLE;

bool loop_enabled = false; // turn on/off SBD callback

uint16_t LED_period = 2000;  // interval to blink LED
uint16_t LED_duration = 50;  // duration of LED blink
const uint16_t GPS_LED_period = 2000;  // interval to blink GPS locked LED
const uint16_t GPS_LED_duration = 2100;  // duration to blink GPS locked LED

static uint32_t launch_time = 0;  // so we can time letdown and flight time.  static is not needed in global vars?
float launch_lat;
float launch_lon;
float launch_alt;
bool gps_isvalid;  // flag to indicate valid lock or not

// average of last N samples
uint32_t current_pressure;
// current single sample of pressure
uint32_t pressure_sample;
float temperature_sample;
int32_t rise_rate;
float batt_voltage;

uint8_t MT_buffer[60];  // buffer for mobile terminated messages (commands to rockblock)
size_t MT_buffer_size; // = sizeof(MT_buffer);
uint8_t MO_buffer[60];  // buffer for mobile originated messages (beacons, replies to commands)
size_t MO_buffer_size;
bool MO_buffer_ready = false;  // semaphone to indicate message ready to send
struct sat_message *beacon;  // pointer to beacon (we'll load the beacon structure in MO_buffer directly)

uint8_t status;  // return status of satellite commands
int16_t x;  // rando temp variable 
uint32_t timer;  // store millis() value for timing


// utility functions

// declare the reset function so we can restart program
void(* resetFunc) (void) = 0;

// print 0 as 00, 12 as 12, etc
void print_decimal(Stream &out, uint8_t c) {
  if (c < 10) { // add leading zero to decimal number < 10
    out.write('0');
  }
  out.print(c);
}

// print 255 as FF, 10 as 0A, etc
void print_hex_char(Stream &out, uint8_t c) {
  if ( c < 0x10 ) { // add leading zero to hex numbers < 0x10. 0x01, 0x02... 0x0F
       out.write('0');
    }
    out.print(c, HEX);
}

// print a whole buffer as ASCII hex
void print_hex_buffer(Stream &out, uint8_t* c, uint16_t count) {
  for(int i=0; i<count; i++) {
    print_hex_char(out, c[i]);
  }
}

void build_beacon() {
  // beacon was initialized to point at MO_buffer, outgoing satellite buffer
  //consoleSerial.println("building a beacon");
  beacon->satellites = (uint8_t)gps.satellites.value();   // was config.unit_id;
  beacon->hdop = (uint8_t)(gps.hdop.value()*10);  // hdop*10.  hdop is X.X
  beacon->state = active_state;
  beacon->second = gps.time.second();
  beacon->minute = gps.time.minute();
  beacon->hour = gps.time.hour();
  beacon->day = gps.date.day();
  beacon->month = gps.date.month();
  beacon->year = (uint8_t)(gps.date.year()-2000);
  beacon->latitude = (uint32_t)(gps.location.lat()*(uint32_t)1000000);
  beacon->longitude = (uint32_t)(gps.location.lng()*(uint32_t)1000000);
  beacon->altitude = (int32_t)gps.altitude.meters();
  beacon->course = (int16_t)(gps.course.value());  // 100ths of degree
  beacon->speed = (int16_t)(gps.speed.value()); // 100ths of knot
  beacon->pressure = current_pressure;
  beacon->temperature = (int16_t)(temperature_sample*10); //current_temp*10;
  beacon->humidity = (int16_t)rise_rate; //current_humidity*10;  // since we don't have humidity sensor use this field for rise rate!
  beacon->batt_voltage = analogRead(BATT_SENSE);  // return 10 bit representation of batt voltage
  MO_buffer_size = sizeof(sat_message);
  MO_buffer_ready = true;  // queue for transmission
  crc16.reset(); // reset the CRC-XMODEM calculator
  crc16.add((uint8_t *)MO_buffer, (uint16_t)(MO_buffer_size-2));  // add an array of values to the CRC
  uint16_t crc = crc16.getCRC();   // calculate CRC16-XMODEM
  beacon->crc16 = ((crc<<8) & 0xFF00) | (crc>>8) ;  // reverse bytes of CRC and add to beacon
  if (0) {
    crc16.reset();
    crc16.add((uint8_t *)MO_buffer, (uint16_t)(MO_buffer_size));  // check CRC is zero
    consoleSerial.print(F("CRC check: 0x"));
    consoleSerial.println(crc16.getCRC(), HEX);
  }
}

// uint8_t process_cmd(uint8_t buffer[], size_t buffer_size) {  // make this operate on global MO/MT buffers
// process_cmd returns 0 (no valid command), 1 (command executed), or 2 (exit serial command mode, END command was found)
uint8_t process_cmd() {

    #ifdef DEBUG_PROCESS_CMD
    print_hex_buffer(consoleSerial,MT_buffer,MT_buffer_size);
    consoleSerial.write(':');
    consoleSerial.println(MT_buffer_size);
    #endif

    // CUT command: CUTXX (5 bytes), XX is duration in milliseconds
    if ((MT_buffer_size == 5) && !strncmp(MT_buffer, "CUT", 3)) {
      config.cut_duration = (uint16_t)*(MT_buffer+3) + 0x100*(uint16_t)*(MT_buffer+4);
      MT_buffer_size = 3;
      // fall through to CUT command
    }
    
    // CUT command:  CUT (3 bytes)
    if ((MT_buffer_size == 3) && !strncmp(MT_buffer, "CUT", 3)) {
      #ifdef DEBUG_MAX
      consoleSerial.print(F("CMD CUT "));
      consoleSerial.println(config.cut_duration);
      #endif
      if (active_state == SETUP) {
        timer = millis();
        while ( (millis()-timer) < config.cut_duration) {
          digitalWrite(LED_RED, LOW);
          digitalWrite(LED_GREEN, LOW);
          digitalWrite(CUTTER, HIGH);
        }
        digitalWrite(CUTTER, LOW);
      } else {
        cut_method = POST_FLIGHT_SATELLITE,  // set final state after cut completed
        active_state = CUT_INIT;  // start the cut
      }
      return(1);  // good command status
    }

    // LET command: LETXX (5 bytes), XX is duration in seconds
    if ((MT_buffer_size == 5) && !strncmp(MT_buffer, "LET", 3)) {
      config.letdown_duration = (uint16_t)*(MT_buffer+3) + 0x100*(uint16_t)*(MT_buffer+4);
      MT_buffer_size = 3;
      // fall through to LET command
    }
    
   // LET command: LET (3 bytes)  activate motor for letdown_duration
    if ((MT_buffer_size == 3) && !strncmp(MT_buffer, "LET", 3)) {
      #ifdef DEBUG_MAX
      consoleSerial.print(F("LET "));
      consoleSerial.println(config.letdown_duration);
      #endif
      if (active_state == SETUP) {
        timer = millis();
        while ( (millis()-timer) < config.cut_duration) {
          digitalWrite(LED_RED, LOW);
          digitalWrite(LED_GREEN, LOW);
          digitalWrite(MOTOR, HIGH);
        }
        digitalWrite(MOTOR, LOW);
      } else {
        //launch_time = millis()-abs(config.letdown_delay)*1000;
        if((millis()/1000)>((uint16_t)abs(config.letdown_delay))) {  // launch can't be before 0
          launch_time = millis()-((uint16_t)abs(config.letdown_delay)*(uint16_t)1000);  // virtual launch occurred at letdown time-letdown delay
        } else {
          launch_time = 0;
        }
        active_state = LETDOWN_INIT;
      }
      
      return(1);
    }
    
    // PRG commmand:  store configuration structure to EEPROM.  38 is eeprom_config length, 3 for PRG
    if ((MT_buffer_size == (sizeof(eeprom_config)+3)) && !strncmp(MT_buffer, "PRG", 3)) {
      #ifdef DEBUG
      consoleSerial.print(F("PRG "));
      print_hex_buffer(consoleSerial, MT_buffer+3, MT_buffer_size-3);
      consoleSerial.println();
      #endif
      // check incoming config CRC16-XMODEM
      crc16.reset(); // reset the CRC16-XMODEM calculator
      //crc16.add((uint8_t *)MT_buffer+3, (uint16_t)(MT_buffer_size-3));  // add an array of values to the CRC
      crc16.add(MT_buffer+3, MT_buffer_size-3);  // add an array of values to the CRC
      if(crc16.getCRC()) {   // successful check will return 0
        #ifdef DEBUG
        consoleSerial.print(F("CRC check: 0x"));
        consoleSerial.println(crc16.getCRC(),HEX);
        #endif
        return(0);  // indicate failure
      }
      // write config to EEPROM
      memcpy((uint8_t *)&config, MT_buffer+3, sizeof(eeprom_config));  // copy MT_buffer to config in memory
      EEPROM.put(EEPROM_BASE_ADDR, config);  // write new config to EEPROM
      
      if (active_state != SETUP) {  // if not in setup mode, fall through to CFG cmd and send a CFG message via satellite
        MT_buffer_size = 3;
        memcpy(MT_buffer,"CFG",3);
      } else {
        return(1);
      }
      
    }

    // CFG command will re-read config from EEPROM, which will reset the value for update_interval_satellite
    //   also, will not recalculate the CRC for the EEPROM, so if the EEPROM is corrupt the CFG will not be delivered
    //   This should be a very, very rare happening as the CRC is checked when programming the EEPROM either through
    //   serial or over the satellite; a configuration with a bad CRC should never be programmed.
    if ((MT_buffer_size == 3) && !strncmp(MT_buffer, "CFG", 3)) {
      // send CFG+configuration string back
      EEPROM.get(EEPROM_BASE_ADDR, config);  // this will also reset update_interval_satellite! 
      memcpy(MO_buffer, MT_buffer, 3);  // copy CFG to MO_buffer   
      memcpy(MO_buffer+3, (uint8_t *)&config, sizeof(eeprom_config));  // add current config to MO_buffer
      MO_buffer_ready = true;  // flag MO buffer for transmission
      MO_buffer_size = 3+sizeof(eeprom_config);
      #ifdef DEBUG
      consoleSerial.print(F("CFG "));
      print_hex_buffer(consoleSerial, MO_buffer, 3+sizeof(eeprom_config));
      consoleSerial.print(F(" size 0x"));
      consoleSerial.print(3+sizeof(eeprom_config), HEX);
      consoleSerial.println();
      #endif
      return(1);
    }    

    // UPD command:  change satellite update interval to xx seconds
    if ((MT_buffer_size == 5) && !strncmp(MT_buffer, "UPD", 3)) {
      config.update_interval_satellite = (uint16_t)*(MT_buffer+3) + 0x100*(uint16_t)*(MT_buffer+4);
      #ifdef DEBUG_MAX
      consoleSerial.print(F("UPD "));
      consoleSerial.print(config.update_interval_satellite);
      consoleSerial.println();
      #endif
      return(1);
    }

    //PNG command:  return bytes 
    if (!strncmp(MT_buffer, "PNG", 3)) {
      //  send MT_buffer back
      memcpy(MO_buffer, MT_buffer[3], MT_buffer_size-3);
      MO_buffer_size = MT_buffer_size-3;
      MO_buffer_ready = true;  // queue message
      #ifdef DEBUG_MAX
      consoleSerial.print(F("PNG "));
      print_hex_buffer(consoleSerial,MT_buffer,MT_buffer_size);
      consoleSerial.println();
      #endif
      return(1);
    }

    if ((MT_buffer_size==3) && !strncmp(MT_buffer, "END", 3)) {
      return(2);   // exit serial command mode
    }

    #ifdef DEBUG_MAX
    consoleSerial.print(F("process_cmd(): err "));
    print_hex_buffer(consoleSerial, MT_buffer, MT_buffer_size);
    consoleSerial.print(F(" size "));
    consoleSerial.println(MT_buffer_size);
    #endif
    return(0);  // no valid command found
}

void error_flash(uint8_t flashes, uint8_t repeats, uint16_t duration) {
  for (uint8_t repeat = 0; repeat < repeats; repeat++) {
    for (uint8_t count = 0; count < flashes; count++) {
        digitalWrite(LED_RED, HIGH);  // blink LED to indicate problem
        delay(duration);
        digitalWrite(LED_RED, LOW);
        delay(100);
    }
    delay(300);  // extra space between repetition of code
  }
}

void sos_flash() {
  error_flash(3,1,100);
  error_flash(3,1,300);
  error_flash(3,1,100);
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
  //pinMode(SLEEP_PIN, OUTPUT);
  //digitalWrite(SLEEP_PIN, HIGH);

  analogReference(EXTERNAL); // use AREF for reference voltage
  pinMode(BATT_SENSE, INPUT);
  
  beacon = (sat_message *)MO_buffer;  // we build the beacon in the MO_buffer
  
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
  
  #ifdef DEBUG_MAX
    consoleSerial.println(F("\r\nStart setup"));
  #endif
  
  // turn off all LED, on since boot for 5 sec (lamp test)
  digitalWrite(LED_RED, LOW);  // turn on RED LED
  digitalWrite(LED_GREEN, LOW);   // turn on GREEN LED
  
  #ifdef DEBUG
  #define CUTTER_BATTERY_DROP_THRESHOLD (1.5)
  // changed MOTOR_BATTERY_DROP_THRESHOLD from 1.2 to 1.5 1/17/2023
  #define MOTOR_BATTERY_DROP_THRESHOLD (1.5)
  #define OPEN_CIRCUIT_THRESHOLD (0.1)
  batt_voltage = read_batt_voltage();
  consoleSerial.print(F("BATTERY "));
  consoleSerial.println(batt_voltage);
  if ( batt_voltage > BATT_VOLTAGE_THRESHOLD ) { 
    consoleSerial.print(F("CUT TEST: "));
    digitalWrite(CUTTER, HIGH);  // turn on cutter
    delay(100);
    batt_voltage = batt_voltage - read_batt_voltage();  // batt_voltage is now voltage drop due to battery internal resistance
    digitalWrite(CUTTER, LOW);  // turn off cutter
    if (batt_voltage > CUTTER_BATTERY_DROP_THRESHOLD) {  // test for cutter fault
      consoleSerial.println(F("OVERCURRENT"));
      sos_flash();
    } else if (batt_voltage < OPEN_CIRCUIT_THRESHOLD) {
      consoleSerial.println(F("OPEN CIRCUIT"));
      sos_flash();
    } else {
      consoleSerial.println(F("PASS"));
    }
    delay(100);  // let battery bounce back a little
    batt_voltage = read_batt_voltage();
    consoleSerial.print(F("MOTOR TEST: "));
    digitalWrite(MOTOR, HIGH);  // turn on motor
    delay(50);
    batt_voltage = batt_voltage - read_batt_voltage();
    digitalWrite(MOTOR, LOW);  // turn off motor
    if (batt_voltage > MOTOR_BATTERY_DROP_THRESHOLD) {  // test for motor fault
      consoleSerial.println(F("OVER CURRENT"));
      sos_flash();
    } else if (batt_voltage < OPEN_CIRCUIT_THRESHOLD) {
      consoleSerial.println(F("OPEN CIRCUIT"));
      sos_flash();
    } else {
      consoleSerial.println(F("PASS"));
    }
  } else {
    // battery too low
    sos_flash();
  }
  
  #endif

  #ifdef MPR
  digitalWrite(RESET_PIN, HIGH);  // take pressure sensor out of reset
  #endif
  
  // read parameters from EEPROM-- things we can adjust for flight

  //   force initial config by making unit_id 0xFFFF
  //EEPROM.put(EEPROM_BASE_ADDR, (uint16_t)0xFFFF);
  
  EEPROM.get(EEPROM_BASE_ADDR, config); 
   
  // check for uninitialized EEPROM
  if (config.unit_id == 0xFFFF) { // not initialized, so lets initialize it
    #ifdef DEBUG_MAX
    consoleSerial.println(F("EEPROM unititialized"));
    #endif
    //  default configuration // default configuration // default configuration // default configuration // default configuration //
    config.unit_id = 0;
    config.letdown_delay = -30;  // seconds; positive: delay after launch detect, negative: delay after power on
    config.cut_duration = 3000;  // milliseconds
    config.max_flight_duration = 0;
    config.cut_pressure = 0;
    config.letdown_duration = 2;  //seconds
    config.rise_rate_threshold = 85;
    config.update_interval_satellite = 60;
    config.max_distance = (uint32_t)0;
    config.min_latitude = (int32_t)0;
    config.max_latitude = (int32_t)0;
    config.min_longitude = (int32_t)0;
    config.max_longitude = (int32_t)0; 
    config.crc16 = (uint16_t)0xE4E2;  // calculate from above values https://crccalc.com/ CRC16-XMODEM, reverse LSB and MSB
    EEPROM.put(EEPROM_BASE_ADDR, config);  // write config to EEPROM
    EEPROM.get(EEPROM_BASE_ADDR, config);  // read back config to verify it took
  } 
  
  #ifdef DEBUG
  consoleSerial.print(F("EEPROM "));
  print_hex_buffer(consoleSerial,(uint8_t *)&config, sizeof(struct eeprom_config));
  consoleSerial.println();
  #endif
  
  // check EEPROM CRC-16
  crc16.reset();
  crc16.add((uint8_t *)&config, sizeof(config));
  if (crc16.getCRC()) {
    #ifdef DEBUG
    consoleSerial.println(F("EEPROM corrupt"));
    #endif
    sos_flash();
  }

  // check for commands on serial port;
  #ifdef DEBUG
  consoleSerial.println(F("Serial cmd mode"));
  #endif
  for(uint8_t i=0; i<10; i++) {
    digitalWrite(LED_RED, i%2);  // blink LEDs during programming phase
    digitalWrite(LED_GREEN, !(i%2));
    
    MT_buffer_size = 0;
    timer = millis();  // timer for end of message
    while( ((millis()-timer) < 1000L) ) {
      if ( (x = consoleSerial.read()) > -1) {
        //consoleSerial.print(F("$"));  // echo character for debug
        timer = millis();  // reset timer
        if (MT_buffer_size < sizeof(MT_buffer)) {
          MT_buffer[MT_buffer_size++] = (uint8_t)x;   // load each serial rx character into cmd buffer until timeout.  use sat buffer for this purpose
        }
      }
    }
    if (MT_buffer_size>0) {  // if some bytes were read
      if (0) {
        print_hex_buffer(consoleSerial,MT_buffer,MT_buffer_size);
        consoleSerial.println();
        delay(100);
      }
      status = process_cmd();  // interpret the command in MT_buffer
      if (status==0) {
        consoleSerial.println(F("ERR"));  // process command returned error status
      } else if (status==1) {
        consoleSerial.println(F("OK"));  // good command receive
      } else if (status==2) {
        consoleSerial.println(F("END"));  // end command received
        break;
      }
    }
  } 

  digitalWrite(LED_RED, LOW);  // make sure leds are off
  digitalWrite(LED_GREEN, LOW);

    // Begin satellite modem operation
  #ifdef DEBUG
  consoleSerial.println(F("Start RockBLOCK"));
  #endif
  loop_enabled = false;  // disable SBD callback during setup
  modem.setPowerProfile(IridiumSBD::DEFAULT_POWER_PROFILE);  // high power
  //modem.setPowerProfile(IridiumSBD::USB_POWER_PROFILE);  // for low power
  modem.adjustSendReceiveTimeout(90);  // what will this do? DWK
  status = modem.begin();
  if (status != ISBD_SUCCESS)
  {
    #ifdef DEBUG_MAX
    consoleSerial.print(F("RockBLOCK failed: err "));
    consoleSerial.println(status);
    #endif
    if (status == ISBD_NO_MODEM_DETECTED) {
      #ifdef DEBUG_MAX
      consoleSerial.println(F("Not detected: check wiring"));
      #endif
      error_flash(2,3,100);
      resetFunc();
    }
  }

  #ifdef DEBUG
  // Get the IMEI
  char IMEI[16];
  status = modem.getIMEI(IMEI, sizeof(IMEI));
  if (status != ISBD_SUCCESS)
  {
    #ifdef DEBUG_MAX
    consoleSerial.print(F("getIMEI failed: err "));
    consoleSerial.println(status);
    #endif
    error_flash(2,3,100);
    resetFunc();
    //return;
  }
  consoleSerial.print(F("IMEI "));
  consoleSerial.println(IMEI);
  #endif

  #ifdef DEBUG
     consoleSerial.print(F("CFG "));
     print_hex_buffer(consoleSerial, (uint8_t *) &config, sizeof(eeprom_config));
     consoleSerial.println();
  #endif
  
  gpsSerial.begin(9600);  // software serial to gps receiver
  gpsSerial.listen();

#ifdef MPL3115A2
  #ifdef DEBUG
  consoleSerial.println(F("Start MPL3115A2"));
  #endif
  if (!baro.begin()) {
    #ifdef DEBUG_MAX
    consoleSerial.println(F("Not detected, check wiring"));
    #endif
    sos_flash();
    resetFunc();  // go back to setup()
  }
  baro.setMode(MPL3115A2_BAROMETER);
#endif
  
  #ifdef DEBUG_SBD_MAX
  // Print the firmware revision
  char version[12];
  status = modem.getFirmwareVersion(version, sizeof(version));
  if (status != ISBD_SUCCESS)
  {
     consoleSerial.print(F("*FirmwareVersion failed: error "));
     consoleSerial.println(status);
     return;
  }
  consoleSerial.print(F("*Firmware Version is "));
  consoleSerial.println(version);

  // Check the signal quality.
  // This returns a number between 0 and 5.
  // 2 or better is preferred.
  int signalQuality;
  status = modem.getSignalQuality(signalQuality);
  if (status != ISBD_SUCCESS)
  {
    consoleSerial.print(F("*getSignalQuality failed: error "));
    consoleSerial.println(status);
    return;
  }

  consoleSerial.print(F("*Signal quality "));
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
    error_flash(3,3,100);
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
    consoleSerial.println(F("*Cannot connect to MPRLS sensor, resetting"));
    error_flash(3,3,100);
    resetFunc();  // go back to setup()
  }
#endif

  // pre-fill sample array with pressures
  #ifdef DEBUG_MAX
  consoleSerial.print(F("Fill digital filter"));
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
    #ifdef DEBUG_MAX
    consoleSerial.print(F("."));
    #endif
  }

  #ifdef DEBUG_MAX
  consoleSerial.println();  // newline after pressure initialization periods....
  #endif
  
  base_pressure /= N;  // base_pressure is average of N readings
  #ifdef DEBUG_MAX
    consoleSerial.print(F("base_pressure="));
    consoleSerial.println(base_pressure);
    consoleSerial.println(F("End setup"));
  #endif
  #ifdef DEBUG
    consoleSerial.println(F("time,state,ring,gps_time,pressure,rise_rate,lat,lon,distance,satellites,hdop,voltage"));
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
  loop_enabled = true;

  // setup timer for LED blinking
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);
}

void loop() {

  static uint32_t last_update_millis, this_update_millis;
  
  // send/receive sat messages here, which will repeatedly call ISBDCallback while executing
  // we also call ISBDCallback once a loop to refresh values when satellite is not communicating

  //  manually do callback at least once before sending a message; probe at least once per loop iteration. 
  ISBDCallback();  
  
  this_update_millis = (millis() % ((uint32_t)1000*(uint32_t)config.update_interval_satellite));

  // when it's update time or there is a message waiting for RX, build the TX status message : 
  // example message: *SBD TX: 000B21107916E8B8192A07531FA6E1B99101D7E100000 size 31

  // if there's not an incoming message (RING) and it's beacon time, queue a beacon packet
  if (!digitalRead(RING_PIN)) {
      #ifdef DEBUG_SBD
      consoleSerial.println(F("SBD RING in loop"));
      #endif
  } else {
    // if not already a MO_buffer queued, no incoming messages waiting, beacons enabled, and it's time for a beacon...
    if ( (!MO_buffer_ready) && (modem.getWaitingMessageCount()<=0) && (config.update_interval_satellite) && (this_update_millis < last_update_millis)) {
      build_beacon();  // load beacon into MO_buffer and set MO_buffer_ready
    }
  }
  
  // send MO_buffer if:  1) ring pin not active and waiting message count is 0, 2) MO_buffer_ready 
  // otherwise, send null if ring pin active or waiting message count is >0
  // if we send/receive, then call process_cmd to handle any incoming command
  if (  (!digitalRead(RING_PIN)) || (modem.getWaitingMessageCount() > 0) || MO_buffer_ready) {
    if (  (!digitalRead(RING_PIN)) || (modem.getWaitingMessageCount() > 0)) {
      if (modem.isConnected()) { // Check that the Qwiic Iridium is connected
        //modem.enableSuperCapCharger(true); // Enable the super capacitor charger
        //consoleSerial.println("super cap charger on");
        //while (!modem.checkSuperCapCharger()) ; // Wait for the capacitors to charge
        //consoleSerial.println("super cap charged");
        //modem.enable9603Npower(true); // Enable power for the 9603N
        //consoleSerial.println("modem turned on");
        //modem.begin(); // Wake up the modem
        MT_buffer_size = sizeof(MT_buffer); // always reset this before receive; message size is returned in this variable
        #ifdef DEBUG_SBD
        consoleSerial.println(F("SBD TX: NULL"));
        #endif
        active_satellite_state = UNINTERRUPTABLE;  // don't stop command retrieval
        status = modem.sendReceiveSBDBinary(MO_buffer, 0, MT_buffer, MT_buffer_size);  // TX null message, receive incoming message
        //active_satellite_state = SEND_IDLE;  // either message was received or ISBD callback cancelled, either way session is over
      }
    } else if (MO_buffer_ready) {
      if (modem.isConnected()) { // Check that the Qwiic Iridium is connected
        //modem.enableSuperCapCharger(true); // Enable the super capacitor charger
        //consoleSerial.println("super cap charger on");
        //while (!modem.checkSuperCapCharger()) ; // Wait for the capacitors to charge
        //consoleSerial.println("super cap charged");
        //modem.enable9603Npower(true); // Enable power for the 9603N
        //consoleSerial.println("modem turned on");
        //modem.begin(); // Wake up the modem
        MT_buffer_size = sizeof(MT_buffer); // always reset this before receive; message size is returned in this variable
        #ifdef DEBUG_SBD
        consoleSerial.print(F("SBD TX: "));
        print_hex_buffer(consoleSerial, MO_buffer, MO_buffer_size);
        consoleSerial.print(F(" size "));
        consoleSerial.println(MO_buffer_size);
        #endif
        active_satellite_state = INTERRUPTABLE;
        status = modem.sendReceiveSBDBinary(MO_buffer, MO_buffer_size, MT_buffer, MT_buffer_size); // TX/RX a message in binary
        //consoleSerial.print(F("SBD TX/RX status: "));
        //consoleSerial.println(status);
        //consoleSerial.print(F("active_satellite_state: "));
        //consoleSerial.println(active_satellite_state);
        if (active_satellite_state != SESSION_INTERRUPTED) {
          MO_buffer_ready = false;  // we tried to send it, dequeue message... but not if we've been interrupted by a new beacon
        } else {
          #ifdef DEBUG_SBD
          consoleSerial.println(F("SBD session interrupted"));
          #endif
        }
      }
    }

    if (active_satellite_state != SESSION_INTERRUPTED) {  // we sent a message or null, let's read the result
      #ifdef DEBUG_SBD
      if ((MT_buffer_size>0) && (status == ISBD_SUCCESS)) {  // successful session, command in MT_buffer
        consoleSerial.print(F("SBD RX (hex): "));
        print_hex_buffer(consoleSerial, MT_buffer, MT_buffer_size);
        consoleSerial.print(F(" size 0x"));
        consoleSerial.println(MT_buffer_size, HEX);
        consoleSerial.print(F("SBD RX (asc): "));
        for(int i=0; i<MT_buffer_size; i++) {
          if ( isprint(MT_buffer[i])) { // print RX message printable characters
            consoleSerial.write(MT_buffer[i]);
          } else {
            consoleSerial.write('.');
            // consoleSerial.write('(');
            // print_hex_char(consoleSerial,MT_buffer[i]);
            // consoleSerial.write(')');
          }
        }
        consoleSerial.print(F(" size "));
        consoleSerial.println(MT_buffer_size);
      }
      #endif
    }

    // at this point, the session is over and has either succeeded, failed, or has been interrupted.  
    active_satellite_state = SEND_IDLE;   // clear sending state
    #ifdef DEBUG_SBD
    consoleSerial.print(F("SBD TX/RX status: "));
    consoleSerial.println(status);
    #endif
    
    // process incoming message
    if ((MT_buffer_size>0) && (status == ISBD_SUCCESS)) {  // successful session, command in MT_buffer
       process_cmd();
    }
    
    // Clear the Mobile Originated message buffer to avoid re-sending the message during subsequent loops
    #ifdef DEBUG_SBD_MAX
    consoleSerial.println(F("SBD Clearing the MO buffer"));
    #endif
    status = modem.clearBuffers(ISBD_CLEAR_MO); // Clear MO buffer
    #ifdef DEBUG_SBD_MAX
    if (status != ISBD_SUCCESS) {
      consoleSerial.print(F("SBD clearBuffers failed: error "));
      consoleSerial.println(status);
    }
    #endif
    
    //modem.sleep(); // Put the modem to sleep
    //modem.enable9603Npower(false); // Disable power for the 9603N
    //modem.enableSuperCapCharger(false); // Disable the super capacitor charger
    //modem.enable841lowPower(true); // Enable the ATtiny841's low power mode (optional)
    
    #ifdef DEBUG_SBD
    consoleSerial.println(F("SBD TX/RX finished"));
    #endif
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

  if (!loop_enabled) {  // this is here to disable the loop while talking to satellite in setup()
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

  #ifdef DEBUG
  // send status message periodically via debug serial ///////////////////////////////////////////////////////////////////////////////////////////

  this_update_millis = (millis() % ((uint32_t)1000 * update_interval_console));
  if ( update_interval_console && (this_update_millis < last_update_millis)) {
    consoleSerial.print(millis() / (uint32_t)1000);
    consoleSerial.print(F(","));
    consoleSerial.print(active_state);
    consoleSerial.print(F(","));
    consoleSerial.print((!modem.hasRingAsserted())?"IDLE,":"RING,");
    consoleSerial.print(active_satellite_state);
    consoleSerial.print(F(","));
    print_decimal(consoleSerial, gps.time.hour());
    consoleSerial.print(F(":"));
    print_decimal(consoleSerial, gps.time.minute());
    consoleSerial.print(F(":"));
    print_decimal(consoleSerial, gps.time.second());
    consoleSerial.print(F(",")); 
    consoleSerial.print(current_pressure);
    consoleSerial.print(F(","));
    consoleSerial.print(rise_rate);
    consoleSerial.print(F(","));
    consoleSerial.print(gps.location.lat(),6);
    consoleSerial.print(F(","));
    consoleSerial.print(gps.location.lng(),6);
    consoleSerial.print(F(","));
    if (gps_isvalid) {
      consoleSerial.print(TinyGPSPlus::distanceBetween(gps.location.lat(),
          gps.location.lng(), launch_lat, launch_lon));
    //  consoleSerial.println(haversine(launch_lat, launch_lon, nmea.getLatitude(), nmea.getLongitude()));
    } else {
      consoleSerial.print(F("NaN"));  // no GPS lock, so no accurate distance available
    }
    consoleSerial.print(F(","));
    //consoleSerial.print(gps.passedChecksum());
    consoleSerial.print(gps.satellites.value());
    consoleSerial.print(F(","));
    consoleSerial.print(gps.hdop.value());
    consoleSerial.print(F(","));
    consoleSerial.println(read_batt_voltage()); // (4*(analogRead(BATT_SENSE)/1024)*3.3); // 4:1 voltage divider, 3.3 analog reference
  }
  
  // update update time so we can detect overflow on next loop iteration
  last_update_millis = this_update_millis;
  #endif // DEBUG_MAX
  
  switch (active_state) {
    case PRELAUNCH:  // short flash, 1Hz
      if ((config.letdown_delay >= 0) && (rise_rate > config.rise_rate_threshold)) {
        // detect launch due to pressure drop
        launch_time = (millis() - FILTER_DELAY);
        LED_period = 500;  // long slow blink
        LED_duration = 450;
        active_state = LETDOWN_INIT;
        #ifdef DEBUG_MAX
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
        build_beacon();  // queue a beacon for transport
        if (active_satellite_state == INTERRUPTABLE) {  // interrupt an ongoing sat session if it's a beacon
          active_satellite_state = SESSION_INTERRUPTED; // lets the other loop know we've interrupted them
          #ifdef DEBUG_SBD
          consoleSerial.println(F("int SBD to report launch"));
          #endif
          return(false);  // returning false will terminate the SBD messages transmission early
        }
      } else if ((config.letdown_delay < 0) && ((millis()/1000L) > (-1*config.letdown_delay))) {
        launch_time = 0;  // for negative letdown_delay, we delay letdown from "launch" at zero millis()
        LED_period = 500;  // long slow blink
        LED_duration = 450;
        active_state = LETDOWN_INIT;
        #ifdef DEBUG_MAX
          consoleSerial.print(F("LAUNCH BY TIMER AT: "));
          consoleSerial.println(millis()/(uint32_t)1000);
          consoleSerial.print(F("LATITUDE: "));
          consoleSerial.println(launch_lat,6);
          consoleSerial.print(F("LONGITUDE: "));
          consoleSerial.println(launch_lon,6);
        #endif
        build_beacon();  // queue a beacon for transport
        if (active_satellite_state == INTERRUPTABLE) {  // interrupt an ongoing sat session if it's a beacon
          active_satellite_state = SESSION_INTERRUPTED;  // lets the other loop know we've interrupted them
          #ifdef DEBUG_SBD
          consoleSerial.println(F("int SBD to report launch"));
          #endif
          return(false);  // returning false will terminate the SBD messages transmission early
        }
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
        #ifdef DEBUG_MAX
          consoleSerial.print(F("MOTOR ON: "));
          consoleSerial.println(millis());
        #endif
      }
      break;

    case LETDOWN_ACTIVE:  // letting down, 10Hz flashes
      // check for overcurrent condition while motor on (jammed motor)
      batt_voltage = read_batt_voltage();  // check battery voltage 
      if ( ( batt_voltage < BATT_VOLTAGE_OVERCURRENT ) || (((millis() - launch_time)/1000L) > ((abs(config.letdown_delay) + config.letdown_duration))) ) {
        LED_period = 2000;
        LED_duration = 200;
        active_state = FLIGHT;
        if ( batt_voltage < BATT_VOLTAGE_OVERCURRENT ) {
          build_beacon();  // signal that letdown failed by low battery voltage
          #ifdef DEBUG_MAX
          consoleSerial.print(F("MOTOR STALLED: "));
          consoleSerial.println(millis());
          #endif
        } else {
          #ifdef DEBUG_MAX
          consoleSerial.print(F("MOTOR OFF: "));
          consoleSerial.println(millis());
          #endif
        }
        digitalWrite(MOTOR, LOW);
      }
      break;

    case FLIGHT:  // short flash, 0.5Hz
      // check time limit; set max_flight_duration to 0 to disable time initiated cut
      if ((config.max_flight_duration > 0) && ( ( (millis() - (uint32_t)launch_time)/(uint32_t)1000) > config.max_flight_duration)) {
        #ifdef DEBUG_MAX
        consoleSerial.println(F("TIME CUT"));
        #endif
        cut_method = POST_FLIGHT_MAX_TIME;
        active_state = CUT_INIT;
        break;
      }
      // check pressure ceiling; set cut_pressure to 0 to disable pressure initiated cut
      // if ((cut_pressure > 0) && (current_pressure < (long)cut_pressure * 100)) {
      if ((config.cut_pressure) && (current_pressure < config.cut_pressure)) {
        #ifdef DEBUG_MAX
        consoleSerial.println(F("PRES CUT"));
        #endif
        cut_method = POST_FLIGHT_PRESSURE;
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
          #ifdef DEBUG_MAX
          consoleSerial.println(F("DIST CUT"));
          #endif
          cut_method = POST_FLIGHT_DISTANCE;
          active_state = CUT_INIT;
          break;
        // check min/max lat/lon to see if limits breached
        }
        if ( config.max_latitude!=0 && ( ((uint32_t)(gps.location.lat()*(uint32_t)1000000)) > config.max_latitude)) {
          #ifdef DEBUG_MAX
          consoleSerial.println(F("MAX LAT CUT"));
          #endif
          cut_method = POST_FLIGHT_GEOFENCE;
          active_state = CUT_INIT;
          break; 
        }
        if ( config.min_latitude!=0 && ( ((uint32_t)(gps.location.lat()*(uint32_t)1000000)) < config.min_latitude)) {
          #ifdef DEBUG_MAX
          consoleSerial.println(F("MIN LAT CUT"));
          #endif
          cut_method = POST_FLIGHT_GEOFENCE;
          active_state = CUT_INIT;
          break; 
        }
        if ( config.max_longitude!=0 && ( ((uint32_t)(gps.location.lng()*(uint32_t)1000000)) > config.max_longitude)) {
          #ifdef DEBUG_MAX
          consoleSerial.println(F("MAX LON CUT"));
          #endif
          cut_method = POST_FLIGHT_GEOFENCE;
          active_state = CUT_INIT;
          break; 
        }
        if ( config.max_longitude!=0 && ( ((uint32_t)(gps.location.lng()*(uint32_t)1000000)) < config.min_longitude)) {
          #ifdef DEBUG_MAX
          consoleSerial.println(F("MIN LAT CUT"));
          #endif
          cut_method = POST_FLIGHT_GEOFENCE;
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
      #ifdef DEBUG_MAX
        consoleSerial.print(F("CUT ON: "));
        consoleSerial.println(millis());
      #endif
      break;

    case CUT_ACTIVE:  // short flash, 5 Hz.
      if ( (millis() - cut_time) > (config.cut_duration)) {
        digitalWrite(CUTTER, LOW);
        LED_period = 4000;
        LED_duration = 200;
        active_state = cut_method;
        #ifdef DEBUG_MAX
          consoleSerial.print(F("CUT OFF: "));
          consoleSerial.println(millis());
        #endif
      }
      break;

    case POST_FLIGHT_SATELLITE:  // short flash, 0.25Hz
    case POST_FLIGHT_MAX_TIME:
    case POST_FLIGHT_PRESSURE:
    case POST_FLIGHT_DISTANCE:
    case POST_FLIGHT_GEOFENCE:
      digitalWrite(CUTTER, LOW);  // Justin Case
      digitalWrite(MOTOR, LOW);
      if (cut_method > SETUP) {   // check if we just cut
        cut_method = SETUP;  // only do this once
        build_beacon();  // queue a beacon for transport
        if (active_satellite_state == INTERRUPTABLE) {  // interrupt an ongoing sat session if it's a beacon
          active_satellite_state = SESSION_INTERRUPTED;  // lets other loop know we interrupted them
          #ifdef DEBUG_SBD
          consoleSerial.println(F("int SBD to report cut"));
          #endif
          return(false);  // returning false will terminate the SBD messages transmission early
        }
      }
      break;

    default:
      break;
  }
  
  #ifdef DEBUG_LOOP_INTERVAL
  if (millis()-loop_timer > 3) {
    consoleSerial.println(millis()-loop_timer);
  }
  #endif

  if( (!digitalRead(RING_PIN)) && (active_satellite_state == INTERRUPTABLE)) {
    #ifdef DEBUG_SBD
    consoleSerial.println(F("SBD RING in ISBDCallback"));
    #endif
    return(false);  // returning false will terminate the SBD messages transmission early
  } else {
    return(true);
  }
}

#ifdef DEBUG_SBD_MAX
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
