/*
detect balloon launch by thresholding rise rate
Doug Kennedy
for BMP180 or similar sensor using adafruit BMP805 library
Output 13 (LED)
ON - rise detected, positive slope exceeds threshold
FLASH - sink detected, negative slope exceeds threshold
OFF - between rise/sink thresholds
*/

//  https://github.com/SlashDevin/NeoSWSerial
//  https://github.com/naszly/Arduino-StaticSerialCommands
//  https://github.com/stevemarple/MicroNMEA
//  http://www.bamfordresearch.com/files/package_jb23_insystemmcu_index.json

// if using BMP180 sensor, define this
#define BMP180

#ifdef BMP180
#include <Adafruit_BMP085.h>
// Connect VCC of the BMP180 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
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

// Motor port
#define MOTOR (2)

// hot-wire cutter port
#define CUTTER (3)

// length of filter (N must be odd)
#define N (63)

// filter delay in milliseconds, approx
#define FILTER_DELAY (2500)

// sample period in msec (loop delay in excess of pressure reading delay, 32ms for MS5611, ~68ms for BMP180)
// integration period is N*(T+pressure reading delay) msec.  63*(10+68) = 4914 ms
#define T (10)

// sink rate to trigger event - not presently used; could be used to detect balloon pop
#define SINK_RATE_THRESHOLD (-500)

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

// rise rate to trigger event. should be higher than noise but less than balloon rise rate. units: Pa/second? i think. depends on sample rate and filter
// elevator in NWC peaks out around 1000.  noise level is +/- 150
static unsigned int rise_rate_threshold;

// how often to send GPS packet
static unsigned int update_interval;

NeoSWSerial gpsSerial (10, 9);
// SoftwareSerial gpsSerial (10, 9);
#ifdef BMP180
  Adafruit_BMP085 bmp;
#endif

// vector of filter coefficients
static long filter[N];

// circular buffer of N samples
static long sample[N];
static long base_pressure;

// index of circular buffer, 0 to N-1
static unsigned int n = 0;  

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

Command subCommands [] {
  COMMAND(cmd_set_unit_id, "unit_id", ArgType::Int, nullptr, "unique address"),
  COMMAND(cmd_set_letdown_delay, "letdown_delay", ArgType::Int, nullptr, "activation after launch, secs"),
  COMMAND(cmd_set_letdown_duration, "letdown_duration", ArgType::Int, nullptr, "time to lower, secs"),
  COMMAND(cmd_set_max_flight_duration, "max_flight_duration", ArgType::Int, nullptr, "elapsed time to terminate flight, secs"),
  COMMAND(cmd_set_cut_pressure, "cut_pressure", ArgType::Int, nullptr, "pressure to terminate flight, mbar"),
  COMMAND(cmd_set_cut_duration, "cut_duration", ArgType::Int, nullptr, "how long to activate cutter, secs"),
  COMMAND(cmd_set_rise_rate_threshold, "rise_rate_threshold", ArgType::Int, nullptr, "let-down trigger sensitivity (~500-1000)"),
  COMMAND(cmd_set_update_interval, "update_interval", ArgType::Int, nullptr, "how often to send beacon, secs"),
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
  sender.getSerial().print(F("active_state: "));
  sender.getSerial().println(active_state);
  
}

void cmd_set(SerialCommands& sender, Args& args) {
  sender.listAllCommands(subCommands, sizeof(subCommands) / sizeof(Command));
}

void cmd_set_unit_id(SerialCommands& sender, Args& args) {
  auto number = args[0].getInt();
  EEPROM.put(0*sizeof(unsigned int),(unsigned int)number);  // save serial number
  unit_id = number;  // set serial in ram as well
  sender.getSerial().print(F("unit_id="));
  sender.getSerial().println(number);
}

void cmd_set_letdown_delay(SerialCommands& sender, Args& args) {
  auto number = args[0].getInt();
  EEPROM.put(1*sizeof(unsigned int),(unsigned int)number);  // save number
  letdown_delay = number;  // set in ram as well
  sender.getSerial().print(F("letdown_delay="));
  sender.getSerial().println(number);
}

void cmd_set_letdown_duration(SerialCommands& sender, Args& args) {
  auto number = args[0].getInt();
  EEPROM.put(2*sizeof(unsigned int),(unsigned int)number);  // save number
  letdown_duration = number;  // set in ram as well
  sender.getSerial().print(F("letdown_duration="));
  sender.getSerial().println(number);
}

void cmd_set_max_flight_duration(SerialCommands& sender, Args& args) {
  auto number = args[0].getInt();
  EEPROM.put(3*sizeof(unsigned int),(unsigned int)number);  // save number
  max_flight_duration = number;  // set in ram as well
  sender.getSerial().print(F("max_flight_duration="));
  sender.getSerial().println(number);
}

void cmd_set_cut_pressure(SerialCommands& sender, Args& args) {
  auto number = args[0].getInt();
  EEPROM.put(4*sizeof(unsigned int),(unsigned int)number);  // save number
  cut_pressure = number;  // set in ram as well
  sender.getSerial().print(F("cut_pressure="));
  sender.getSerial().println(number);
}

void cmd_set_cut_duration(SerialCommands& sender, Args& args) {
  auto number = args[0].getInt();
  EEPROM.put(5*sizeof(unsigned int),(unsigned int)number);  // save number
  cut_duration = number;  // set in ram as well
  sender.getSerial().print(F("cut_duration="));
  sender.getSerial().println(number);
}

void cmd_set_rise_rate_threshold(SerialCommands& sender, Args& args) {
  auto number = args[0].getInt();
  EEPROM.put(6*sizeof(unsigned int),(unsigned int)number);  // save number
  rise_rate_threshold = number;  // set in ram as well
  sender.getSerial().print(F("rise_rate_threshold="));
  sender.getSerial().println(number);
}

void cmd_set_update_interval(SerialCommands& sender, Args& args) {
  auto number = args[0].getInt();
  EEPROM.put(7*sizeof(unsigned int),(unsigned int)number);  // save serial number
  update_interval = number;  // set in ram as well
  sender.getSerial().print(F("update_interval="));
  sender.getSerial().println(number);
}

double haversine(double lat1, double lon1, double lat2, double lon2) {
  const double earth_radius = 6371.009;  // earth radius in km
  //const double pi180 = 3.1415926535897932384626433832795/180.0/1000000.0;
  const double pi180 = 0.000000017453292519943295769236907684886127134428718885417254560971;

  // convert millionths of degrees to radians
  double dlat = (double)(lat2-lat1) * pi180;
  double dlon = (double)(lon2-lon1) * pi180;
  lat1 = lat1 * pi180;
  lat2 = lat2 * pi180;

  Serial.print("lat1: ");
  Serial.println(lat1,10);
  Serial.print("lat2: ");
  Serial.println(lat2,10);
  Serial.print("dlat: ");
  Serial.println(dlat,10);
  Serial.print("dlon: ");
  Serial.println(dlon,10);
  Serial.print("pow: "),
  Serial.println(pow(sin(dlat/2),2)+pow(sin(dlon/2),2)*cos(lat1)*cos(lat2), 10);
         
  double dist = 2*earth_radius*asin(
    sqrt( pow(sin( (dlat/2)),2)
         +pow(sin( (dlon/2)),2)
         *cos(lat1)*cos(lat2)));

  Serial.print("dst: ");
  Serial.println(dist,10);

  return(dist);  // great circle distance in km
}

char buffer[80];  // a buffer big enough to hold serial commands or reminder prompts
SerialCommands serialCommands(Serial, commands, sizeof(commands) / sizeof(Command),
                              buffer, sizeof(buffer));

char nmeaBuffer[85];  // a buffer big enough to hold largest expected NMEA sentence
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

void setup() {
  // configure control ports and make sure they're off
  pinMode(MOTOR, OUTPUT);  // motor control port
  digitalWrite(MOTOR, LOW);  // motor off
  pinMode(CUTTER, OUTPUT);  // cutter control port
  digitalWrite(CUTTER, LOW);  // cutter off

  delay(150);  // avoid double setup() when programming
  
  // set up output indicator LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // blip blip on boot
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);  
  delay(50);
  digitalWrite(LED_BUILTIN, HIGH);  // blip #2
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);  

  // set serial port baud rates
  Serial.begin(9600);  // hardware serial is connected to sat radio and/or programming FTDI cable
  gpsSerial.begin(9600);

  // calculate filter coefficients
  //  (could be done statically if N is fixed)
  for (int i=0; i<N; i++) {
    //filter[i] = (double)( -(((double)(i-((N-1)/2))/(double)N)) );
    //filter[i] = 1;
    // filter coeffs with double -- works
    // filter[i] = -(12.0 * (double)i-6.0 * ((double)N-1.0)) / ((double)N * ((double)N*(double)N - 1.0)); // - (12*i - 6*(N-1))   /   N*(N^2-1)
    // filter[i] = 10000*filter[i];  // scale filter coefficients, delta Pressure / sample period
    // try filter coeefs with longs, 1000000 is in there to ensure the coefficients don't get wiped out due to lack of precision
    filter[i] = (-1000000*(12.0 * (long)i-6.0 * ((long)N-1.0))) / ((long)N * ((long)N*(long)N - 1.0)); // - (12*i - 6*(N-1))   /   N*(N^2-1)
    if (DEBUG_SENSOR) {
      Serial.print(i);
      Serial.print(F(": "));
      Serial.println(filter[i]);
    }
  }

  // read parameters from EEPROM
  // parameters to adjust for flight 

  // unit address/ serial number
  EEPROM.get(0*sizeof(unsigned int),unit_id);

  // check for uninitialized EEPROM
  if (unit_id==65535) {  // not initialized, so lets initialize it
    EEPROM.put(1*sizeof(unsigned int), (unsigned int)30);  // letdown_delay
    EEPROM.put(2*sizeof(unsigned int), (unsigned int)30);  // letdown_duration
    EEPROM.put(3*sizeof(unsigned int), (unsigned int)0);   // max_flight_duration
    EEPROM.put(4*sizeof(unsigned int), (unsigned int)0);   // cut_pressure
    EEPROM.put(5*sizeof(unsigned int), (unsigned int)30);   // cut_duration
    EEPROM.put(6*sizeof(unsigned int), (unsigned int)800);  // rise_rate_threshold
    EEPROM.put(7*sizeof(unsigned int), (unsigned int)120);  // update_interval
    EEPROM.put(0*sizeof(unsigned int), (unsigned int)0);    // unit_id 0=UNSET
    unit_id = 0;
  }

  // how many seconds before activation of the let down motor
  EEPROM.get(1*sizeof(unsigned int),letdown_delay);

  // how long to run the motor (seconds)
  EEPROM.get(2*sizeof(unsigned int),letdown_duration);

  // when to terminate the flight (seconds)
  EEPROM.get(3*sizeof(unsigned int),max_flight_duration);

  // pressure at which to cut, mbar
  EEPROM.get(4*sizeof(unsigned int),cut_pressure);

  // how long to cut (seconds)
  EEPROM.get(5*sizeof(unsigned int),cut_duration);

  // rise rate to trigger event. 
  EEPROM.get(6*sizeof(unsigned int),rise_rate_threshold);

  // update interval
  EEPROM.get(7*sizeof(unsigned int),update_interval);
  
#ifdef BMP180  
  while(!bmp.begin()) {
    Serial.println("Could not find a valid BMP085/180 sensor, check wiring");
    for(int i=0; i<4; i++) {
      digitalWrite(LED_BUILTIN, HIGH);  // blink LED 4 times to indicate sensor problem
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
    }
    delay(400);  // give an extra time between the 4 pulses
  }
#endif

 
 if (DEBUG) {
  serialCommands.getSerial().println(F("Starting"));
 }
  
 if (DEBUG_PLOT) {  // print out a bunch of zeros for the sake of simple arduino serial plotter
   for (int i=0; i<500; i++) {
     Serial.println("0");
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
  for (int i=0; i<N; i++) {
    sample[i] = (long)bmp.readPressure();  // read pressure in Pa, mbar * 100
    base_pressure += sample[i];  // base_pressure is only used for arduino simple serial plotter
  }
  base_pressure /= N;  // base_pressure is average of N readings

  if (DEBUG) {
    Serial.println("***");
  }
  
  n = 0;  // oldest sample, first to be replaced in buffer; n is the index into the circular buffer of pressures

  // set initial state of controller
  active_state = PRELAUNCH;
  // LED blinky parameters for initial state
  LED_period = 1000;  
  LED_duration = 50;

  if (1) {
    const double lat1 = 51500700;
    const double lon1 =  0124600;
    const double lat2 = 40689200;
    const double lon2 = 74044500;
    Serial.print(F("haversine test (5574.84): "));
    Serial.println(haversine(lat1,lon1,lat2,lon2));
  }
  
}

void loop() {
  long rise_rate;
  long current_pressure;
  long pressure_sample;
  static unsigned long cut_time;
  static unsigned long last_update; 

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
  
  // read new pressure into circular buffer position n
  // increment n
  // set rise_rate to 0
  // starting at n (oldest sample), continuing to n+(N-1) mod N (aka n-1)
  //    calculate filter*sample, add to rise_rate (FIR filter)
  //  scale rise_rate to mbar/sec??
  // that's the rise/fall rate. rise is positive, fall negative

#ifdef BMP180
  pressure_sample = (long)bmp.readPressure();  // pressure in Pa (~97000 Pa in Norman, OK)
#else
  pressure_sample = (long)97400;
#endif
  sample[n] = pressure_sample;
  
  //Serial.print((sample[n]-base_pressure)*25);  // scaled and shifted for serial plotter
  //Serial.println(sample[n]);

  // increment the circular buffer index 
  n++;
  if (n >= N) {
    n = 0;
  }

  // apply linear regression filter to calculate rise rate
  rise_rate = 0;
  current_pressure = 0;
  for(int i=0; i<N; i++) {
    rise_rate += (filter[i] * sample[ (n+i) % N ]);
    current_pressure += sample[i];
    if (0) {  // debug code
      Serial.print("filter[");
      Serial.print(i);
      Serial.print("]: (");
      Serial.print(filter[i]);
      Serial.print(") * ");
      Serial.print("sample[");
      Serial.print( (n+i)%N );
      Serial.print("]: (");
      Serial.print(sample[(n+i) % N]);
      Serial.print(") = ");
      Serial.println(filter[i] * sample[ (n+i) % N]);
    }
  } 

  current_pressure /= N;  // moving average of pressure
  rise_rate /= 1000;  // rise_rate scaled to make up for integer filter coefficients
  
  if (DEBUG_SENSOR) {
    Serial.print(F("Time: "));
    Serial.print(millis());
    Serial.print(F("\t"));
    Serial.print(F("AvePres: "));
    Serial.print(current_pressure);
    Serial.print(F("\t"));
    Serial.print(F("PressSamp: "));
    Serial.print(pressure_sample);
    Serial.print(F("\t"));
    Serial.print(F("RiseRate: "));
    Serial.println(rise_rate);
  }

  // send status message periodically via HW serial / satellite
  if (((unsigned long)millis() % ((unsigned long)1000*update_interval)) < (unsigned long)last_update) {
    serialCommands.getSerial().print(millis()/1000);
    serialCommands.getSerial().print(F(","));
    serialCommands.getSerial().print(active_state);
    serialCommands.getSerial().print(F(","));
    int c = nmea.getMinute();
    if (c<10) {
      serialCommands.getSerial().print(F("0")); // leading zero
    }
    serialCommands.getSerial().print(c);
    serialCommands.getSerial().print(F(":"));
    c = nmea.getSecond();
    if (c<10) {
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
    serialCommands.getSerial().println(nmea.getLongitude());
  }
    
  // update update time so we can detect overflow on next loop iteration
  last_update = millis() % ((unsigned long)1000*update_interval);
  
  // LED update
  digitalWrite(LED_BUILTIN, (millis() % LED_period) < LED_duration);
  
  switch (active_state) {
    case PRELAUNCH:  // short flash, 1Hz
       if (rise_rate > (long)rise_rate_threshold) {
         launch_time = (millis()-(unsigned long)FILTER_DELAY);
         LED_period = 500;  // long slow blink
         LED_duration = 450;
         active_state = LETDOWN_INIT;
         if (DEBUG) {
           Serial.print(F("LAUNCH ESTIMATE: "));
           Serial.println(launch_time);
           Serial.print(F("LAUNCH DETECT: "));
           Serial.println(millis());
         }
       }
       break;
       
    case LETDOWN_INIT:  // long flashes 2Hz
       if ( ((millis()-(unsigned long)launch_time)) > (unsigned long)letdown_delay*1000) {
          digitalWrite(CUTTER, LOW);
          digitalWrite(MOTOR, HIGH);
          LED_period = 100;
          LED_duration = 50; 
          active_state = LETDOWN_ACTIVE;
          if (DEBUG) {
            Serial.print(F("LETDOWN ON: "));
            Serial.println(millis());
          }
       }
       break;
       
    case LETDOWN_ACTIVE:  // letting down, 10Hz flashes
       if ( (millis()-(unsigned long)launch_time) > ((unsigned long)letdown_delay*1000 + (unsigned long)letdown_duration*1000) ) {
          digitalWrite(MOTOR, LOW);
          LED_period = 2000;
          LED_duration = 50;
          active_state = FLIGHT;
          if (DEBUG) {
            Serial.print(F("LETDOWN STOPPED: "));
            Serial.println(millis());
          }
       }
       break;
       
    case FLIGHT:  // short flash, 0.5Hz
       // check time limit; set max_flight_duration to 0 to disable time initiated cut
       if ((max_flight_duration>0) && ( (millis()-(unsigned long)launch_time) > (unsigned long)max_flight_duration*1000 )) {
          active_state = CUT_INIT;
          break;
       }
       // check pressure ceiling; set cut_pressure to 0 to disable pressure initiated cut
       if ((cut_pressure>0) && (current_pressure < (long)cut_pressure*100)) {
          active_state = CUT_INIT;
          break;
       }
       // check geofence here
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
         Serial.println(millis()-(unsigned long)launch_time);
       }
       break;
       
    case CUT_ACTIVE:  // short flash, 5 Hz.
       if( (millis()-(unsigned long)cut_time) > (unsigned long)cut_duration*1000) {
          digitalWrite(CUTTER, LOW);
          LED_period = 4000;
          LED_duration = 100;
          active_state = POST_FLIGHT;
          if (DEBUG) {
            Serial.print(F("CUTDOWN OFF: "));
            Serial.println(millis()-(unsigned long)launch_time);
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

  if (DEBUG_SENSOR) { 
    Serial.println(F(""));
  }
  
  delay(T);
}
