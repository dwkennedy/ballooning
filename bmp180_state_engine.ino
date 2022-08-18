/*
detect balloon launch by thresholding rise rate
Doug Kennedy
for BMP180 or similar sensor using adafruit BMP805 library
Output 13 (LED)
ON - rise detected, positive slope exceeds threshold
FLASH - sink detected, negative slope exceeds threshold
OFF - between rise/sink thresholds
*/

// parameters to adjust for flight 

// how many milliseconds before activation of the let down motor
#define LET_DOWN_DELAY (6000)

// how long to run the motor (ms)
#define LET_DOWN_DURATION (6000)

// when to terminate the flight (ms)
#define MAX_FLIGHT_DURATION (50000)

// pressure at which to cut, mbar
//#define CUT_PRESSURE (972.5)
#define CUT_PRESSURE (10.0)

// how long to cut (ms)
#define CUT_DURATION (6000)

#include <Adafruit_BMP085.h>

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4

Adafruit_BMP085 bmp;

// Motor port
#define MOTOR (2)

// hot-wire cutter port
#define CUTTER (3)

// length of filter (N must be odd)
#define N (129)

// filter delay in milliseconds
#define FILTER_DELAY (3000)

// sample period in msec (loop delay in excess of pressure reading delay, 32ms for MS5611, ~68ms for BMP180)
#define T (10)

// rise rate to trigger event.  mbar/second... i think. depends on sample rate
#define RISE_RATE_THRESHOLD (40)

// sink rate to trigger event
#define SINK_RATE_THRESHOLD (-40)

// interval in seconds to send position update
#define UPDATE_INTERVAL (120)

// integration period is N*(T+32) msec = 5418 msec

// vector of filter coefficients
static double filter[N];

// circular buffer of N samples
static double sample[N];
static double base_pressure;

// index of circular buffer, 0 to N-1
static unsigned int n = 0;  

// state engine labels
enum state
{ 
    PRELAUNCH, 
    LETDOWN_DELAY,
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

void setup() {
  pinMode(MOTOR, OUTPUT);  // motor control port
  digitalWrite(MOTOR, LOW);  // motor off
  pinMode(CUTTER, OUTPUT);  // cutter control port
  digitalWrite(CUTTER, LOW);  // cutter off

  // set up output indicator LED
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // blip blip on boot
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);  
  delay(50);
  digitalWrite(LED_BUILTIN, HIGH);  // blip #2
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);  

  // calculate filter coefficients
  //  (could be done statically if N is fixed)

  for (int i=0; i<N; i++) {
    //filter[i] = (double)( -(((double)(i-((N-1)/2))/(double)N)) );
    //filter[i] = 1;
    filter[i] = -(12.0*(double)i-6.0*((double)N-1.0)) / ((double)N*((double)N*(double)N-1.0));
    //Serial.println(1000*filter[i]);
    filter[i] = 10000*filter[i];  // scale filter coefficients, delta Pressure / sample period
  }
      
  Serial.begin(115200);
  while(!bmp.begin()) {
    Serial.println("Could not find a valid BMP085/180 sensor, check wiring");
    for(int i=0; i<4; i++) {
      digitalWrite(LED_BUILTIN, HIGH);  // blink LED 4 times to indicate sensor problem
      delay(50);
      digitalWrite(LED_BUILTIN, LOW);
      delay(50);
    }
    delay(200);  // give an extra time between the 4 pulses
  }

 Serial.println("***");
  
   // print out a bunch of zeros for the sake of simple arduino serial plotter
  //for (int i=0; i<500; i++) {
  //  Serial.println("0");
 // }

  base_pressure = 0;  // calculate base pressure for plotting later, average of several readings
  for (int i=0; i<N*2; i++) {
    // WHY DO I DO THIS WEIRD THING ABOUT SAMPLE[I/2]????? maybe to get more samples for base pressure?
    sample[i/2] = bmp.readPressure()/(double)100.0;  // read pressure in Pa, convert to mbar because we like mbar
    base_pressure += sample[i/2];  // base_pressure is only used for arduino simple serial plotter
    //sample[i] = 0;
    //Serial.println(sample[i/2]); // pressure in mbar
    delay(T);
  }
  base_pressure /= N*2;
  
  //Serial.println("***");
  n = 0;  // oldest sample, first to be replaced in buffer; n is the index into the circular buffer of pressures

  // set initial state of controller
  active_state = PRELAUNCH;
  // LED blinky parameters
  LED_period = 1000;  
  LED_duration = 50;
  
}

void loop() {
  double rise_rate;
  double current_pressure;
  double pressure_sample;
  static unsigned long launch_time;
  static unsigned long cut_time;

  // read GPS string/update GPS structure here
  // read command from satellite radio
  //    if cut command received, active_state = CUT_INIT
  //    if PING command received, return PONG response

  if (millis() % (1000*UPDATE_INTERVAL)) {
    // send position/alt/pressure/temp etc update here
    // need flag so message is send once per interval
  }
  
  // read new pressure into circular buffer position n
  // increment n
  // set rise_rate to 0
  // starting at n (oldest sample), continuing to n+(N-1) mod N (aka n-1)
  //    calculate filter*sample, add to rise_rate (FIR filter)
  //  scale rise_rate to mbar/sec??
  // that's the rise/fall rate. rise is positive, fall negative


  pressure_sample = bmp.readPressure()/(double)100.0;
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

  current_pressure /= N;
  
  if (1) {
    Serial.print("T: ");
    Serial.print(millis());
    Serial.print("\t");
    Serial.print("PR: ");
    Serial.print(current_pressure);
    Serial.print("\t");
    Serial.print("PS: ");
    Serial.print(pressure_sample);
    Serial.print("\t");
    Serial.print("RR: ");
    Serial.println(rise_rate);
  }
  
  //rise_rate *= 1000;

  // LED update
  digitalWrite(LED_BUILTIN, (millis() % LED_period) < LED_duration);
  
  switch (active_state) {
    case PRELAUNCH:  // short flash, 1Hz
       if (rise_rate > (double)RISE_RATE_THRESHOLD) {
         launch_time = (millis()-(unsigned long)FILTER_DELAY);
         LED_period = 500;  // long slow blink
         LED_duration = 400;
         active_state = LETDOWN_DELAY;
         Serial.print("LAUNCH ESTIMATE: ");
         Serial.println(launch_time);
         Serial.print("LAUNCH DETECT: ");
         Serial.println(millis());
       }
       break;
       
    case LETDOWN_DELAY:  // long flashes 2Hz
       if ( ((millis()-(unsigned long)launch_time)) > (unsigned long)LET_DOWN_DELAY) {
          digitalWrite(CUTTER, LOW);
          digitalWrite(MOTOR, HIGH);
          LED_period = 100;
          LED_duration = 50; 
          active_state = LETDOWN_ACTIVE;
          Serial.print("LETDOWN ON: ");
          Serial.println(millis());
       }
       break;
       
    case LETDOWN_ACTIVE:  // letting down, 10Hz flashes
       if ( (millis()-(unsigned long)launch_time) > ((unsigned long)LET_DOWN_DELAY + (unsigned long)LET_DOWN_DURATION) ) {
          digitalWrite(MOTOR, LOW);
          LED_period = 2000;
          LED_duration = 50;
          active_state = FLIGHT;
          Serial.print("LETDOWN STOPPED: ");
          Serial.println(millis());
       }
       break;
       
    case FLIGHT:  // short flash, 0.5Hz
       // check time limit
       if ( (millis()-(unsigned long)launch_time) > (unsigned long)MAX_FLIGHT_DURATION ) {
          active_state = CUT_INIT;
          break;
       }
       // check pressure ceiling
       if (current_pressure < (double)CUT_PRESSURE) {
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
       Serial.print("CUTDOWN ON: ");
       Serial.println(millis()-(unsigned long)launch_time);
       break;
       
    case CUT_ACTIVE:  // short flash, 5 Hz.
       if( (millis()-(unsigned long)cut_time) > (unsigned long)CUT_DURATION) {
          digitalWrite(CUTTER, LOW);
          LED_period = 4000;
          LED_duration = 100;
          active_state = POST_FLIGHT;
          Serial.print("CUTDOWN OFF: ");
          Serial.println(millis()-(unsigned long)launch_time);
       }
       break;
       
    case POST_FLIGHT:  // short flash, 0.25Hz
       digitalWrite(CUTTER, LOW);
       digitalWrite(MOTOR, LOW);
       break;

    default:
       break;
  }

  if (0) { 
    Serial.println("");
  }
  
  delay(T);
}
