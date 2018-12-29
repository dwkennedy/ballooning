/*

detect balloon launch by thresholding rise rate
Doug Kennedy

Requires MS5XXX library by Roman Schmitz

Output 13 (LED)
ON - rise detected, positive slope exceeds threshold
FLASH - sink detected, negative slope exceeds threshold
OFF - between rise/sink thresholds

*/

#include <Wire.h>
#include <MS5xxx.h>
#include <MS5611.h>

MS5611 sensor(&Wire);

// length of filter (N must be odd)
#define N (129)

// sample period in msec (delay in excess of loop delay, 32ms)
#define T (10)

// rise rate to trigger event
#define RISE_THRESHOLD (20)

// sink rate to trigger event
#define SINK_THRESHOLD (-15)

// integration period is N*(T+32) msec = 5418 msec

// vector of filter coefficients
static double filter[N];

// circular buffer of N samples
double sample[N];
double base_pressure;

// index of circular buffer, 0 to N-1
int n = 0;  

void setup() {
  // set up output indicator LED
  pinMode(13, OUTPUT);
  Serial.begin(9600);
  if(sensor.connect()>0) {
    Serial.println("Error connecting...");
    delay(500);
    setup();
  }


  // calculate filter coefficients
  //  (could be done statically if N is fixed)

  for (int i=0; i<N; i++) {
    //filter[i] = (double)( -(((double)(i-((N-1)/2))/(double)N)) );
    //filter[i] = 1;
    filter[i] = -(12.0*(double)i-6.0*((double)N-1.0)) / ((double)N*((double)N*(double)N-1.0));
    //Serial.println(1000*filter[i]);
    filter[i] = 10000*filter[i];  // scale filter coefficients, delta Pressure / sample period
  }
  Serial.println("***");
  
  // initialize sensor and buffer
  sensor.ReadProm();
  sensor.Readout();

  for (int i=0; i<500; i++) {
    Serial.println("0");
  }

  base_pressure = 0;  // calculate base pressure for plotting later
  for (int i=0; i<N*2; i++) {
    sensor.Readout();
    sample[i/2] = sensor.GetPres()/100;
    base_pressure += sample[i/2];
    //sample[i] = 0;
    //Serial.println(sample[i/2]); // pressure in mbar
    delay(T);
  }
  base_pressure /= N*2;
  
  Serial.println("***");
  n = 0;  // oldest sample, first to be replaced in buffer

  //delay(5000);
  
}

void loop() {
  double result = 0;
  
  // read new pressure into circular buffer position n
  // increment n
  // set result to 0
  // starting at n (oldest sample), continuing to n+(N-1) mod N (aka n-1)
  //    calculate filter*sample, add to result
  //  scale result??
  // that's the rise/fall rate. rise is positive

  sensor.Readout();
  sample[n] = sensor.GetPres()/100;
  Serial.print((sample[n]-base_pressure)*25);  // scaled and shifted for serial plotter
  //Serial.println(sample[n]);
  
  n++;
  if (n >= N) {
    n = 0;
  }
  
  result = 0;
  for(int i=0; i<N; i++) {
    result += (filter[i] * sample[ (n+i) % N ]);
    if (0) {
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

  //result *= 1000;

  //Serial.print("Result: ");
  Serial.print("\t");
  Serial.println(result);

  if (result > RISE_THRESHOLD) {
    digitalWrite(13, HIGH);
  } else {
    if (result < SINK_THRESHOLD) {
       digitalWrite(13, (millis() % 256) > 128);
    } else {
       digitalWrite(13, LOW);
    }
  }
  
  delay(T);
}

void test_crc() {
  sensor.ReadProm();
  sensor.Readout(); 
  Serial.print("CRC=0x");
  Serial.print(sensor.Calc_CRC4(), HEX);
  Serial.print(" (should be 0x");
  Serial.print(sensor.Read_CRC4(), HEX);
  Serial.print(")\n");
  Serial.print("Test Code CRC=0x");
  Serial.print(sensor.CRCcodeTest(), HEX);
  Serial.println(" (should be 0xB)");
}
