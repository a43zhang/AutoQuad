/* 
Wire: 20 (SDA), 21 (SCL)
GPS:  18 (tx1), 19 (rx1)

Inputs
th      41
ro      43
pi      45
ya      47
mo      49

Outputs
thrust  42 
roll    44
pitch   46
yaw     48
mode    50

*/


#include <Servo.h> 

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

#include "GPS_coords.cpp"

const float pie = 3.1415;

Adafruit_GPS GPS(&Serial1);
HardwareSerial mySerial = Serial1;


#define GPSECHO  false

boolean usingInterrupt = false;

// struct GPS_coords
// {
//   float lat = 0;
//   float lon = 0;
//   float alt_G = 0;
// };

void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy
// int lowpass(int, int);
// void alt_hold();
// void slow_hold();
// float angle_to_home(GPS_coords, GPS_coords);

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

Servo thrust;
Servo roll;
Servo pitch;
Servo yaw;  
Servo mode;

//pwm values for output to fc
int th = 0;
int ro = 0;
int pi = 0;
int ya = 0;
int mo = 0;

//tx pins
int txth = 41;
int txro = 43;
int txpi = 45;
int txya = 47;
int txmo = 49;
  
//lpf for pwm outputs
int prev_th = 0;
int prev_ro = 0;
int prev_pi = 0;
int prev_ya = 0;
int prev_mo = 0;
float raw_alt = 0;
float alt = 250;
float hold_height = 350;

float angle2home = 0;

GPS_coords home;
GPS_coords current;

void setup() 
{ 

  Serial.begin(9600);
  Serial.print("test");
  GPS.begin(9600);
  
  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // the parser doesn't care about other sentences at this time
  
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // For the parsing code to work nicely and have time to sort thru the data, and
  // print it out we don't suggest using anything higher than 1 Hz

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);

  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  pinMode(txth, INPUT);
  pinMode(txro, INPUT);
  pinMode(txpi, INPUT);
  pinMode(txya, INPUT);
  pinMode(txmo, INPUT);
  
  thrust.attach(42);  //w
  roll.attach(44);    //b
  pitch.attach(46);   //y
  yaw.attach(48);     //g
  mode.attach(50);      //o
  baro.begin();
  alt = baro.getAltitude();
  if(alt > 600)
    alt = 300;  
  hold_height = alt + 10;
} 

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

void loop()
{ 

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  raw_alt = baro.getAltitude();
  
  if (raw_alt < 600 && raw_alt > 200) //and close to previous value? CAREFUL WHEN DIFFERENT LOCATION
    alt = raw_alt;

   //Set home when GPS gets fix
  if((int)GPS.fix == 1 && home.lat == 0)
  {
    home.lat = GPS.latitudeDegrees;
    home.lon = GPS.longitudeDegrees;
    home.alt_G = GPS.altitude;
  }
   //Set home when GPS gets fix
  if((int)GPS.fix == 1)
  {
    current.lat = GPS.latitudeDegrees;
    current.lon = GPS.longitudeDegrees;
    current.alt_G = GPS.altitude;
  }

  // Serial.print("Fi: "); Serial.print((int)GPS.fix);
  // Serial.print("Loc: ");
  // Serial.print(GPS.latitude, 6); 
  // Serial.print(", "); 
  // Serial.print(GPS.longitude, 6); 
  // Serial.print("Loc2: ");
  // Serial.print(GPS.lat, 6);
  // Serial.print(", "); 
  // Serial.print(GPS.lon, 6);
  // Serial.print(" knot: "); Serial.print(GPS.speed);
  Serial.print(" Ang: "); Serial.print(GPS.angle);

  Serial.print("ho: ");
  Serial.print(home.lat, 6);
  Serial.print(", "); 
  Serial.print(home.lon, 6);


  Serial.print("cur: ");
  Serial.print(current.lat, 6);
  Serial.print(", "); 
  Serial.print(current.lon, 6);

  Serial.print(" ath ");
  Serial.println(angle_to_home(home, current));
    // Serial.print(" Alt: "); Serial.print(GPS.altitude);
  // Serial.print("\the: ");
  // Serial.print(alt);
  // Serial.print("\tth: ");
  // Serial.print(th);
  // Serial.print(" ro: ");
  // Serial.print(ro);
  // Serial.print(" pi: ");
  // Serial.print(pi);
  // Serial.print(" ya: ");
  // Serial.print(ya);
  // Serial.print(" mo: ");
  // Serial.println(mo);

  th = pulseIn(txth, HIGH);
  ro = pulseIn(txro, HIGH);
  pi = pulseIn(txpi, HIGH);
  ya = pulseIn(txya, HIGH);
  mo = pulseIn(txmo, HIGH);

 
 
  if (mo > 1500)
  {
    // I could alternatively round to the nearest hundred for example for
    // fast response and no oscillation but stepwise changes
    th = lowpass(th,prev_th);
    ro = lowpass(ro,prev_ro);
    pi = lowpass(pi,prev_pi);
    ya = lowpass(ya,prev_ya);
    mo = lowpass(mo,prev_mo);

    prev_th = th;
    prev_ro = ro;
    prev_pi = pi;
    prev_ya = ya;
    prev_mo = mo;
    
    thrust.writeMicroseconds(th);
    roll.writeMicroseconds(ro);     
    pitch.writeMicroseconds(pi);  
    yaw.writeMicroseconds(ya);
  }
  else
  {
    //go home: go to altitude, go forwards slowly and rotate towards home

    alt_hold();
    slow_hold();
    angle2home = constrain(angle_to_home(home, current)*1.3 + 1460,1100,1700);
    yaw.writeMicroseconds(angle2home);

    // //if angle is within 15 degrees of home, go forward 
    // //(but still yaw towards home)
    // if (abs(angle2home) < 15 && alt > hold_height-5)
    // {
    //     slow_hold();  
    // }
  
  }
}

int lowpass(int input, int prev_input)
{
  float a = 0.5;
  int output = prev_input + int(a*float(input - prev_input));
  return output;
}

void alt_hold()
{
  // make sure to include clamping on reasonable values!
  float out = constrain((hold_height-alt+1000)*1.75, 1111, 1700);
  thrust.writeMicroseconds(out);
}

void slow_hold()
{
  float out = constrain((5 - GPS.speed)*20 + 1460, 1370, 1650);
  pitch.writeMicroseconds(out);
}

float angle_to_home(GPS_coords home, GPS_coords current)
{

  float h_long = home.lon;
  float c_long = current.lon;
  float h_lat = home.lat;
  float c_lat = current.lat;

  //Calculate angle from current position to home position
  float numer = sin(c_long-h_long)*cos(c_lat);
  float denom = cos(h_lat)*sin(c_lat) - sin(h_lat)*cos(c_lat)*cos(c_long-h_long);
  float angle = atan2( numer , denom );
  angle = angle*180/pie + 180;
  
  //We have velocity direction and home direction
  int home_wrtN = int(angle) % 360;
  float dir_wrtN = GPS.angle;

  //find angle between velocity direction and home direction
  angle = float(home_wrtN) - float(dir_wrtN);

  return (angle == 0)? 0 : (angle + 360*(abs(angle)/(-angle))*(abs(angle)>180));
}