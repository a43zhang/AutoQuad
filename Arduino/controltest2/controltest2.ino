#include <Servo.h> 

#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>

#include <Wire.h>
#include <Adafruit_MPL3115A2.h>

// SoftwareSerial mySerial(3, 2);
// Adafruit_GPS GPS(&mySerial);

// #define GPSECHO  true

// boolean usingInterrupt = false;
// void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

// struct GPS_coords

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
int txth = 2;
int txro = 3;
int txpi = 4;
int txya = 5;
int txmo = 12;
  
//lpf for pwm outputs
int prev_th = 0;
int prev_ro = 0;
int prev_pi = 0;
int prev_ya = 0;
int prev_mo = 0;

float alt = 330;
float hold_height = 350;

//float angle2home = 0;

//GPS home;

void setup() 
{ 

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(9600);
  Serial.println("WTF");

  // // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  // GPS.begin(9600);
  
  // // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  // GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // // uncomment this line to turn on only the "minimum recommended" data
  // //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
  // // the parser doesn't care about other sentences at this time
  
  // // Set the update rate
  // GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  // // For the parsing code to work nicely and have time to sort thru the data, and
  // // print it out we don't suggest using anything higher than 1 Hz

  // // Request updates on antenna status, comment out to keep quiet
  // GPS.sendCommand(PGCMD_ANTENNA);

  // // the nice thing about this code is you can have a timer0 interrupt go off
  // // every 1 millisecond, and read data from the GPS for you. that makes the
  // // loop code a heck of a lot easier!
  // useInterrupt(true);

  // delay(1000);
  // // Ask for firmware version
  // mySerial.println(PMTK_Q_RELEASE);


  pinMode(txth, INPUT);
  pinMode(txro, INPUT);
  pinMode(txpi, INPUT);
  pinMode(txya, INPUT);
  pinMode(txmo, INPUT);
  
  thrust.attach(6);  //w
  roll.attach(7);    //b
  pitch.attach(8);   //y
  yaw.attach(9);     //g
  mode.attach(10);   //o
  baro.begin();
  alt = baro.getAltitude();

  hold_height = alt + 10;
  
} 


// // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
// SIGNAL(TIMER0_COMPA_vect) {
//   char c = GPS.read();
//   // if you want to debug, this is a good time to do it!
// #ifdef UDR0
//   if (GPSECHO)
//     if (c) UDR0 = c;  
//     // writing direct to UDR0 is much much faster than Serial.print 
//     // but only one character can be written at a time. 
// #endif
// }

// void useInterrupt(boolean v) {
//   if (v) {
//     // Timer0 is already used for millis() - we'll just interrupt somewhere
//     // in the middle and call the "Compare A" function above
//     OCR0A = 0xAF;
//     TIMSK0 |= _BV(OCIE0A);
//     usingInterrupt = true;
//   } else {
//     // do not call the interrupt function COMPA anymore
//     TIMSK0 &= ~_BV(OCIE0A);
//     usingInterrupt = false;
//   }
// }

void loop()
{ 
    Serial.print("what the hell ");
    alt = baro.getAltitude();
    Serial.print("herte? ");
    // Serial.print("Fix: "); Serial.print((int)GPS.fix);
    // Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    // Serial.print("Location: ");
    // Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    // Serial.print(", "); 
    // Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    // Serial.print("Location (in degrees, works with Google Maps): ");
    // Serial.print(GPS.latitudeDegrees, 4);
    // Serial.print(", "); 
    // Serial.println(GPS.longitudeDegrees, 4);
    // Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    // Serial.print("Angle: "); Serial.println(GPS.angle);
    // Serial.print("Altitude: "); Serial.println(GPS.altitude);

  th = pulseIn(txth, HIGH);
  ro = pulseIn(txro, HIGH);
  pi = pulseIn(txpi, HIGH);
  ya = pulseIn(txya, HIGH);
  mo = pulseIn(txmo, HIGH);
  
  // // Set home when GPS gets fix
  // if((int)GPS.fix == 1 && home.x == 0)
  // {
  //   home.x = GPS.x;
  //   home.y = GPS.y;
  //   home.z = GPS.z;
  // }
    Serial.print("height: ");
    Serial.print(alt);
    Serial.print("hheight: ");
    Serial.print(hold_height);
    Serial.print("th: ");
    Serial.print(th);
    Serial.print(" ro: ");
    Serial.print(ro);
    Serial.print(" pi: ");
    Serial.print(pi);
    Serial.print(" ya: ");
    Serial.print(ya);
    Serial.print(" mo: ");
    Serial.println(mo);
 
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
    //go home
    alt_hold();
    // angle2home = constrain(angle_to_home(home, current)*1.2 + 1500,1000,1900);
    // yaw.writeMicroseconds(angle2home);

    // //if angle is within 15 degrees of home, go forward 
    // //(but still yaw towards home)
    // if (angle2home < 15 && alt > hold_height-5)
    // {
    //     slow_hold();  
    // }
  }
}

int lowpass(int input, int prev_input)
{
    float a = 0.15;
    int output = prev_input + int(a*float(input - prev_input));
    return output;
}
void alt_hold()
{
  // make sure to include clamping on reasonable values!
  float out = constrain((hold_height-alt+1000)*1.75, 1111, 1900);
  thrust.writeMicroseconds(out);
}

// float angle_to_home(GPS home, GPS current)
// {
//   //pos(1) to home(2) wrt N
//   numer = sin(long2-long1)*cos(lat2);
//   denom = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(long2-long1);
//   angle = atan2( numer , denom );
//   angle = angle*180/pi + 180;
//   home_wrtN = mod(angle,360);

//   dir_wrtN = GPS.angle;

//   angle = home_wrtN - dir_wrtN;

//   return (angle - 360*abs(angle)/(-angle)*(abs(angle)>180))
// }

// void slow_hold()
// {
//   float out = constrain((5 - GPS.speed)*320, 1400, 1900);
//   pitch.writeMicroseconds(out);
// }
