#include <Servo.h> 
#include <Adafruit_GPS.h>
#include <SoftwareSerial.h>
SoftwareSerial mySerial(3, 2);
Adafruit_GPS GPS(&mySerial);

#define GPSECHO  true

boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

struct GPS_coords

Servo thrust;
Servo roll;
Servo pitch;
Servo yaw;  
Servo mode;

int th = 0;
int ro = 0;
int pi = 0;
int ya = 0;
int mo = 0;

//GPS home;

void setup() 
{ 

  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  Serial.begin(115200);
  Serial.println("test!");

  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
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

  // the nice thing about this code is you can have a timer0 interrupt go off
  // every 1 millisecond, and read data from the GPS for you. that makes the
  // loop code a heck of a lot easier!
  useInterrupt(true);

  delay(1000);
  // Ask for firmware version
  mySerial.println(PMTK_Q_RELEASE);

  int txth = 1;
  int txro = 2;
  int txpi = 3;
  int txya = 4;
  int txmo = 5;

  pinMode(txth, INPUT);
  pinMode(txro, INPUT);
  pinMode(txpi, INPUT);
  pinMode(txya, INPUT);
  pinMode(txmo, INPUT);
  
  thrust.attach(6);  
  roll.attach(7);
  pitch.attach(8);
  yaw.attach(9);
  mode.attach(10);
  
  thrust.write(50);  
  roll.write(85);
  pitch.write(85);
  yaw.write(85);
  mode.write(50);
  
  delay(14000);
  
  //arm: low pitch low roll
  thrust.write(50);  
  roll.write(125);
  pitch.write(50);
  yaw.write(85);
  
  delay(6000);
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

    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", "); 
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
    Serial.print("Location (in degrees, works with Google Maps): ");
    Serial.print(GPS.latitudeDegrees, 4);
    Serial.print(", "); 
    Serial.println(GPS.longitudeDegrees, 4);
    Serial.print("Speed (knots): "); Serial.println(GPS.speed);
    Serial.print("Angle: "); Serial.println(GPS.angle);
    Serial.print("Altitude: "); Serial.println(GPS.altitude);
        
  th = pulseIn(txth, HIGH)
  ro = pulseIn(txro, HIGH)
  pi = pulseIn(txpi, HIGH)
  ya = pulseIn(txya, HIGH)
  mo = pulseIn(txmo, HIGH)
  
  
  // Set home when GPS gets fix
  if((int)GPS.fix == 1 && home.x == 0)
  {
    home.x = GPS.x;
    home.y = GPS.y;
    home.z = GPS.z;
  }
  
  
  if (mo > 1700)
  {
    thrust.writeMicroseconds(th);  
    roll.writeMicroseconds(ro);  
    pitch.writeMicroseconds(pi);  
    yaw.writeMicroseconds(ya);

    Serial.print("th: %d ro: %d pi: %d ya: %d mo: %d", th, ro, pi, ya, mo);

  }
  else
  {
    //go home
    alt_hold();
    yaw.writeMicroseconds(angle_to_home(home, current)*1.2 + 1500);
    go_slow();
  }
}

void alt_hold()
{
  // make sure to include clamping on reasonable values!
  thrust.writeMicroseconds((350-alt+1000)*1.75);
}

float angle_to_home(GPS home, GPS current)
{
  //pos(1) to home(2) wrt N
  numer = sin(long2-long1)*cos(lat2);
  denom = cos(lat1)*sin(lat2) - sin(lat1)*cos(lat2)*cos(long2-long1);
  angle = atan2( numer , denom );
  angle = angle*180/pi + 180;
  home_wrtN = mod(angle,360);

  dir_wrtN = GPS.angle;

  angle = home_wrtN - dir_wrtN;

  return (angle - 360*abs(angle)/(-angle)*(abs(angle)>180))
}

void go_slow()
{
  pitch.writeMicroseconds((5 - GPS.speed)*320);
}