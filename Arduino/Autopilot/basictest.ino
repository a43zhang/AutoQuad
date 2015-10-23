#include <Servo.h> 

struct GPS

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
 
void loop() 
{              
  th = pulseIn(txth, HIGH)
  ro = pulseIn(txro, HIGH)
  pi = pulseIn(txpi, HIGH)
  ya = pulseIn(txya, HIGH)
  mo = pulseIn(txmo, HIGH)
  
  /*
  // Set home when GPS gets fix
  if(fix = 1 && home.x == 0)
  {
    home.x = GPS.x;
    home.y = GPS.y;
    home.z = GPS.z;
  }
  */
  
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
    yaw.writeMicroseconds(angle_to_home(home, current)*10);
    pitch.writeMicroseconds(1600); //MAKE IT SLOW
  }
  delay(50);
} 

void alt_hold()
{
  // make sure to include clamping on reasonable values!
  thrust.write((350-alt)*0.6);
}

float angle_to_home(GPS home, GPS current)
{
  //direction of travel with respect to North
  dir_wrtN = arctan(vx_wrth/vy_wrth)(deg) + 180*x/abs(x)*(y<0)
  //direction to home with respect to North
  home_wrtN = arctan(x_wrth/y_wrth)(deg) + 180*x/abs(x)*(y<0)
  
  angle = home_wrtN - dir_wrtN;

  return (angle - 360*abs(angle)/(-angle)*(abs(angle)>180))
}