#include <Servo.h>

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
int txth = 5;

void setup()
{
  Serial.begin(9600);
  pinMode(txth, INPUT);

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

  th = pulseIn(txth, HIGH);


  Serial.print("th: ");
  Serial.println(th);


  if (th > 1100)
  {
    thrust.writeMicroseconds(th);
  }
  else
  {
    //thrust.writeMicroseconds(th);
    thrust.write(70);
    roll.write(85);
    pitch.write(85);
    yaw.write(120);

    delay(2000);

    thrust.write(70);
    roll.write(85);
    pitch.write(85);
    yaw.write(50);

    delay(2000);
  }
}

