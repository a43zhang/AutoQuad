float output = 0;

void setup()
{

  Serial.begin(9600);
// 1 is home
// 2 is current
  float h_long = -80.542604;
  float c_long = -80.542639;
  float h_lat = 43.465839;
  float c_lat = 43.466839;

  //Calculate angle from current position to home position
  float numer = sin(c_long-h_long)*cos(c_lat);
  float denom = cos(h_lat)*sin(c_lat) - sin(h_lat)*cos(c_lat)*cos(c_long-h_long);
  float angle = atan2( numer , denom );
  angle = angle*180/3.1415 + 180;
  
  //We have velocity direction and home direction
  int home_wrtN = int(angle) % 360;
  float dir_wrtN = 0;

  //find angle between velocity direction and home direction
  angle = float(home_wrtN) - float(dir_wrtN);
  // RETURN 0 IF 0
  if (angle == 0)
    angle = 0.1;
  output = angle + 360*(abs(angle)/(-angle))*(abs(angle)>180);
}
void loop()
{
  Serial.println(output);
  delay(10000);
}