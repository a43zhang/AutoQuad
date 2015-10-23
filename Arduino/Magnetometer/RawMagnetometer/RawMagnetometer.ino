#include <Wire.h>
#include <HMC5883L.h>
 
HMC5883L compass;
int i;
float X_tot, Y_tot, Z_tot;
float X,Y,Z;
void setup()
{
   Wire.begin();
   Serial.begin(9600);
   compass = HMC5883L();
 
   Serial.println("Setting scale to +/- 1.3Ga");
   int error = compass.SetScale(1.3);
   if(error != 0)
     Serial.println(compass.GetErrorText(error));
 
   Serial.println("Setting measurement mode to continuous");
   error = compass.SetMeasurementMode(Measurement_Continuous);
   if(error != 0)
     Serial.println(compass.GetErrorText(error));
 
   i = 0;
   X_tot = 0;
   Y_tot = 0;
   Z_tot = 0;
   X = 0; Y = 0; Z = 0;
}
 
void loop()
{
   MagnetometerRaw raw = compass.ReadRawAxis();
   if(i<499){
     X_tot += raw.XAxis;
     Y_tot += raw.YAxis;
     Z_tot += raw.ZAxis;
   }else{
     X = X_tot/500;
     Y = Y_tot/500;
     Z = Z_tot/500;
     X_tot = 0; Y_tot = 0; Z_tot = 0;
     i = 0;
     Serial.print(i+":\t");
     Serial.print(X);
     Serial.print(" ");
     Serial.print(Y);
     Serial.print(" ");
     Serial.print(Z);
     Serial.println(" ");
     delay(500);
   }
   i++;
}
