
// calculate rotation orientation with respect to forward x, left, y, and down z
#include <Servo.h>
#include <SPI.h> // Included for SFE_LSM9DS0 library
#include <Wire.h>
#include <SFE_LSM9DS0.h>
// SDO_XM and SDO_G are both grounded, therefore our addresses are:
#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW
// parameters for this constructor are:
// [SPI or I2C Mode declaration], [gyro I2C address], [xm I2C address]
LSM9DS0 dof(MODE_I2C, LSM9DS0_G, LSM9DS0_XM);
//Interrupt Pin Definitions
const byte INT1XM = 2; // INT1XM tells us when accel data is ready
const byte INT2XM = 3; // INT2XM tells us when mag data is ready
const byte DRDYG = 4;  // DRDYG tells us when gyro data is ready
//Magnetometer variables
float startHeadingMagX,   startHeadingMagY,   startHeadingMagZ;
float headingMagX,       headingMagY,         headingMagZ;
float magZ,               magY,               magX;
float initialMagX = headingMagX;
float initialMagY = headingMagY;
float mx,                 my,                 mz;
//Accelerometer Variables
float startHeadingAccelX, startHeadingAccelY, startHeadingAccelZ;
float headingAccelX,      headingAccelY,      headingAccelZ;
float accelX,             accelY,             accelZ;
float angleX,             angleY,             angleZ;
float magAccel = 1; //should be 1 g
//Gyroscope variables
float gyroXinitial=0,     gyroYinitial=0,     gyroZinitial=0;
float headingGyroX,       headingGyroY,       headingGyroZ = 0;
float gyroX,              gyroY,              gyroZ;
float vectGyroX,          vectGyroY,          vectGyroZ; 
float gAngleX,            gAngleY,            gAngleZ;
float gyroDegAboutZ = 0;
float gyroDegAboutY = 0;
float gyroRotRateAboutZ = 0;
float gyroRotRateAboutY = 0;
//Servo Variables
Servo servoHoriz;
Servo servoVert;
float angleHoriz = 0 , angleVert = 0;
int buttonValue;
float startAngleH = 81;
float startAngleV = 95;
int angleCommand1, angleCommand2;
//Time variables
unsigned long loopIterateTime = 0;
float oldVert = 0;
float oldHoriz = 0;

void setup()
{// Set up interrupt pins as inputs:
  Serial.println("in setup()");
  pinMode(INT1XM, INPUT); //accelorometer
  pinMode(INT2XM, INPUT); //magnetometer
  pinMode(DRDYG, INPUT);  //gyroscope

  Serial.begin(115200); // Start serial at 115200 bps
  // You can either call it with no parameters (the easy way):
  // uint16_t status = dof.begin();
  // Or call it with declarations for sensor scales and data rates:  
  uint16_t status = dof.begin(dof.G_SCALE_500DPS, dof.A_SCALE_4G, dof.M_SCALE_2GS);
  //Set the sample rates of the IMU, more options are at the bottom of the sketch
  dof.setAccelODR(dof.A_ODR_25); //25 //Set Accel ODR (Hz);
  dof.setGyroODR(dof.G_ODR_95_BW_25);//15.2 //Gyro ODR/Cutoff (Hz):
  dof.setMagODR(dof.M_ODR_100); //100// Set Magnetometer ODR (Hz):

  servoHoriz.attach(9,600,2400);   // attaches lower servo to pin 9 
  servoVert.attach(10, 600, 2400); //attaches upper servo to pin 10
  pinMode(11, INPUT_PULLUP);       //creates a pushbutton if needed on pin 11
  delay(500);
  if(digitalRead(INT2XM))         // if there is magnetometer data, read it
  {
    dof.readMag();
  }
  headingMagX = dof.calcMag(dof.mx); //x component of mag field in Gause
  headingMagY = dof.calcMag(dof.my); //y component of mag field in Gause
  headingMagZ = dof.calcMag(dof.mz); //z component of mag field in Gause
  startHeadingMagZ = getRotationFromXYPlane(headingMagX, headingMagY, headingMagZ);//initial z
  initialMagX = headingMagX;
  initialMagY = headingMagY;

  if (digitalRead(INT1XM)) //if there is accelerometer data, read it
  {
    dof.readAccel();
  }
  accelX = dof.calcAccel(dof.ax); //x component of accel in g's
  accelY = dof.calcAccel(dof.ay); //y component of accel in g's
  accelZ = dof.calcAccel(dof.az); //z component of accel in g's
  startHeadingAccelX = calcHeadingAccel(accelY, accelZ);//initial x
  startHeadingAccelY = calcHeadingAccel(accelX, accelZ);//initial y
  startHeadingAccelZ = calcHeadingAccel(accelX, accelZ);//initial z

  for (int samples = 0; samples < 50 ; samples++) 
  {
    if (digitalRead(DRDYG)) //if therei is new gyro data, read it
    {
      dof.readGyro();
    }
    gyroXinitial = dof.calcGyro(dof.gx)+gyroXinitial;
    gyroYinitial = dof.calcGyro(dof.gy)+gyroYinitial;
    gyroZinitial = dof.calcGyro(dof.gz)+gyroZinitial;
  }
  gyroXinitial = gyroXinitial/50;
  gyroYinitial = gyroYinitial/50;
  gyroZinitial = gyroZinitial/50;
  Serial.println("initial gyro readings deg/s");
  Serial.print("gyroX ");
  Serial.print(gyroXinitial);
  Serial.print("  gyroY ");
  Serial.print(gyroYinitial);
  Serial.print("  gyroZ ");
  Serial.print(gyroZinitial);
}

void loop()
{
  if (digitalRead(DRDYG)) //if there is new gyro data, read it
  {
    unsigned long milliseconds = millis();
    loopIterateTime = milliseconds - loopIterateTime;
    Serial.println(loopIterateTime);
    dof.readGyro();
    gyroX = dof.calcGyro(dof.gx)-gyroXinitial;
    gyroY = dof.calcGyro(dof.gy)-gyroYinitial;
    gyroZ = dof.calcGyro(dof.gz)-gyroZinitial;
    Serial.print("gyroX ");
    Serial.print(gyroX) ;
    Serial.print("  gyroY ");
    Serial.print(gyroY) ;
    Serial.print("  gyroZ ");
    Serial.println(gyroZ) ;
    if ((gyroX < 5 && gyroX > -5)&&(gyroY < 5 && gyroY > -5)&&(gyroZ < 5 && gyroZ > -5))
    {
      gyroRotRateAboutZ = 0;
    }
    else
    {
      gyroRotRateAboutY = getGyroRotRateAboutY(gyroX, gyroY, gyroZ);
      gyroRotRateAboutZ = getGyroRotRateAboutZ(gyroX, gyroY, gyroZ);
    }
    gyroDegAboutY = -getDegreesFromGyro(gyroRotRateAboutY, 25)+gyroDegAboutY;
    gyroDegAboutZ = getDegreesFromGyro(gyroRotRateAboutZ, 25)+gyroDegAboutZ;
  }
  if (digitalRead(INT1XM)) //if there is new accelerometer data, read it
  {
    dof.readAccel();
    accelX = dof.calcAccel(dof.ax); //get x component in g's
    accelY = dof.calcAccel(dof.ay); //get y component in g's
    accelZ = dof.calcAccel(dof.az); //get z component in g's
    magAccel = getMagnitude(accelX, accelY, accelZ); //pythagorean theorem
    accelX = accelX/magAccel;   //x of accel unit vector
    accelY = accelY/magAccel;   //x of accel unit vector
    accelZ = accelZ/magAccel;   //x of accel unit vector
  }

  angleHoriz = -0.5*gyroDegAboutZ;
  Serial.print(  " degZ calc from gyro ");
  Serial.println(angleHoriz);
  Serial.print(  " degY calc from gyro ");
  Serial.println(gyroDegAboutY);

  angleVert = 0.5*getVertAngle(accelX, accelY, accelZ);
  Serial.print(  " deg calc from accel ");
  Serial.println(angleVert);

  angleHoriz = startAngleH - angleHoriz;
  if (abs(angleHoriz - oldHoriz)<2)
  {
    angleHoriz=oldHoriz;
  }
  oldHoriz = angleHoriz;
  
  angleVert = startAngleV+angleVert; 
  if (abs(angleVert - oldVert)<2)
  {
    angleVert=oldVert;
  }
  oldVert = angleHoriz;

  Serial.print(" M headZ ");
  Serial.println(calcHeading(headingMagX, headingMagY));
  Serial.print(" angle1 ");
  Serial.print(angleHoriz);
  Serial.print(" angle2 " );
  Serial.println(angleVert);
  Serial.print(" ax " );
  Serial.print(accelX);
  Serial.print(" ay " );
  Serial.print(accelY);
  Serial.print( " az ");
  Serial.println(accelZ);

  angleCommand1 = computeBottomServoCmd(angleHoriz);
  angleCommand2 = computeTopServoCmd(angleVert);

  servoHoriz.write(angleCommand1);
  servoVert.write(angleCommand2);

  checkStopButton();
}

// Here's a simple example function to calculate heading based on
// magnetometer readings. This only works when the 9DOF is flat
// (x-axis normal to gravity).
float calcHeading(float hx, float hy)
{  
  if (hy > 0)
  {
    return 90 - (atan(hx / hy) * 180 / PI);
  }
  else if (hy < 0)
  {
    return 270 - (atan(hx / hy) * 180 / PI);
  }
  else // hy = 0
  {
    if (hx < 0) return 180;
    else return 0;
  }
}

// Here's a function to calculate heading based on
// accelorometer readings. This only works when the 9DOF is flat
// (x-axis normal to gravity).
float calcHeadingAccel(float hx, float hz)
{  
  if (hz > 0 && hx > 0)
  {
    return (atan(hx / hz) * 180 / PI);
  }
  else if (hz > 0 && hx < 0)
  {
    return (atan(hx / hz) * 180 / PI) ;
  }
  else if ( hz < 0 && hx > 0)
  {
    return (-(atan(hz/hx) * 180 / PI)+90) ;
  }
  else if (hz < 0 && hx < 0)
  {
    return (-(atan(hz/hx) * 180 / PI) - 90);
  }
  else if (hz > 0 && hx == 0)
  {
    return 0;
  }
  else if ( hz == 0 && hx < 0)
  {
    return -90;
  } 
  else if ( hz < 0 && hx == 0 )
  {
    return 180;
  }
  else if ( hz == 0 && hx > 1)
  {
    return 90;
  }
}
float getMagnitude(float ax, float ay, float az)
{
  float magnitude = sqrt((ax*ax) + (ay*ay) + (az*az));
  return magnitude;
}
float getGyroMag(float ax, float ay, float az)
{
  float vector = sqrt((ax*ax) + (ay*ay) + (az*az));
  return vector;
}
float getAccelerationAngle(float var, float accelMag)
{
  float angle = acos(var/accelMag);
  return angle;
}
float getGyroRotRateAboutZ(float gx, float gy, float gz)
{
  float dotProd = gx*0 + gy*0 + gz*1 ;
  float gMagnitude = getMagnitude(gx,gy, gz);
  float zMagnitude = getMagnitude(0, 0, 1) ;
  float cosTheta = dotProd/(gMagnitude*zMagnitude);
  return  (gMagnitude * cosTheta) ;

}
float getGyroRotRateAboutY(float gx, float gy, float gz)
{
  float dotProd = gx*0 + gy*1 + gz*0 ;
  float gMagnitude = getMagnitude(gx,gy, gz);
  float yMagnitude = getMagnitude(0, 1, 0) ;
  float cosTheta = dotProd/(gMagnitude*yMagnitude);
  return(gMagnitude * cosTheta) ;
}
float getDegreesFromGyro(float rotationRateAboutZ, float rotTime)
{
  return (rotationRateAboutZ*(float)(rotTime)/1000);
}

float getVertAngle(float ax, float ay, float az)
{
  float gravMaganitude = getMagnitude(ax, ay , az);
  if (ax <= 0)
  {
    return (90-acos(ax/(gravMaganitude))*180/PI);
  }
  if (ax > 0)
  {
    return (90-acos(ax/(gravMaganitude))*180/PI);
  }
}

float getRotFromZaxis(float accelX, float accelY, float accelZ)
{ //z unit vector is (0, 0, 1)
  float dotProd = (0*accelX+0*accelY+1*accelZ);
  float accelMag = getMagnitude(accelX, accelY, accelZ);
  if ( ((accelX>0)&&(accelY>=0)) || ((accelX>0)&&(accelY<=0)) )
  {
    return -acos(dotProd/accelMag)*180/PI;
  }
  if ( ((accelX<0)&&(accelY<=0)) || ((accelX<0)&&(accelY>=0)) )
  {
    return acos(dotProd/accelMag)*180/PI;
  }
}
//function takes magnetic field vector B with elements (mx, my, and mz)
float getRotationFromXYPlane( float mx, float my, float mz)
{ //recall from calc 2, the vector projection, u,  for magnetic field vector B,
  //on the XY plane with normal vector N_hat = [0 0 1] is
  // u = B - N(B dot N)
  float BdotN = (mx * 0 + my * 0 + mz * 1);
  float BdotNx= 0 * BdotN;
  float BdotNy= 0 * BdotN;
  float BdotNz= 1 * BdotN;
  float ux = mx - BdotNx;
  float uy = my - BdotNy;
  float uz = mz - BdotNz;

  // recall from calc 2, the angle, alpha, between u, 
  // and the unit vector normal the the XY plane, u_hat is found from:
  // sin(alpha) = magnitude(n_hat dot u)/((magnitude(n_hat)*magnitude(u))
  // n_hat = [ 0 1 0];
  float uMagnitude = getMagnitude(ux, uy, uz);
  float nMagnitude = 1;
  float dotProd = (ux*0 + uy * 1 + uz * 0);
  float dotProdMagnitude = abs(dotProd);

  if (ux > 0 && uy > 0)
  {
    return asin(dotProdMagnitude/(nMagnitude*uMagnitude)) * 180 / PI;
  }
  if (ux > 0 && uy < 0)
  {
    return asin(dotProdMagnitude/(nMagnitude*uMagnitude)) * 180 / PI + 90;
  }
  if (ux < 0 && uy < 0)
  {
    return -(asin(dotProdMagnitude/(nMagnitude*uMagnitude)) * 180 / PI + 90);
  }
  if (ux < 0 && uy > 0)
  {
    return -(asin(dotProdMagnitude/(nMagnitude*uMagnitude)) * 180 / PI);
  }
}
float getRotationFromInitialBfield(float mx, float my, float mz, float startAngle)
{
  float newAngle = getRotationFromXYPlane(mx, my, mz);
  return startAngle - newAngle;
}

int computeTopServoCmd(float angle)
{
  int angleCommand = 0 ;
  if (angle > 115)
  {
    angleCommand = (int)115;
  }
  else 
  {
    angleCommand = (int)angle;
  }
  return angleCommand;
}

int computeBottomServoCmd(float angle)
{
  int angleCommand = 0 ;
  if (angle > 120)
  {
    angleCommand = (int)120;
  }
  else 
  {
    angleCommand = (int)angle;
  }
  return angleCommand;
}

void checkStopButton()
{
  if (digitalRead(11) == 0)
  {
    exit(0);
  }
}  
//Accelorometer Sample Rate Options
//  Set  Accel ODR (Hz):
//  dof.setAccelODR(dof.A_ODR_3125); //3.123
//  dof.setAccelODR(dof.A_ODR_625);  //6.25
//  dof.setAccelODR(dof.A_ODR_125);  //12.5
//  dof.setAccelODR(dof.A_ODR_25);     //25
//  dof.setAccelODR(dof.A_ODR_50);   //50
//  dof.setAccelODR(dof.A_ODR_100);  //100
//  dof.setAccelODR(dof.A_ODR_200);  //200 
//  dof.setAccelODR(dof.A_ODR_400);  //400
//  dof.setAccelODR(dof.A_ODR_800);  //800
//  dof.setAccelODR(dof.A_ODR_1600); //1600

// Gyroscope Sample Rate Options    
//  Gyro ODR/Cutoff (Hz):
//  dof.setGyroODR(dof.G_ODR_95_BW_125);  //7.6
//  dof.setGyroODR(dof.G_ODR_95_BW_25);   //3.8
//  dof.setGyroODR(dof.G_ODR_190_BW_125); //1.52
//  dof.setGyroODR(dof.G_ODR_190_BW_25);  //1.52
//  dof.setGyroODR(dof.G_ODR_190_BW_50);  //3.8
//  dof.setGyroODR(dof.G_ODR_190_BW_70);  //2.71
//  dof.setGyroODR(dof.G_ODR_380_BW_20);  //19
//  dof.setGyroODR(dof.G_ODR_380_BW_25);  //15.2
//  dof.setGyroODR(dof.G_ODR_380_BW_50);  //7.6
//  dof.setGyroODR(dof.G_ODR_380_BW_100); //3.8
//  dof.setGyroODR(dof.G_ODR_760_BW_30);  //25.33
//  dof.setGyroODR(dof.G_ODR_760_BW_35);  //21.71
//  dof.setGyroODR(dof.G_ODR_760_BW_50);  //15.2
//  dof.setGyroODR(dof.G_ODR_760_BW_100); //7.6

// Magnetometer Sample rate Options
//  Set Magnetometer ODR (Hz):
//  dof.setMagODR(dof.M_ODR_3125);  //3.25
//  dof.setMagODR(dof.M_ODR_625);   //6.45
//  dof.setMagODR(dof.M_ODR_125);   //12.5
//  dof.setMagODR(dof.M_ODR_25);    //25
//  dof.setMagODR(dof.M_ODR_50);    //50
//  dof.setMagODR(dof.M_ODR_100);   //100



