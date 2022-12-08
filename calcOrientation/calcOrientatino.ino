//calculateRobotPose

// calculate pose with respect to north (y), east (x) and gravity (z)

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
// A boolean to keep track of whether we're printing raw (ADC)
// or calculated (g's, DPS, Gs) sensor data:
boolean printRaw = true;

float heading = 0;
float headingX;
float headingY;

void setup()
{
  // Set up interrupt pins as inputs:
  pinMode(INT1XM, INPUT);
  pinMode(INT2XM, INPUT);
  pinMode(DRDYG, INPUT);
  
  Serial.begin(115200); // Start serial at 115200 bps
  // Use the begin() function to initialize the LSM9DS0 library.
  // You can either call it with no parameters (the easy way):
  //uint16_t status = dof.begin();
  // Or call it with declarations for sensor scales and data rates:  
  uint16_t status = dof.begin(dof.G_SCALE_2000DPS, dof.A_SCALE_6G, dof.M_SCALE_2GS);
  
  // begin() returns a 16-bit value which includes both the gyro and
  // accelerometers WHO_AM_I response. You can check this to make sure
  // communication was successful.
  Serial.println(status, HEX);
}



void loop()
{
  //steamAll();
  headingX = dof.calcMag(dof.mx);
  headingY = dof.calcMag(dof.my);
  heading = calcHeading(headingX, headingY);
  Serial.println(heading);
  
  
}


// This function will print all data from all sensors at once.
// It'll wait until every sensor interrupt triggers before
// printing.
void streamAll()
{
  if ((digitalRead(INT2XM)) && (digitalRead(INT1XM)) &&
      (digitalRead(DRDYG)))
  {
    printAccel();
    printGyro();
    printMag();
  }
}

void printAccel()
{
  // Only read from the accelerometer if the accel interrupts,
  // which means that new data is ready.
  if (digitalRead(INT1XM))
  {
    // Use the readAccel() function to get new data from the accel.
	// After calling this function, new values will be stored in
	// the ax, ay, and az variables.
    dof.readAccel();
	
    Serial.print("A: ");
    if (printRaw)
    {
      Serial.print(dof.ax);
      Serial.print(", ");
      Serial.print(dof.ay);
      Serial.print(", ");
      Serial.println(dof.az);
    }
    else
    {
	  // Using the calcAccel helper function, we can get the
	  // accelerometer readings in g's.
      Serial.print(dof.calcAccel(dof.ax));
      Serial.print(", ");
      Serial.print(dof.calcAccel(dof.ay));
      Serial.print(", ");
      Serial.println(dof.calcAccel(dof.az));
    }
  }
}

void printGyro()
{
  // Only read from the gyro if the DRDY interrupts,
  // which means that new data is ready.
  if (digitalRead(DRDYG))
  {
    // Use the readGyro() function to get new data from the gyro.
	// After calling this function, new values will be stored in
	// the gx, gy, and gz variables.
    dof.readGyro();
	
    Serial.print("G: ");
    if (printRaw)
    {
      Serial.print(dof.gx);
      Serial.print(", ");
      Serial.print(dof.gy);
      Serial.print(", ");
      Serial.println(dof.gz);
    }
    else
    {
	  // Using the calcGyro helper function, we can get the
	  // gyroscope readings in degrees per second (DPS).
      Serial.print(dof.calcGyro(dof.gx));
      Serial.print(", ");
      Serial.print(dof.calcGyro(dof.gy));
      Serial.print(", ");
      Serial.println(dof.calcGyro(dof.gz));
    }
  }
}

void printMag()
{
  // Only read from the magnetometer if the INT2XM interrupts,
  // which means that new data is ready.
  if (digitalRead(INT2XM))
  {
    // Use the readMag() function to get new data from the mag.
	// After calling this function, new values will be stored in
	// the mx, my, and mz variables.
    dof.readMag();
	
    Serial.print("M: ");
    if (printRaw)
    {
      Serial.print(dof.mx);
      Serial.print(", ");
      Serial.print(dof.my);
      Serial.print(", ");
      Serial.print(dof.mz);
      Serial.print(", ");
      Serial.println(calcHeading(dof.mx, dof.my));
    }
    else
    {
	  // Using the calcMg helper function, we can get the
	  // magnetometer readings in gauss (Gs).
      Serial.print(dof.calcMag(dof.mx), 4);
      Serial.print(", ");
      Serial.print(dof.calcMag(dof.my), 4);
      Serial.print(", ");
      Serial.print(dof.calcMag(dof.mz), 4);
      Serial.print(", ");
      Serial.println(calcHeading(dof.mx, dof.my));
    }
  }
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
