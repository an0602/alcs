#include <Stepper.h>
#include <AccelStepper.h>
#include <stdlib.h>
#include <stdint.h>
#include <Wire.h>
#include <math.h>
#include "Kalman.h"

/************/
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

//function prototypes for I2C
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);

const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data
/*
  Stepper motors 1 and 4 are paired with roll, tilt towards this side is negative roll
  Stepper motors 2 and 3 are paired with roll, tilt towards this side is positive roll
  Stepper motors 1 and 2 are paired with pitch, tilt towards this side is positive pitch
  Stepper motors 3 and 4 are paired with pitch, tilt towards this side is negative pitch
  ****
  Positive speed makes leg longer (counterClockWise). Negative speed (clockwise) makes
  leg shorter
  ***
*/


/************/

volatile signed counter1 = 0;
volatile signed counter2 = 0;
volatile signed counter3 = 0;
volatile signed counter4 = 0;

// Define step counter for each motor
volatile unsigned int motorCount1 = 0;
volatile unsigned int motorCount2 = 0;
volatile unsigned int motorCount3 = 0;
volatile unsigned int motorCount4 = 0;

// Define the pins for each motor
const int MOTOR1_STEP = 47;
const int MOTOR1_DIR = 51;
const int MOTOR2_STEP = 23;
const int MOTOR2_DIR = 25;
const int MOTOR3_STEP = 31;
const int MOTOR3_DIR = 33;
const int MOTOR4_STEP = 39;
const int MOTOR4_DIR = 41;

// Define the number of steps per direction for each motor
const int STEPS_PER_DIRECTION = 400;

// Define the limit switch pins for each motor
const int MOTOR1_LIMIT = 18;
const int MOTOR2_LIMIT = 2;
const int MOTOR3_LIMIT = 3;
const int MOTOR4_LIMIT = 19;

// Define the button pins
const int BUTTON1_PINB = 8;    //button1_blueButton, casters retract until limit-switches is pressed
const int BUTTON2_PINRR = 10;   //button2red_button, casters extend first, then balance

// Define the state of the buttons
int button1State = HIGH;
int lastButton1State = HIGH;
int button2State = HIGH;
int lastButton2State = HIGH;

// Define sensor variables
double actualAngleRoll = 0;
double actualAnglePitch = 0;
const double targetAngleRoll = 0.30;    //this is absolute value, so positive or negative 0.5 is good
const double targetAnglePitch = 0.30;   //this is absolute value, so positive or negative 0.5 is good

// Create a Stepper object for each motor
volatile int highestLeg = 0;
Stepper motor1(STEPS_PER_DIRECTION, MOTOR1_STEP, MOTOR1_DIR);
Stepper motor2(STEPS_PER_DIRECTION, MOTOR2_STEP, MOTOR2_DIR);
Stepper motor3(STEPS_PER_DIRECTION, MOTOR3_STEP, MOTOR3_DIR);
Stepper motor4(STEPS_PER_DIRECTION, MOTOR4_STEP, MOTOR4_DIR);
const int motorDirectionValue = 1;
const uint32_t maxStepsMotor = 55700;

void getHundredValues();
float rollArray[100];
float pitchArray[100];

double getAverage(float myArray[100]);
double averageForRoll;
double averageForPitch;

// function prototypes
void raisePlatform();
void reset1();
void reset2();
void reset3();
void reset4();
void readSensorData(double *myRoll, double *myPitch);
int highestPositionMotorFixedLeg();  //returns 1, 2, 3, or 4, indicating highest stepper caster
void correctMotor1();
void correctMotor2();
void correctMotor3();
void correctMotor4();


void setup() {
  // AccelStepper set maximium speed
  Serial.begin(115200);
  Wire.begin();
  TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (i2cData[0] << 8) | i2cData[1];
  accY = (i2cData[2] << 8) | i2cData[3];
  accZ = (i2cData[4] << 8) | i2cData[5];

#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

  // Set the motor speed in revolutions per minute, previously 400 was hard-coded
  motor1.setSpeed(STEPS_PER_DIRECTION);
  motor2.setSpeed(STEPS_PER_DIRECTION);
  motor3.setSpeed(STEPS_PER_DIRECTION);
  motor4.setSpeed(STEPS_PER_DIRECTION);

  // Configure the limit switch pins as inputs with pull-up resistors enabled
  pinMode(MOTOR1_LIMIT, INPUT_PULLUP);
  pinMode(MOTOR2_LIMIT, INPUT_PULLUP);
  pinMode(MOTOR3_LIMIT, INPUT_PULLUP);
  pinMode(MOTOR4_LIMIT, INPUT_PULLUP);

  // Configure the button pins as inputs with pull-up resistors enabled
  pinMode(BUTTON1_PINB, INPUT_PULLUP);
  pinMode(BUTTON2_PINRR, INPUT_PULLUP);

  //attach interrupt pins to reset count
  attachInterrupt(digitalPinToInterrupt(MOTOR1_LIMIT), reset1, FALLING);
  attachInterrupt(digitalPinToInterrupt(MOTOR2_LIMIT), reset2, FALLING);
  attachInterrupt(digitalPinToInterrupt(MOTOR3_LIMIT), reset3, FALLING);
  attachInterrupt(digitalPinToInterrupt(MOTOR4_LIMIT), reset4, FALLING);
}

void loop() {
  // Read the state of the buttons
  //readSensorData(&actualAngleRoll, &actualAnglePitch);
  button1State = digitalRead(BUTTON1_PINB);
  button2State = digitalRead(BUTTON2_PINRR);

  //button1_blueButton, casters retract until limit-switches is pressed
  // If button 1 is pressed and was not pressed before, move all the motors
  if (button1State == LOW && lastButton1State == HIGH) {
    // Move all the motors until they hit their limit switches
    while (digitalRead(MOTOR1_LIMIT) == HIGH || digitalRead(MOTOR2_LIMIT) == HIGH || digitalRead(MOTOR3_LIMIT) == HIGH || digitalRead(MOTOR4_LIMIT) == HIGH) {
      if (digitalRead(MOTOR1_LIMIT) == HIGH) {
        motor1.step(-1 * motorDirectionValue);
      }
      if (digitalRead(MOTOR2_LIMIT) == HIGH) {
        motor2.step(-1 * motorDirectionValue);
      }
      if (digitalRead(MOTOR3_LIMIT) == HIGH) {
        motor3.step(-1 * motorDirectionValue);
      }
      if (digitalRead(MOTOR4_LIMIT) == HIGH) {
        motor4.step(-1 * motorDirectionValue);
      }
    }
  }
  
  //button2red_button, casters extend first, then balance
  // If button 2 is pressed and was not pressed before, move all the motors in the other direction for 5 seconds
  if (button2State == LOW && lastButton2State == HIGH)
  {
    if(counter1 == 0 && counter2 == 0 && counter3 == 0 && counter4 == 0)
    {
      raisePlatform();
    }
    //delay(100);
    highestLeg = highestPositionMotorFixedLeg();
    
    if(highestLeg == 1)
    {
      correctMotor1();
    }
    else if(highestLeg == 2)
    {
      correctMotor2();
    }
    else if(highestLeg == 3)
    {
      correctMotor3();
    }
    else if(highestLeg == 4)
    {
      correctMotor4();
    }

    //Test code here, may delete below
    Serial.println("We are calculating the highestLeg again, 2nd iteration: ");
    //delay(100);
    highestLeg = highestPositionMotorFixedLeg();
    
    if(highestLeg == 1)
    {
      correctMotor1();
    }
    else if(highestLeg == 2)
    {
      correctMotor2();
    }
    else if(highestLeg == 3)
    {
      correctMotor3();
    }
    else if(highestLeg == 4)
    {
      correctMotor4();
    }
    
  }
}

void raisePlatform()
{
  int i = 0;
  //if all limit-switches are pressed -> step 2972 times
  if(counter1 == 0 && counter2 == 0 && counter3 == 0 && counter4 == 0)
  {
    Serial.print("Counter1 value before: ");
    Serial.println(counter1);
    //for(i = 0; i < 12000; i++)  //This is for prototype
    for(i = 0; i < 20000; i++)    //actual platform
    {
      motor1.step(motorDirectionValue);
      motor2.step(motorDirectionValue);
      motor3.step(motorDirectionValue);
      motor4.step(motorDirectionValue);
    }
    counter1 = i;
    counter2 = i;
    counter3 = i;
    counter4 = i;
  Serial.print("Counter1 value: ");
  Serial.println(counter1);
  }
}

int highestPositionMotorFixedLeg()  //returns 1,2,3, or 4, indicating highest stepper caster
{
  //readSensorData(&actualAngleRoll, &actualAnglePitch);
  //Get the average roll and average pitch values for 100 values
  getHundredValues();

  //Pass arrays into average function here
  averageForRoll = getAverage(rollArray);
  averageForRoll = averageForRoll -1.094;
  Serial.print("Average value for Roll: ");
  Serial.println(averageForRoll);

  averageForPitch = getAverage(pitchArray);
  averageForPitch = averageForPitch + 0.9;
  Serial.print("Average value for Pitch: ");
  Serial.println(averageForPitch);

  if(averageForRoll > 0 && averageForPitch < 0) //Motor 1: Positive Roll/ Neg Pitch
  {
    return 1;
  }
  else if(averageForRoll < 0 && averageForPitch < 0)  //Motor2: Negative / Neg
  {
    return 2;
  }
  else if(averageForRoll < 0 && averageForPitch > 0)  //Motor3: Neg / Pos
  {
    return 3;
  }
  else if(averageForRoll > 0 && averageForPitch > 0)  //Motor4: Pos / Pos
  {
    return 4;
  }
  else
  {
    return 1;
  }


}

void correctMotor1()    //Motor 1: Positive Roll/ Neg Pitch
{
  //Serial.println("inside 1");
  uint32_t longLift = 0;
  uint32_t shortLift = 0;
  uint32_t diagLift = 0;
  /*
  uint32_t  inchLift = 11700;
  float shortLength = 9.3125;
  float longLength = 15.374;
  */
  uint32_t  inchLift = 31546;
  float shortLength = 28.5;
  float longLength = 41;
  //int signSwitch = -1;
  float averageForRollDegToRad = 0;
  float averageForPitchDegToRad = 0;

  averageForRollDegToRad = averageForRoll * (3.14159 / 180);
  averageForPitchDegToRad = averageForPitch * (3.14159 / 180);

  shortLift = inchLift * shortLength * tan(abs(averageForRollDegToRad));
  longLift = inchLift * longLength * tan(abs(averageForPitchDegToRad));
  diagLift = shortLift + longLift;

 if(diagLift > maxStepsMotor){}
 else{
  for(uint32_t i = 0; i < diagLift; i++)
  {
    if(i < shortLift)
    {
      motor2.step(motorDirectionValue);  //Correct Motor 2 (shortLift)
    }
    if(i < longLift)
    {
      motor4.step(motorDirectionValue);              //Correct Motor 4 (LongLift)
    }
    motor3.step(motorDirectionValue);  //Correct Motor 3 (diagonalLift)
  }
 }


  //The code below is for testing purposes (will delete)
  getHundredValues();
  //Pass arrays into average function here
  averageForRoll = getAverage(rollArray);
  averageForRoll = averageForRoll -1.094;
  //Serial.print("Average value for Roll (after correction): ");
  //Serial.println(averageForRoll);

  averageForPitch = getAverage(pitchArray);
  averageForPitch = averageForPitch + 0.9;
  //Serial.print("Average value for Pitch(after correction): ");
  //Serial.println(averageForPitch);

}

void correctMotor2()    //Motor2: Negative / Neg
{
  //Serial.println("inside 2");
  uint32_t longLift = 0;
  uint32_t shortLift = 0;
  uint32_t diagLift = 0;
  /*
  uint32_t  inchLift = 11700;
  float shortLength = 9.3125;
  float longLength = 15.374;
  */
  uint32_t  inchLift = 31546;
  float shortLength = 28.5;
  float longLength = 41;
  //int signSwitch = -1;
  float averageForRollDegToRad = 0;
  float averageForPitchDegToRad = 0;

  averageForRollDegToRad = averageForRoll * (3.14159 / 180);
  averageForPitchDegToRad = averageForPitch * (3.14159 / 180);
  shortLift = inchLift * shortLength * tan(abs(averageForRollDegToRad));
  longLift = inchLift * longLength * tan(abs(averageForPitchDegToRad));
  diagLift = shortLift + longLift;
  /*
  Serial.print("tan(abs(averageForRollDegToRad): ");
  Serial.println(tan(abs(averageForRollDegToRad)));
  Serial.print("\ttan(abs(averageForPitchDegToRad): ");
  Serial.println(tan(abs(averageForPitchDegToRad)));
  Serial.print("shortLift: ");
  Serial.print(shortLift);
  Serial.print("\tlongLift: ");
  Serial.print(longLift);
  Serial.print("\tDiagonalLift: ");
  Serial.println(diagLift);
  */
  //Note, if value is negative, motor raises that platform, so value should
  //be positive
  /* 
  motor1.step(signSwitch * shortLift);  //Correct Motor 1 (shortLift)
  motor3.step(signSwitch * longLift);  //Correct Motor 3 (LongLift)
  motor4.step(signSwitch * diagLift);  //Correct Motor 4 (diagonalLift)
  */
 if(diagLift > maxStepsMotor){}
 else
 {
    for(uint32_t i = 0; i < diagLift; i++)
    {
     if(i < shortLift)
      {
        motor1.step(motorDirectionValue);  //Correct Motor 1 (shortLift)
      }
      if(i < longLift)
      {
        motor3.step(motorDirectionValue);              //Correct Motor 3 (LongLift)
     }
      motor4.step(motorDirectionValue);  //Correct Motor 4 (diagonalLift)
    }
 }


  //The code below is for testing purposes (will delete)
  getHundredValues();
  //Pass arrays into average function here
  averageForRoll = getAverage(rollArray);
  averageForRoll = averageForRoll -1.094;
  //Serial.print("Average value for Roll (after correction): ");
  //Serial.println(averageForRoll);

  averageForPitch = getAverage(pitchArray);
  averageForPitch = averageForPitch + 0.9;
  //Serial.print("Average value for Pitch(after correction): ");
  //Serial.println(averageForPitch);
}

void correctMotor3()    //Motor3: Neg / Pos
{
  //Serial.println("inside 3");
  uint32_t longLift = 0;
  uint32_t shortLift = 0;
  uint32_t diagLift = 0;
  /*
  uint32_t  inchLift = 11700;
  float shortLength = 9.3125;
  float longLength = 15.374;
  */
  uint32_t  inchLift = 31546;
  float shortLength = 28.5;
  float longLength = 41;
  //int signSwitch = -1;
  float averageForRollDegToRad = 0;
  float averageForPitchDegToRad = 0;

  averageForRollDegToRad = averageForRoll * (3.14159 / 180);
  averageForPitchDegToRad = averageForPitch * (3.14159 / 180);
  shortLift = inchLift * shortLength * tan(abs(averageForRollDegToRad));
  longLift = inchLift * longLength * tan(abs(averageForPitchDegToRad));
  diagLift = shortLift + longLift;
  /*
  Serial.print("tan(abs(averageForRollDegToRad): ");
  Serial.println(tan(abs(averageForRollDegToRad)));
  Serial.print("\ttan(abs(averageForPitchDegToRad): ");
  Serial.println(tan(abs(averageForPitchDegToRad)));
  */
  //Note, if value is negative, motor raises that platform, so value should
  //be positive 
  /*
  motor4.step(signSwitch * shortLift);  //Correct Motor 4 (shortLift)
  motor2.step(signSwitch * longLift);  //Correct Motor 2 (LongLift)
  motor1.step(signSwitch * diagLift);  //Correct Motor 1 (diagonalLift)
  */
 
 if(diagLift > maxStepsMotor){}
 else
 {
    for(uint32_t i = 0; i < diagLift; i++)
   {
     if(i < shortLift)
     {
       motor4.step(motorDirectionValue);  //Correct Motor 4 (shortLift)
     }
     if(i < longLift)
     {
       motor2.step(motorDirectionValue);              //Correct Motor 2 (LongLift)
      }
      motor1.step(motorDirectionValue);  //Correct Motor 1 (diagonalLift)
 }

  }

  //The code below is for testing purposes (will delete)
  getHundredValues();
  //Pass arrays into average function here
  averageForRoll = getAverage(rollArray);
  averageForRoll = averageForRoll -1.094;
  //Serial.print("Average value for Roll (after correction): ");
  //Serial.println(averageForRoll);

  averageForPitch = getAverage(pitchArray);
  averageForPitch = averageForPitch + 0.9;
  //Serial.print("Average value for Pitch(after correction): ");
  //Serial.println(averageForPitch);
}

void correctMotor4()    //Motor4: Pos / Pos
{
  //Serial.println("inside 4");
  uint32_t longLift = 0;
  uint32_t shortLift = 0;
  uint32_t diagLift = 0;
  /*
  uint32_t  inchLift = 11700;
  float shortLength = 9.3125;
  float longLength = 15.374;
  */
  uint32_t  inchLift = 31546;
  float shortLength = 28.5;
  float longLength = 41;
  //int signSwitch = -1;
  float averageForRollDegToRad = 0;
  float averageForPitchDegToRad = 0;

  averageForRollDegToRad = averageForRoll * (3.14159 / 180);
  averageForPitchDegToRad = averageForPitch * (3.14159 / 180);
  shortLift = inchLift * shortLength * tan(abs(averageForRollDegToRad));
  longLift = inchLift * longLength * tan(abs(averageForPitchDegToRad));
  diagLift = shortLift + longLift;
  /*
  Serial.print("tan(abs(averageForRollDegToRad): ");
  Serial.println(tan(abs(averageForRollDegToRad)));
  Serial.print("\ttan(abs(averageForPitchDegToRad): ");
  Serial.println(tan(abs(averageForPitchDegToRad)));
  */
  //Note, if value is negative, motor raises that platform, so value should
  //be positive 
  /*
  motor3.step(signSwitch * shortLift);  //Correct Motor 3 (shortLift)
  motor1.step(signSwitch * longLift);  //Correct Motor 1 (LongLift)
  motor2.step(signSwitch * diagLift);  //Correct Motor 2 (diagonalLift)
  */
  if(diagLift > maxStepsMotor){}
  else
  {
    for(uint32_t i = 0; i < diagLift; i++)
   {
     if(i < shortLift)
     {
       motor3.step(motorDirectionValue);  //Correct Motor 3 (shortLift)
     }
      if(i < longLift)
     {
       motor1.step(motorDirectionValue);              //Correct Motor 1 (LongLift)
     }
     motor2.step(motorDirectionValue);  //Correct Motor 2 (diagonalLift)
   }
  }


  //The code below is for testing purposes (will delete)
  getHundredValues();
  //Pass arrays into average function here
  averageForRoll = getAverage(rollArray);
  averageForRoll = averageForRoll -1.094;
  //Serial.print("Average value for Roll (after correction): ");
  //Serial.println(averageForRoll);

  averageForPitch = getAverage(pitchArray);
  averageForPitch = averageForPitch + 0.9;
  //Serial.print("Average value for Pitch(after correction): ");
  //Serial.println(averageForPitch);
}

void reset1()
{
  counter1 = 0;
  //Serial.print("counter1 value: ");
}

void reset2()
{
  counter2 = 0;
}

void reset3()
{
  counter3 = 0;
}

void reset4()
{
  counter4 = 0;
}

//*************************************************************

//Add I2C functions
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) {
  return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}

uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) {
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  Wire.write(data, length);
  uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
  if (rcode) {
    Serial.print(F("i2cWrite failed: "));
    Serial.println(rcode);
  }
  return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}

uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) {
  uint32_t timeOutTimer;
  Wire.beginTransmission(IMUAddress);
  Wire.write(registerAddress);
  uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
  if (rcode) {
    Serial.print(F("i2cRead failed: "));
    Serial.println(rcode);
    return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
  }
  Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
  for (uint8_t i = 0; i < nbytes; i++) {
    if (Wire.available())
      data[i] = Wire.read();
    else {
      timeOutTimer = micros();
      while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
      if (Wire.available())
        data[i] = Wire.read();
      else {
        Serial.println(F("i2cRead timeout"));
        return 5; // This error value is not already taken by endTransmission
      }
    }
  }
  return 0; // Success
}

void readSensorData(double *myRoll, double *myPitch)
{
/* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (i2cData[6] << 8) | i2cData[7];
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG; 
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

  *myRoll = roll;
  *myPitch = pitch; 

}

void getHundredValues()
{
  for(int i = 0; i < 100; i ++)
  {
    readSensorData(&actualAngleRoll, &actualAnglePitch);
    rollArray[i] = actualAngleRoll;
    pitchArray[i] = actualAnglePitch;
  }
}

double getAverage(float myArray[100])
{
  double sum = 0;

  for(int i = 0; i < 100; i ++)
  {
    sum += myArray[i];
  }
  return sum / 100;
}
