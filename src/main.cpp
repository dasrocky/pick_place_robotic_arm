/*
 Name:		MCS_V3_20190619.ino
 Created:	19-Jun-19 1:27:37 AM
 Author:	rocky
 Team: MechTechAutomation
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Pinouts
////* Motor Driver pinouts
const int input1MotorDriver = 2;
const int input2MotorDriver = 10;
const int enable1MotorDriver = 3;
////* Ultrasonic sensor pinouts
const int triggerPin = 5;
const int echoPin = 4;
////* color sensor pinout
const int s0 = 11;
const int s1 = 6;
const int s2 = 7;
const int s3 = 12;
const int output = 13;

// Variables
int frequencyRed = 0;
int frequencyBlue = 0;
int frequencyGreen = 0;
// int css = 0;
long int ultraSonicduration = 0, distanceInCM = 0;
int getColorSensorState = 0;
int systemState = 1;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

uint8_t slaveServoMotor = 0; //? Servo motor -> Base - 1, elbow - 2, wrist - 3 , claw - 4
const struct RoboticArm
{
  uint8_t base = 0;
  uint8_t shoulder = 1;
  uint8_t elbow = 2;
  uint8_t wrist = 3;
  uint8_t claw = 4;
}roboticArmAnatomy;

uint16_t pulselength;
int timeDelay2 = 30, armTimeDelay = 500;
int roboticArmState = 0, ex_state = 1;

// Postion Controller_Arm
int sp1h = 460;
int sp1l = 315;
int sp2h = 630;
int sp2l = 520;
int sp3h = 295;
int sp3l = 225;
int sp2l_r = 380;
int sp1l_r = 400;
int sp0cw = 520;
int sp0ccw = 120;
int sp0ccw_r = 320;

//* Function Prototypes
////* arm
void resetArm();
void downPick();
void upPick();
void rotateClockwiseRed();
void downDropRed();
void upDropRed();
void rotateCounterClockwise();
void resetArm();
void rotateClockwiseBlue();
void downDropBlue();
void upDropBlue();
////* color sensor
int colorSensor();
long getDistanceFromUltrasonic();
////* utils
void millisDelay(int time);

// the setup function runs once when you press reset or power the board
void setup()
{
  Serial.begin(9600);
  while (!Serial)
    ;

  pinMode(input1MotorDriver, OUTPUT);
  pinMode(input2MotorDriver, OUTPUT);
  pinMode(s0, OUTPUT);
  pinMode(s1, OUTPUT);
  pinMode(s2, OUTPUT);
  pinMode(s3, OUTPUT);
  pinMode(triggerPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(output, INPUT);

  digitalWrite(s0, HIGH);
  digitalWrite(s1, HIGH);

  pwm.begin();
  pwm.setPWMFreq(60);
  delay(15);
  delay(2000);
  resetArm();
  delay(armTimeDelay);
}

// the loop function runs over and over again until power down or reset
void loop()
{
  Serial.print("State: ");
  Serial.println(systemState);
  Serial.println();
  if (systemState == 1)
  {
    digitalWrite(input1MotorDriver, HIGH);
    digitalWrite(input2MotorDriver, LOW);
    analogWrite(enable1MotorDriver, 120);
  }
  Serial.println("Ultransonic Sensor active!!");
  distanceInCM = getDistanceFromUltrasonic();
  if (distanceInCM <= 4)
  {
    analogWrite(enable1MotorDriver, 0);
    delay(100);
    getColorSensorState = colorSensor();
    analogWrite(enable1MotorDriver, 120);
    delay(900); // Delay for Box
    analogWrite(enable1MotorDriver, 0);
    delay(500);
    systemState = 0;
    if (getColorSensorState == 1)
    {
      // Initializing Red Sequences
      downPick();
      delay(armTimeDelay);
      upPick();
      delay(armTimeDelay);
      rotateClockwiseRed();
      delay(armTimeDelay);
      downDropRed();
      delay(armTimeDelay);
      upDropRed();
      delay(armTimeDelay);
      rotateCounterClockwise();
      delay(armTimeDelay);
      resetArm();
      delay(armTimeDelay);
    }
    if (getColorSensorState == 2)
    {
      // Initializing Blue Sequences
      downPick();
      delay(armTimeDelay);
      upPick();
      delay(armTimeDelay);
      rotateClockwiseBlue();
      delay(armTimeDelay);
      downDropBlue();
      delay(armTimeDelay);
      upDropBlue();
      delay(armTimeDelay);
      rotateCounterClockwise();
      delay(armTimeDelay);
      resetArm();
      delay(armTimeDelay);
    }
  }
  else
  {
    systemState = 1;
  }
}

// Funtions
// Colour_Sensor
int colorSensor()
{
  Serial.println("Colour Sensor");
  Serial.println();
  int colorSensorState = 0;
  digitalWrite(s2, LOW); // basically notice that i put for this and the next low, coz this make the sensor take the input for red for that wire + am using red color
  digitalWrite(s3, LOW);
  frequencyRed = pulseIn(output, LOW);
  delay(500);
  digitalWrite(s2, LOW);  // basically notice that i put for this and the next low then high, coz this make the sensor take the input for blue for that wire + am using blue color
  digitalWrite(s3, HIGH); // it is set in binary 0 low 1 hhigh
  frequencyBlue = pulseIn(output, LOW);
  delay(500);
  digitalWrite(s2, HIGH); // basically notice that i put for this and the next high, coz this make the sensor take the input for green for that wire + am using red color
  digitalWrite(s3, HIGH);
  frequencyGreen = pulseIn(output, LOW);
  delay(500);
  if ((frequencyRed < frequencyBlue) && (frequencyRed < frequencyGreen))
  {
    colorSensorState = 1;
    Serial.println("Red");
  }
  if ((frequencyBlue < frequencyRed) && (frequencyBlue < frequencyGreen))
  {
    colorSensorState = 2;
    Serial.println("Blue");
  }
  return colorSensorState;
}

// Ultrasonic_Sensor
long getDistanceFromUltrasonic()
{
  long DistanceInCM = 0;
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);
  ultraSonicduration = pulseIn(echoPin, HIGH);
  DistanceInCM = ultraSonicduration / 74 / 2;
  Serial.print("Ultrasonic: ");
  Serial.print(DistanceInCM);
  Serial.println(" cm");
  Serial.println();
  delay(100);
  return DistanceInCM;
}

// Robotic_Arm
void resetArm()
{
  Serial.println("Reset Function");
  if (ex_state == 1)
  {
    slaveServoMotor = 0;
    pwm.setPWM(roboticArmAnatomy.base, 0, sp0cw); // Rotating base
    delay(timeDelay2);
  }
  ex_state++;
  slaveServoMotor = 1;
  pwm.setPWM(roboticArmAnatomy.shoulder, 0, sp1h); // Shoulder
  slaveServoMotor = 2;
  pwm.setPWM(roboticArmAnatomy.elbow, 0, sp2h); // Elbow
  delay(timeDelay2);
  slaveServoMotor = 4;
  pwm.setPWM(roboticArmAnatomy.claw, 0, sp3h); // Claw
  delay(timeDelay2);
  Serial.println("Exit reset");
}
void downPick()
{
  Serial.println("Down_p fucntion");
  // Rotating base moved to picking position
  roboticArmState++;
  if (roboticArmState == 1)
  {
    slaveServoMotor = 0;
    pwm.setPWM(slaveServoMotor, 0, sp0cw); // Move to picking position
    delay(timeDelay2);
  }
  // Shoulder
  for (pulselength = sp1h; pulselength >= sp1l; pulselength--)
  {
    slaveServoMotor = 1;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  // claw
  for (pulselength = sp3h; pulselength >= sp3l; pulselength--)
  {
    slaveServoMotor = 4;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  // Elbow
  for (pulselength = sp2h; pulselength >= sp2l; pulselength--)
  {
    slaveServoMotor = 2;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  Serial.println("Exit down_p");
}
void upPick()
{
  Serial.println("Up_pick function");
  // Claw
  for (pulselength = sp3l; pulselength <= sp3h; pulselength++) // or 295
  {
    slaveServoMotor = 4;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  // Shoulder
  for (pulselength = sp1l; pulselength <= sp1h; pulselength++)
  {
    slaveServoMotor = 1;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  // Elbow
  for (pulselength = sp2l; pulselength <= sp2h; pulselength++)
  {
    slaveServoMotor = 2;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  Serial.println("Exit up_p");
}
void rotateClockwiseBlue()
{
  Serial.println("rotateClockwiseBlue");
  for (pulselength = sp0cw; pulselength >= sp0ccw; pulselength--)
  {
    slaveServoMotor = 0;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  Serial.println("Exit CW");
}
void rotateCounterClockwise()
{
  Serial.println("Rotate CCW");
  for (pulselength = sp0ccw; pulselength <= sp0cw; pulselength++)
  {
    slaveServoMotor = 0;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  Serial.println("Exit CCW");
}
void downDropBlue()
{
  Serial.println("Down_drop");
  // Shoulder
  for (pulselength = sp1h; pulselength >= sp1l; pulselength--) // before it was 260
  {
    slaveServoMotor = 1;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  // Elbow
  for (pulselength = sp2h; pulselength >= sp2l; pulselength--) // before it was 195
  {
    slaveServoMotor = 2;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  // Claw
  for (pulselength = sp3h; pulselength >= sp3l; pulselength--)
  {
    slaveServoMotor = 4;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  Serial.println("Exit down_d");
}
void upDropBlue()
{
  Serial.println("up_d");
  // Shoulder
  for (pulselength = sp1l; pulselength <= sp1h; pulselength++) // 260
  {
    slaveServoMotor = 1;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  // Elbow
  for (pulselength = sp2l; pulselength <= sp2h; pulselength++) // 195
  {
    slaveServoMotor = 2;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  // Claw
  for (pulselength = sp3l; pulselength <= sp3h; pulselength++)
  {
    slaveServoMotor = 4;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  Serial.println("Exit up_d");
}
void rotateClockwiseRed()
{
  Serial.println("rotateClockwiseBlue for red");
  for (pulselength = sp0cw; pulselength >= sp0ccw_r; pulselength--)
  {
    slaveServoMotor = 0;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  Serial.println("Exit CW for red");
}
void downDropRed()
{
  Serial.println("Down_drop");
  // Shoulder
  for (pulselength = sp1h; pulselength >= sp1l_r; pulselength--) // 290
  {
    slaveServoMotor = 1;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  // Elbow
  for (pulselength = sp2h; pulselength >= sp2l_r; pulselength--) // 195
  {
    slaveServoMotor = 2;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  // Claw
  for (pulselength = sp3h; pulselength >= sp3l; pulselength--)
  {
    slaveServoMotor = 4;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  Serial.println("Exit down_d");
}
void upDropRed()
{
  Serial.println("up_d");
  // Shoulder
  for (pulselength = sp1l_r; pulselength <= sp1h; pulselength++) // 290
  {
    slaveServoMotor = 1;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  // Elbow
  for (pulselength = sp2l_r; pulselength <= sp2h; pulselength++) // 195
  {
    slaveServoMotor = 2;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  // Claw
  for (pulselength = sp3l; pulselength <= sp3h; pulselength++)
  {
    slaveServoMotor = 4;
    pwm.setPWM(slaveServoMotor, 0, pulselength);
    delay(timeDelay2);
  }
  Serial.println("Exit up_d");
}
