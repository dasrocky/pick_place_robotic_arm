/*
 Name:		MCS_V3_20190619.ino
 Created:	19-Jun-19 1:27:37 AM
 Author:	rocky
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Pinouts
const int in1 = 2;
const int in2 = 10;
const int en1 = 3;

const int trig = 5;
const int echo = 4;

const int s0 = 11;
const int s1 = 6;
const int s2 = 7;
const int s3 = 12;
const int output = 13;

// Variables
int freq_r = 0;
int freq_g = 0;
int freq_b = 0;
int css = 0;
long int duration = 0, cm = 0;
int cs_stat = 0;
int state = 1;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

uint8_t s = 0;
uint16_t pulselen;
int d = 30, t = 500;
int rb_state = 0, ex_state = 1;

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



// the setup function runs once when you press reset or power the board
void setup() 
{
	Serial.begin(9600);

	pinMode(in1, OUTPUT);
	pinMode(in2, OUTPUT);
	pinMode(s0, OUTPUT);
	pinMode(s1, OUTPUT);
	pinMode(s2, OUTPUT);
	pinMode(s3, OUTPUT);
	pinMode(trig, OUTPUT);
	pinMode(echo, INPUT);

	pinMode(output, INPUT);

	digitalWrite(s0, HIGH);
	digitalWrite(s1, HIGH);

	pwm.begin();
	pwm.setPWMFreq(60);
	delay(15);
	delay(2000);
	reset_arm();
	delay(t);
}

// the loop function runs over and over again until power down or reset
void loop() 
{
	Serial.print("State: "); Serial.println(state); Serial.println();
	if (state == 1)
	{
		digitalWrite(in1, HIGH);
		digitalWrite(in2, LOW);
		analogWrite(en1, 120);
	}
	digitalWrite(trig, LOW);
	delayMicroseconds(2);
	digitalWrite(trig, HIGH);
	delayMicroseconds(10);
	digitalWrite(trig, LOW);
	duration = pulseIn(echo, HIGH);
	cm = duration / 74 / 2;
	Serial.print("Ultrasonic: "); Serial.print(cm); Serial.println(" cm"); Serial.println();
	delay(100);
	if (cm <= 4)
	{
		analogWrite(en1, 0);
		delay(100);
		cs_stat = clrsensor();
		analogWrite(en1, 120);
		delay(900);     // Delay for Box
		analogWrite(en1, 0);
		delay(500);
		state = 0;

		if (cs_stat == 1)
		{
			// Initializing Red Sequences
			down_pick();
			delay(t);
			up_pick();
			delay(t);
			rot_cw_c();
			delay(t);
			down_drop_red();
			delay(t);
			up_drop_red();
			delay(t);
			rot_ccw();
			delay(t);
			reset_arm();
			delay(t);
		}
		if (cs_stat == 2)
		{
			// Initializing Blue Sequences
			down_pick();
			delay(t);
			up_pick();
			delay(t);
			rot_cw();
			delay(t);
			down_drop();
			delay(t);
			up_drop();
			delay(t);
			rot_ccw();
			delay(t);
			reset_arm();
			delay(t);
		}
	}
	else
	{
		state = 1;
	}
}

// Funtions
// Colour_Sensor
int clrsensor()
{
	Serial.println("Colour Sensor"); Serial.println();
	int cs_s = 0;
	digitalWrite(s2, LOW);  //basically notice that i put for this and the next low, coz this make the sensor take the input for red for that wire + am using red color
	digitalWrite(s3, LOW);
	freq_r = pulseIn(output, LOW);
	delay(500);
	digitalWrite(s2, LOW);  //basically notice that i put for this and the next low then high, coz this make the sensor take the input for blue for that wire + am using blue color
	digitalWrite(s3, HIGH);//it is set in binary 0 low 1 hhigh
	freq_b = pulseIn(output, LOW);
	delay(500);
	digitalWrite(s2, HIGH);//basically notice that i put for this and the next high, coz this make the sensor take the input for green for that wire + am using red color
	digitalWrite(s3, HIGH);
	freq_g = pulseIn(output, LOW);
	delay(500);
	if ((freq_r < freq_b) && (freq_r < freq_g))
	{
		cs_s = 1;
		Serial.println("Red");
	}
	if ((freq_b < freq_r) && (freq_b < freq_g))
	{
		cs_s = 2;
		Serial.println("Blue");
	}
	return cs_s;
}

// Robotic_Arm
void reset_arm()
{
	Serial.println("Reset Function");
	if (ex_state == 1)
	{
		s = 0;
		pwm.setPWM(s, 0, sp0cw);      // Rotating base
		delay(d);
	}
	ex_state++;
	s = 1;
	pwm.setPWM(s, 0, sp1h);      // Shoulder
	s = 2;
	pwm.setPWM(s, 0, sp2h);      // Elbow
	delay(d);
	s = 4;
	pwm.setPWM(s, 0, sp3h);      // Claw
	delay(d);
	Serial.println("Exit reset");
}
void down_pick()
{
	Serial.println("Down_p fucntion");
	// Rotating base moved to picking position
	rb_state++;
	if (rb_state == 1)
	{
		s = 0;
		pwm.setPWM(s, 0, sp0cw);    // Move to picking position
		delay(d);
	}
	// Shoulder
	for (pulselen = sp1h; pulselen >= sp1l; pulselen--)
	{
		s = 1;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	// claw
	for (pulselen = sp3h; pulselen >= sp3l; pulselen--)
	{
		s = 4;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	// Elbow
	for (pulselen = sp2h; pulselen >= sp2l; pulselen--)
	{
		s = 2;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	Serial.println("Exit down_p");
}
void up_pick()
{
	Serial.println("Up_pick function");
	// Claw
	for (pulselen = sp3l; pulselen <= sp3h; pulselen++)  // or 295
	{
		s = 4;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	// Shoulder
	for (pulselen = sp1l; pulselen <= sp1h; pulselen++)
	{
		s = 1;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	// Elbow
	for (pulselen = sp2l; pulselen <= sp2h; pulselen++)
	{
		s = 2;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	Serial.println("Exit up_p");
}
void rot_cw()
{
	Serial.println("rot_cw");
	for (pulselen = sp0cw; pulselen >= sp0ccw; pulselen--)
	{
		s = 0;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	Serial.println("Exit CW");
}
void rot_ccw()
{
	Serial.println("Rotate CCW");
	for (pulselen = sp0ccw; pulselen <= sp0cw; pulselen++)
	{
		s = 0;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	Serial.println("Exit CCW");
}
void down_drop()
{
	Serial.println("Down_drop");
	// Shoulder
	for (pulselen = sp1h; pulselen >= sp1l; pulselen--)  // before it was 260
	{
		s = 1;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	// Elbow
	for (pulselen = sp2h; pulselen >= sp2l; pulselen--)	// before it was 195
	{
		s = 2;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	// Claw
	for (pulselen = sp3h; pulselen >= sp3l; pulselen--)
	{
		s = 4;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	Serial.println("Exit down_d");
}
void up_drop()
{
	Serial.println("up_d");
	// Shoulder
	for (pulselen = sp1l; pulselen <= sp1h; pulselen++)	// 260
	{
		s = 1;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	// Elbow
	for (pulselen = sp2l; pulselen <= sp2h; pulselen++)	// 195
	{
		s = 2;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	// Claw
	for (pulselen = sp3l; pulselen <= sp3h; pulselen++)
	{
		s = 4;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	Serial.println("Exit up_d");
}
void rot_cw_c()
{
	Serial.println("rot_cw for red");
	for (pulselen = sp0cw; pulselen >= sp0ccw_r; pulselen--)
	{
		s = 0;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	Serial.println("Exit CW for red");
}
void down_drop_red()
{
	Serial.println("Down_drop");
	// Shoulder
	for (pulselen = sp1h; pulselen >= sp1l_r; pulselen--) //290
	{
		s = 1;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	// Elbow
	for (pulselen = sp2h; pulselen >= sp2l_r; pulselen--)	// 195
	{
		s = 2;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	// Claw
	for (pulselen = sp3h; pulselen >= sp3l; pulselen--)
	{
		s = 4;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	Serial.println("Exit down_d");
}
void up_drop_red()
{
	Serial.println("up_d");
	// Shoulder
	for (pulselen = sp1l_r; pulselen <= sp1h; pulselen++)	// 290
	{
		s = 1;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	// Elbow
	for (pulselen = sp2l_r; pulselen <= sp2h; pulselen++)	// 195
	{
		s = 2;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	// Claw
	for (pulselen = sp3l; pulselen <= sp3h; pulselen++)
	{
		s = 4;
		pwm.setPWM(s, 0, pulselen);
		delay(d);
	}
	Serial.println("Exit up_d");
}
