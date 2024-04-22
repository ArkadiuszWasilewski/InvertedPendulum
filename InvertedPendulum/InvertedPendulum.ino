/*
 Name:		InvertedPendulum.ino
 Created:	4/11/2024 2:31:26 PM
 Author:	wasil
*/

#include <PID_v1.h>
#include <Encoder.h>
#include "pendulum.h"
#include "Motor.h"


int flagReady;

long currT;
long prevT = 0;
float deltaT;

float impulseLength = 0.041962;  // ~=(308 [mm]) / 7340
float impulseLength_rot = 0.15;

float v1 = 0;
float v1Prev = 0;
float v1Filt = 0;
int pos = 0;
int posPrev;
double pos_mm = 0;

float v1_rot = 0;
float v1Prev_rot = 0;
float v1Filt_rot = 0;
int pos_rot = 0;
float pos_rot_degree = 0;
int posPrev_rot;

static volatile float receivedPset;
static volatile float receivedKp;
static volatile float receivedKi;
static volatile float receivedKd;

Encoder linearEncoder(pinA, pinB);
Encoder rotaryEncoder(pinA_angular, pinB_angular);
Motor motor(PWM, DIR);

// moveToPosition regulator
double Input, Output, linearSetPoint;
double Kp = 3, Ki = 0.5, Kd = 0;

double angle, Output_rot, rot_Setpoint;
double rot_Kp = 1, rot_Ki = 0, rot_Kd = 0;

double lin_position, lin_Setpoint = 2640;
double lin_Kp = 1, lin_Ki = 0, lin_Kd = 0;

double limit_angle = 150;
int limit_lin_min = LIMIT_LEFT_RIGHT;
int limit_lin_max = LIMIT_LEFT_RIGHT;

PID linearController(&Input, &Output, &linearSetPoint, Kp, Ki, Kd, DIRECT);

PID rotationController(&angle, &Output_rot, &rot_Setpoint, rot_Kp, rot_Ki, rot_Kd, DIRECT);
PID linearController_casc(&lin_position, &rot_Setpoint, &lin_Setpoint, lin_Kp, lin_Ki, lin_Kd, DIRECT);

float measureVelocity()
{
	currT = micros();
	deltaT = (float)(currT - prevT) / 1.0e6;


	// Compute linear speed of a carriage [mm/s]
	v1 = ((pos - posPrev) * impulseLength) / deltaT;
	posPrev = pos;
	prevT = currT;


	// Low-pass filter, 25Hz cutoff
	v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
	v1Prev = v1;

	return v1Filt; // filtered linear speed 
}

float measureRotSpeed()
{
	currT = micros();
	deltaT = (float)(currT - prevT) / 1.0e6;


	v1_rot = ((pos_rot - posPrev_rot)) * 0.15 / deltaT;
	posPrev_rot = pos_rot;
	prevT = currT;


	v1Filt_rot = 0.854 * v1Filt_rot + 0.0728 * v1_rot + 0.0728 * v1Prev_rot;
	v1Prev_rot = v1_rot;

	return v1Filt_rot;
}

void setup() {
	Serial.begin(BAUDRATE);
	
	// setting PWM frequency on pin 10 to 31kHz
	TCCR2B = (TCCR2B & 0b11111000) | 0x01;

	pinMode(end_switch, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(end_switch), limitSwitch, CHANGE);

	rotaryEncoder.write(-1200);
	motor.initializeMove(end_switch);
	linearEncoder.write(0);
	

	
	Input = pos_mm;
	linearController.SetMode(AUTOMATIC);
	linearController.SetOutputLimits(-155, 155);
	
}

void loop() {
	pos = linearEncoder.read();
	pos_rot = rotaryEncoder.read();

	pos_mm = pos * impulseLength;					//mm
	pos_rot_degree = pos_rot * impulseLength_rot;	//degrees

	/*
	linearSetPoint = receivedPset;
	double Kp = receivedKp;
	double Ki = receivedKi;
	double Kd = receivedKd;
*/


	if (rotaryEncoder.read() >= MAX_ROTARY_IMPULSES || rotaryEncoder.read() <= -MAX_ROTARY_IMPULSES)
	{
		rotaryEncoder.write(0);
	}

	rotationController.SetMode(AUTOMATIC);
	rotationController.SetOutputLimits(-255, 255);
	rotationController.SetSampleTime(3); // ?? why
	linearController_casc.SetMode(AUTOMATIC);
	linearController_casc.SetOutputLimits(-255, 255);  // (-300, 300) ???
	linearController_casc.SetSampleTime(3);


	angle = pos_rot;
	lin_position = pos;

	lin_Kp = -0.02;	//0.0012750000   0.0012
	lin_Ki = 0;			//0			0
	lin_Kd = -0.000017;//-0.000001;			//0
	rot_Kp = 20;		//4.6799998283			20
	rot_Ki = 90;		//208.0000000000	100
	rot_Kd = 0;			//0

	
	if ((lin_position> limit_lin_min) && (lin_position< MAX_LINEAR_IMPULSES-limit_lin_max))
	{
		if (angle<limit_angle && angle>(limit_angle * -1))
		{
			linearController_casc.SetTunings(lin_Kp, lin_Ki, lin_Kd);
			linearController_casc.Compute();
			rotationController.SetTunings(rot_Kp, rot_Ki, rot_Kd);
			rotationController.Compute();
			if (fabs(Output_rot) > 50)
			{
				motor.moveToPosition(Output_rot);
			}
			else
			{
				motor.moveToPosition(Output_rot);
			}
			
		}
		else
		{
			//motor.moveToPosition(0);
		}
	}
	else if (lin_position <= limit_lin_min)
	{
		/*
		linearSetPoint = 1000;
		Input = pos;
		linearController.SetTunings(Kp, Ki, Kd);
		linearController.Compute();
		motor.moveToPosition(Output);
		delay(50);*/
		
	}
	else if (lin_position >= MAX_LINEAR_IMPULSES - limit_lin_max)
	{
		/*
		linearSetPoint = MAX_LINEAR_IMPULSES-1000;
		Input = pos;
		linearController.SetTunings(Kp, Ki, Kd);
		linearController.Compute();
		motor.moveToPosition(Output);
		delay(50);
		*/
	}

	

	//Buffer received
	/*static char buffer[BUFFER_SZ];
	static size_t lg = 0;

	
	while (Serial.available() > 0)
	{
		char c = Serial.read();
		if (c == '\r') {				// Carriage return
			buffer[lg] = '\0';			// Terminate the string
			parse(buffer);
			lg = 0;
		}
		else if (lg < BUFFER_SZ - 1) {
			buffer[lg++] = c;
		}

	}
	*/

	if (Serial.availableForWrite() > 0)
	{
		Serial.println(pos);
		//Serial.print(",");
		//Serial.println(measureVelocity());
	}
	/*
	//Serial.println(measureVelocity());
	/*Serial.print(',');
	Serial.print(pos_rot);
	Serial.print(',');
	Serial.print(measureRotSpeed());
	Serial.print(',');
	Serial.print(lin_Setpoint);
	Serial.print(',');
	Serial.println(rot_Setpoint);*/
	/*Serial.print(',');
	Serial.println(rot_Setpoint);*/

}

void parse(char* buffer)
{
	char* s = strtok(buffer, ",");
	receivedPset = atoi(s);
	s = strtok(NULL, ",");
	receivedKp = atoi(s);
	s = strtok(NULL, ",");
	receivedKi = atoi(s);
	s = strtok(NULL, ",");
	receivedKd = atoi(s);
	//s = strtok(NULL, ",");
	//receivedMethod = atoi(s);
}

void limitSwitch()
{
	if (bitRead(PIND, PD2) == LOW) // pin 19 on Arduino Mega
	{
		flagReady = 0;
	}
	if (bitRead(PIND, PD2) == HIGH)
	{
		flagReady = 1;
	}
	analogWrite(PWM, 0);
}