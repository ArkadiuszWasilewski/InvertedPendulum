#include "Motor.h"


Motor::Motor(int pinPWM, int pinDIR)
{
	_pinPWM = pinPWM;
	_pinDIR = pinDIR;
}

Motor::~Motor()
{
	//?
}

void Motor::motorSetup()
{
	pinMode(_pinPWM, OUTPUT);
	pinMode(_pinDIR, OUTPUT);
}

void Motor::startMotor(int valDIR, int valPWM, int pinPWM)
{
	analogWrite(pinPWM, valPWM);
	if (valDIR == 1)
	{
		digitalWrite(_pinDIR, HIGH);
	}
	else if (valDIR == -1)
	{
		digitalWrite(_pinDIR, LOW);
	}
	else
	{
		digitalWrite(_pinDIR, LOW);
		analogWrite(_pinPWM, 0);
	}
}

void Motor::moveLeft(int valPWM)
{
	Motor::startMotor(MOVE_LEFT, valPWM, _pinPWM);
}

void Motor::moveRight(int valPWM)
{
	Motor::startMotor(MOVE_RIGHT, valPWM, _pinPWM);
}

void Motor::initializeMove(int pinLimitSwitch)
{
	do
	{
		Motor::moveLeft(INITIALIZE_SPEED);
		delay(10);
	} while (digitalRead(end_switch) == HIGH);
	do
	{
		Motor::moveRight(INITIALIZE_SPEED);
		delay(10);
	} while (digitalRead(end_switch) == LOW);	
}

void Motor::moveToPosition(double u)
{
	if (u < 0)
	{
		dir = -1;
	}
	else if (u > 0)
	{
		dir = 1;
	}

	Motor::startMotor(dir, (int)fabs(u), _pinPWM);
}