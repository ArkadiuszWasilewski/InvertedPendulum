#pragma once
#include "pendulum.h"
#include <Arduino.h>

class Motor
{
public:
	Motor(int, int); //pinPWM, pinDIR
	~Motor();

	void motorSetup();					// set PWM and DIR pins as OUTPUT (pinMode)
	//void setDirection(float Ucalc);		// Set direction based on calculated U (control signal from PID)
	//void setPower(float Ucalc);			// Set and limit the power if PWM>255
	void startMotor(int valDIR, int valPWM, int pinPWM); // Start the motor
	void moveLeft(int valPWM);
	void moveRight(int valPWM);
	void initializeMove(int pinLimitSwitch);
	void moveToPosition(double u); // U - power PWM


private:
	int _pinPWM, _pinDIR;		// Motor PINs
	int dir;
};