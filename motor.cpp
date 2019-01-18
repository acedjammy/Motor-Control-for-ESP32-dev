#include "Arduino.h"
#include "motor.h"
int pwm_CH = 1;

Motor::Motor(int EN, int DIR1, int DIR2){
	this->EN = EN;
	this->DIR1 = DIR1;
	this->DIR2 = DIR2;
}

void Motor::Setup(){
	this->pwm_ch = pwm_CH;
	pinMode(DIR1, OUTPUT);
	pinMode(DIR2, OUTPUT);
	ledcSetup(this->pwm_ch, 8000, 8);
	ledcAttachPin(EN, this->pwm_ch);
	pwm_CH++;
	if (pwm_CH == 16) pwm_CH = 1;
}

void Motor::Spin(double effort) {
	pwm = (uint8)abs(effort);
	if (effort > 0) {
		digitalWrite(DIR1, HIGH);
		digitalWrite(DIR2, LOW);
		ledcWrite(this->pwm_ch, pwm);
	}
	else if (effort < 0){
		digitalWrite(DIR1, LOW);
		digitalWrite(DIR2, HIGH);
		ledcWrite(this->pwm_ch, pwm);
	}
 else {
    ledcWrite(this->pwm_ch, 0);
 }
}

void Motor::reset() {
	this->DIR1 = NULL;
	this->DIR2 = NULL;
	this->EN = NULL;
	this->pwm_ch = NULL;
	this->pwm = NULL;
	pwm_CH = 1;
}
