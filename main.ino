/*******************************************************
 * Copyright (C) 2018-2019 {Yang Ding} <{ding92@purdue.edu}>
 * 
 * This file is part of {Robotic Arm Development}.
 * 
 * {Robotic Arm Development} can not be copied and/or distributed without the express
 * permission of {Yang Ding}
 *******************************************************/ 
#include "motor.h"
#include "pid.h"

/*Timed loop parameter setup*/
hw_timer_t *timer = NULL;
double Ts = 0.001; //Loop period
volatile uint32 isrCounter = 0;
volatile uint32 lastIsrAt = 0;
unsigned long time_index = 0;

/*Create Objects for motor and pid_c*/
Motor *TM = new Motor(0, 2, 15);
pid_c *pid = new pid_c;

/*Setup measurement pin and data storage*/
int sensor_pin = A0; //Potentiometer
double motor_position;

/*Kalman*/
int F, G, H;
double Q, RR;
double X_pre, P_pre, Kg, e;
double Xkf[2] = { 0.01 };
double P[2] = { 0.01 };

/*Desired trajectory*/
double yd;

void IRAM_ATTR onTimer(){
  /*Generating desired trajectory point at each time point*/
  yd = 5-4*cos(2 * PI * 0.25 * Ts * time_index++);

  /*Read the position value*/
  motor_position = analogRead(sensor_pin);

  /*Kalman Filter Calculation*/
  X_pre = F * Xkf[LAST];
  P_pre = F * P[LAST] * F + Q;
  Kg = P_pre * 1/ ( H * P_pre * H + RR );
  e = motor_position / 4095 * 10 - H * X_pre; //12 bits ADC, real measurement maximum is 10 mm
  Xkf[NOW] = X_pre + Kg * e;
  P[NOW] = (1 - Kg * H) * P_pre;

  /*Call pid_cal to generate pid control effort*/
  pid_cal(pid,yd,Xkf[NOW]);

  /*Call motor spin function to let motor rotate*/
  TM->Spin(pid->out);  
}


void setup() {
  /*Initialize timed interrupt*/
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000 / (1/Ts), true);
  timerAlarmEnable(timer);

  /*Initialize PID*/
  pid_init(pid,POSITION_PID,7,0,10,0,0);
  
  /*Initialize Kalman Filter*/
  F = G = H = 1;
  Q = 0.00000001;
  RR = 0.000001;
  Kg = 0.001;
  
  Serial.begin(115200);
}

void loop() {
  
  
}
