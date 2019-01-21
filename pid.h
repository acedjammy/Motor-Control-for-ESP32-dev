/*******************************************************
 * Copyright (C) 2018-2019 {Yang Ding} <{ding92@purdue.edu}>
 * 
 * This file is part of {Robotic Arm Development}.
 * 
 * {Robotic Arm Development} can not be copied and/or distributed without the express
 * permission of {Yang Ding}
 *******************************************************/
 #ifndef uint8
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long uint32;
#endif

#define INCREMENTAL_PID 0x00
#define POSITION_PID 0x01
#define NOW 0x00
#define LAST 0X01
#define LLAST 0X02


typedef struct pid_c
{
  double Kp;
  double Ki;
  double Kd;

  double set;
  double Get;
  double error[3];

  double pout;
  double iout;
  double dout;
  double out;

  double input_max_err;    //input max err;
  double output_deadband;  //output deadband; 
  
  uint8 pid_mode;
  double max_output;
  uint32 intergral_limit;

  void (*out_func)(struct pid_c* pid);
   
} pid_c;

void pid_init(struct  pid_c*    pid, 
                      uint8    pid_mode,
                      double    max_output,
                      uint32    intergral_limit,
                      double     Kp,
                      double     Ki,
                      double     Kd);
                      
void pid_reset(struct pid_c* pid, double Kp, double Ki, double Kd);

void pid_cal(struct pid_c* pid, double set, double Get);
