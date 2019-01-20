#include "pid.h"

static void out_saturation(double *output, double abs_sat);
static void pid_output_pos(struct pid_c* pid);
static void pid_output_inc(struct pid_c* pid);

void pid_init(struct    pid_c*    pid, 
                        uint8    pid_mode,
                        double    max_output,
                        uint32    intergral_limit,
                        double     Kp,
                        double     Ki,
                        double     Kd) 
{
  pid->pid_mode = pid_mode;  
  pid->max_output = max_output;
  pid->intergral_limit = intergral_limit;
  pid->pout = 0;
  pid->dout = 0;
  pid->iout = 0;
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  }

void pid_cal(struct pid_c* pid, double set, double Get){
  pid->set = set;
  pid->Get = Get;
  if (pid->pid_mode == INCREMENTAL_PID) pid_output_inc(pid);
  if (pid->pid_mode == POSITION_PID) pid_output_pos(pid);
}

void pid_reset(struct pid_c* pid, double Kp, double Ki, double Kd){  
  pid->pout = 0;
  pid->dout = 0;
  pid->iout = 0;
  pid->Kp = Kp;
  pid->Ki = Ki;
  pid->Kd = Kd;
  pid->out = pid->pout = pid->iout = pid->dout = 0;
}

static void pid_output_pos(struct pid_c* pid){
  pid->error[NOW] = pid->set - pid->Get;
  pid->pout = pid->Kp * pid->error[NOW];
  pid->iout += pid->Ki * pid->error[NOW];
  pid->dout = pid->Kd * (pid->error[NOW] - pid->error[LAST]);  
  pid->out = pid->pout + pid->iout + pid->dout;
  out_saturation(&pid->out, pid->max_output);
  pid->error[LAST] = pid->error[NOW];  
}

static void pid_output_inc(struct pid_c* pid){
  double delta_out;
  pid->error[NOW] = pid->set - pid->Get;
  delta_out = pid->Kp * (pid->error[NOW] - pid->error[LAST]) \
              + pid->Ki * pid->error[NOW] \
              + pid->Kd * (pid->error[NOW] - 2 * pid->error[LAST] + pid->error[LLAST]);  
  pid->out += delta_out;
  out_saturation(&pid->out,pid->max_output);
  pid->error[LLAST] = pid->error[LAST];
  pid->error[LAST] = pid->error[NOW];
}

void out_saturation(double *output, double abs_sat)
{
  if (*output > abs_sat)
    *output = abs_sat;
  if (*output < -abs_sat)
    *output = -abs_sat;
}
