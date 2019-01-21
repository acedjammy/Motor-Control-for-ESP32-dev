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

class Motor {
	public:
		Motor(int EN, int DIR1, int DIR2);
		void Setup();
		void Spin(float effort);
		void reset();
		
	private:
		int EN;
		int DIR1;
		int DIR2;
		int pwm_ch = 1;
		uint8 pwm = 0;
};

extern int pwm_CH;
