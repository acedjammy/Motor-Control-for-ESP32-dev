#ifndef uint8
typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned long uint32;
#endif // !uint8

class Motor {
	public:
		Motor(int EN, int DIR1, int DIR2);
		void Setup();
		void Spin(double effort);
		void reset();
		
	private:
		int EN;
		int DIR1;
		int DIR2;
		int pwm_ch = 1;
		uint8 pwm = 0;
};

extern int pwm_CH;
