//#define MPU_DEBUG
#ifndef MPU_H
#define MPU_H

struct s_mympu {
	float ypr[3];
	float gyro[3];
	float accel[3];
	float q[4];
	float steps;
	float DMP[3];
	float fifocounter;
	float DMPreadfifotime;
	float fifoRate;
};

extern struct s_mympu mympu;

int mympu_open();
int mympu_update();

#endif

