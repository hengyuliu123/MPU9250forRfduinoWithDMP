#include "mpu.h"
#include <cstddef>
#include <cmath>
extern "C" {
  #include "inv_mpu.h"
  #include "inv_mpu_dmp_motion_driver.h"
}

//#define MPU_DEBUG
#define FSR 1000  //FSR=2000 1000 500 250
//#define GYRO_SENS       ( 131.0f * 250.f / (float)FSR )
#define ACCEL_SENS      4096.375f      //FSR=2000 ACCEL_SENS = 2048, FSR = 1000, ACCEL_SENS = 4096, FSR = 500, ACCEL_SENS = 8192
#define GYRO_SENS       32.875f        //FSR=2000 GYRO_SENS = 16.4, FSR = 1000, GYRO_SENS = 32.8, FSR = 500, GYRO_SENS = 65.5
#define QUAT_SENS       1073741824.f //2^30 

#define EPSILON         0.0001f
#define PI_2            1.57079632679489661923f

struct s_mympu mympu;

struct s_quat { float w, x, y, z; }; 
struct s_grav {float x, y, z;};

struct hal_s {
  //  unsigned char sensors;
    unsigned char dmp_on;
  //  unsigned char wait_for_tap;
    volatile unsigned char new_gyro;
  //  unsigned short report;
  //  unsigned short dmp_features;
  //  unsigned char motion_int_mode;
};

static struct hal_s hal = {0};

static const signed char gyro_orientation[9] = {1, 0, 0,
                                                0, 1, 0,
                                                0, 0, 1};
												
static unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}


static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;

    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */

    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;


    return scalar;
}

/* Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void)
{
    hal.new_gyro = 1;
}												

union u_quat {
	struct s_quat _f;
	long _l[4];
} q;

union u_grav {
	struct s_grav _g;
	float _ga[3];
} g;


static float grav[3];
static float yp[3];
static float yprr[3];
static int ret;
static int ret2;
static unsigned long time =0;
static short accel[3];
static short gyro[3];
static short accell[3];
static short gyroo[3];
static long qq[4];
static unsigned short ratefifo;
static short sensors;
static unsigned char fifoCount;
static unsigned long count ;     //used for pedometer features



int mympu_open() {	
	struct int_param_s int_param;
	int_param.cb = gyro_data_ready_cb;
    int_param.pin = 3;

	ret = mpu_init(&int_param);	
#ifdef MPU_DEBUG
	if (ret) return 10+ret;
#endif	
	ret = mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL); 
#ifdef MPU_DEBUG
	if (ret) return 20+ret;
#endif
        ret = mpu_set_gyro_fsr(FSR);  // set gyro's FSR
#ifdef MPU_DEBUG
	if (ret) return 30+ret;
#endif
        ret = mpu_set_accel_fsr(8);   // set accel's FSR		
#ifdef MPU_DEBUG
	if (ret) return 40+ret;
#endif
        mpu_get_power_state((unsigned char *)&ret);
#ifdef MPU_DEBUG
	if (!ret) return 50+ret;
#endif

        ret = mpu_configure_fifo(INV_XYZ_GYRO|INV_XYZ_ACCEL);
#ifdef MPU_DEBUG
	if (ret) return 60+ret;
#endif
	ret = dmp_load_motion_driver_firmware();
	ret = dmp_set_orientation( inv_orientation_matrix_to_scalar( gyro_orientation ) );
#ifdef MPU_DEBUG
	if (ret) return 80+ret;
#endif
//	ret = mpu_set_sample_rate(1000); //set sample rate 	can not be used when dmp set fifo
	ret = dmp_set_fifo_rate(100);	
#ifdef MPU_DEBUG
	if (ret) return 90+ret;
#endif
    
	ret = mpu_set_dmp_state(1);
	hal.dmp_on = 1;
#ifdef MPU_DEBUG
	if (ret) return 100+ret;
#endif
	ret = dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_SEND_CAL_GYRO|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_PEDOMETER|DMP_FEATURE_TAP); // double check

#ifdef MPU_DEBUG
	if (ret) return 110+ret;
#endif

	return 0;
}

static inline float rad2deg( float rad )
{
        //return (180.f/PI) * rad;
	return 57.2957795131f * rad;
}

void computeAngles(float *data2,float q0,float q1,float q2,float q3)
{
    data2[2] = atan2f(2.0f*(q0 * q1 + q2 * q3), 2.0f*(0.5f - q1 * q1 - q2 * q2));
    data2[1] = asinf(-2.0f * (q1 * q3 - q0 * q2));
    data2[0] = atan2f(2.0f*(q1 * q2 + q0 * q3), 2.0f*(0.5f - q2 * q2 - q3 * q3));
  //  anglesComputed = 1;
}

static inline float wrap_180(float x) {
	return (x<-180.f?x+360.f:(x>180.f?x-180.f:x));
}

int mympu_update() {
	//do {

if (hal.new_gyro && hal.dmp_on) {
	ret = dmp_read_fifo(gyro,accel,q._l,&time,&sensors,&fifoCount);
	if (!fifoCount)
		hal.new_gyro = 0;	
		/* will return:
			0 - if ok
			1 - no packet available
			2 - if BIT_FIFO_OVERFLOWN is set
			3 - if frame corrupted
		       <0 - if error
			   */
	//}while (fifoCount>1);	
	if ( ret == 0 ){
	q._f.w = (float)q._l[0] / (float)QUAT_SENS;
	q._f.x = (float)q._l[1] / (float)QUAT_SENS;
	q._f.y = (float)q._l[2] / (float)QUAT_SENS;
	q._f.z = (float)q._l[3] / (float)QUAT_SENS;
	
	
	//dmp_get_pedometer_step_count(&count);

	//quaternionToEuler( &q._f, &mympu.ypr[2], &mympu.ypr[1], &mympu.ypr[0] );
	
	computeAngles(yprr,q._f.w,q._f.x,q._f.y,q._f.z);
	/* need to adjust signs and do the wraps depending on the MPU mount orientation */ 
	/* if axis is not centered around 0 but around i.e 90 degree due to mount orientation */
	/* then do:  mympu.ypr[x] = wrap_180(90.f+rad2deg(mympu.ypr[x])); */
	/*
	mympu.ypr[0] = rad2deg(mympu.ypr[0]);                  //yaw    z
	mympu.ypr[1] = -rad2deg(mympu.ypr[1]);   //pitch  y
	mympu.ypr[2] = rad2deg(mympu.ypr[2]);   //roll   x 
	*/

	/* need to adjust signs depending on the MPU mount orientation */ 
	mympu.gyro[0] = (float)gyro[0]/GYRO_SENS; //* _gyroRes;  x
	mympu.gyro[1] = (float)gyro[1]/GYRO_SENS; //* _gyroRes;  y
	mympu.gyro[2] = (float)gyro[2]/GYRO_SENS; //* _gyroRes;  z
	
	mympu.accel[0] = (float)accel[0]/ACCEL_SENS; //* _accelRes;  x
	mympu.accel[1] = (float)accel[1]/ACCEL_SENS; //* _accelRes;  y
	mympu.accel[2] = (float)accel[2]/ACCEL_SENS; //* _accelRes;  z
	
	ret2 = dmp_get_pedometer_step_count(&count);
	dmp_get_fifo_rate(&ratefifo);
	/*
	mympu.q[0] = q._f.w;
	mympu.q[1] = q._f.x;
	mympu.q[2] = q._f.y;
	mympu.q[3] = q._f.z;   */
	/*
	mympu.DMP[0] = rad2deg(yp[0]);
	mympu.DMP[1] = -rad2deg(yp[1]);
	mympu.DMP[2] = rad2deg(yp[2]); */
    mympu.DMPreadfifotime = time;
	mympu.ypr[0] = rad2deg(yprr[0]);                  //yaw    z
	mympu.ypr[1] = rad2deg(yprr[1]);   //pitch  y
	mympu.ypr[2] = rad2deg(yprr[2]);	
	
	mympu.fifoRate = ratefifo;
	mympu.steps = count;
	mympu.fifocounter = fifoCount;
		}
	//return 0;
	return ret; 
	}
}

