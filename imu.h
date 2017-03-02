#ifndef LIBIMU_IMU_H
#define LIBIMU_IMU_H

#include <stdint.h>
#include <inttypes.h>

#if defined(__cplusplus)
extern "C" {
#endif

//! Relative time, in nanoseconds
#ifndef imu_reltime_t
#define imu_reltime_t int64_t
#define PRIimureltime PRIi64
#endif

//! Absolute time, in nanoseconds
#ifndef imu_abstime_t
#define imu_abstime_t int64_t
#define PRIimuabstime PRIi64
#endif

#if !defined(BIT)
#define BIT(x) (1<<(x))
#endif

typedef void * imu_ctx_t;

struct imu; // fwd, see later

//! User-supplied function that would perform a sleep
typedef int (imut_sleep_t)(imu_ctx_t, imu_reltime_t dt);

typedef int (imut_read_t)(imu_ctx_t, int reg, uint8_t * buf, int buflen);

typedef int (imut_write_t)(imu_ctx_t, int reg, uint8_t const * buf, int buflen);

typedef int (imut_now_t)(imu_ctx_t, imu_abstime_t * now);



/*! Initialize device
 */
typedef int (imu_init_t)(struct imu * imu);


enum imu_read_bits {
	IMU_READ_ACC = 0, //! Accelerometer / gravity
	IMU_READ_GYR, //! Gyroscope / angular rate
	IMU_READ_MAG, //! Magnetometer / magnetic field orientation
};

/*! Read sensor

	\param flags bit mask of enum imu_read_bits
*/
typedef int (imu_poll_t)(struct imu * imu, unsigned flags);

/*! Return timestamp and accelerometer data (in m/s²)
*/
typedef int (imu_get_acc_t)(struct imu * imu, imu_abstime_t * at, float * ax, float * ay, float * az);

/*! Return timestamp and gyroscope data (in rad/s)
 */
typedef int (imu_get_gyr_t)(struct imu * imu, imu_abstime_t * gt, float * gx, float * gy, float * gz);

/*! Return timestamp and magnetometer data (in T)
 */
typedef int (imu_get_mag_t)(struct imu * imu, imu_abstime_t * mt, float * mx, float * my, float * mz);



struct imu {
	//! Context provided by user
	imu_ctx_t ctx;

	//! \name OS functions to be provided
	//! \{

	//! Request to sleep
	imut_sleep_t * sleep;

	//! Request to read a register
	imut_read_t * tread;

	//! Request to write a register
	imut_write_t * twrite;

	//! Request timestamp
	imut_now_t * now;
	//! \}


	//! \name Implemented functions
	//! \{

	imu_init_t * init;

	//! Poll hardware to retrieve data
	imu_poll_t * poll;

	//! Get engineering values of accelerometer (m/s²)
	imu_get_acc_t * get_acc;

	//! Get engineering values of gyroscope (rad/s)
	imu_get_gyr_t * get_gyr;

	//! Get engineering values of magnetometer (T)
	imu_get_mag_t * get_mag;
};


#if defined(__cplusplus)
}
#endif

#endif // LIBIMU_IMU_H
