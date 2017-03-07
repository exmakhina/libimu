/*!
  \file
  \brief Implementation of MPU-9250 transport

  TODO:
  - Make on-board I2C master interface work
  - Support I2C bypass too

*/

#include <math.h>
#include <errno.h>

#if !defined(IMU_EXPORT)
# define IMU_EXPORT static inline
#endif

#include "imu_mpu9250.h"

//#define IMU_DEBUG

#if defined(IMU_DEBUG)
# include <stdio.h>
#else
# define printf(fmt, ...)
# define fprintf(f, fmt, ...)
#endif

#define IMU_MPU9250_MAGACCESS_OPTIMIZE_FAVOR_R

/*! Return the magnetometer resolution (T) from current config

    Ref: §5 Register Map for Magnetometer / §5.6 Measurement Data
*/
IMU_EXPORT float imu_mpu9250_get_magres(uint8_t mag_scale)
{
	float const fs = 4912e-6f;
	switch (mag_scale) {
		case MFS_14BITS: return fs/8190.0;
		case MFS_16BITS: return fs/32760.0;
	}
}


/*! Return the gyro resolution (rad/s) from current config

    Ref: §4 Register Descriptions / §4.6 Register 27 – Gyroscope Configuration
*/
IMU_EXPORT float imu_mpu9250_get_gyrres(uint8_t gyr_scale)
{
	switch (gyr_scale) {
		case GFS_250DPS: return 250.0/32768.0*M_PI/180;
		case GFS_500DPS: return 500.0/32768.0*M_PI/180;
		case GFS_1000DPS: return 1000.0/32768.0*M_PI/180;
		case GFS_2000DPS: return 2000.0/32768.0*M_PI/180;
	}
}


/*! Return the accel resolution (m/s²) from current config

    Ref: §4 Register Descriptions / §4.7 Register 28 – Accelerometer Configuration
*/
IMU_EXPORT float imu_mpu9250_get_accres(uint8_t acc_scale)
{
	float const g = 9.80665;
	switch (acc_scale) {
		case AFS_2G: return g*2.0/32768.0;
		case AFS_4G: return g*4.0/32768.0;
		case AFS_8G: return g*8.0/32768.0;
		case AFS_16G: return g*16.0/32768.0;
	}
}


IMU_EXPORT int imu_mpu9250_read_acc(struct imu * imu)
{
	struct imu_mpu9250 * self = (struct imu_mpu9250*)imu;
	int res;

	uint8_t rawData[6];
	res = imu->tread(imu->ctx, ACCEL_XOUT_H, &rawData[0], sizeof(rawData));
	self->acc_data[0] = ((int16_t)rawData[0] << 8) | rawData[1];
	self->acc_data[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	self->acc_data[2] = ((int16_t)rawData[4] << 8) | rawData[5];
	return res;
}

IMU_EXPORT int imu_mpu9250_read_gyr(struct imu * imu)
{
	struct imu_mpu9250 * self = (struct imu_mpu9250*)imu;
	int res;

	uint8_t rawData[6];
	res = imu->tread(imu->ctx, GYRO_XOUT_H, &rawData[0], sizeof(rawData));
	self->gyr_data[0] = ((int16_t)rawData[0] << 8) | rawData[1];
	self->gyr_data[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	self->gyr_data[2] = ((int16_t)rawData[4] << 8) | rawData[5];
	return res;
}


IMU_EXPORT int imu_mpu9250_tread_mag(struct imu * imu,
 uint8_t reg, uint8_t * buf, int count)
{
	struct imu_mpu9250 * self = (struct imu_mpu9250*)imu;
	int res;
	uint8_t val;

	/* Note that we're already set to read */

#if !defined(IMU_MPU9250_MAGACCESS_OPTIMIZE_FAVOR_R)
	val = AK8963_ADDRESS | BIT(7);
	imu->twrite(imu->ctx, I2C_SLV0_ADDR, &val, 1);
#endif

#if defined(IMU_MPU9250_MAGREAD_OPTIMIZED)
	/* clear read interrupt, will use it to know when data is ready*/
	imu->tread(imu->ctx, INT_STATUS, &val, sizeof(val));
#endif

	imu->twrite(imu->ctx, I2C_SLV0_REG, &reg, sizeof(reg));

	val = 0x80 | count;
	imu->twrite(imu->ctx, I2C_SLV0_CTRL, &val, sizeof(val));

#if defined(IMU_MPU9250_MAGREAD_OPTIMIZED)
	do {
		imu->tread(imu->ctx, INT_STATUS, &val, sizeof(val));
		if ((val & BIT(0)) == 0) {
			break;
		}
	} while (1);
#else
	imu->sleep(imu->ctx, (imu_abstime_t)100000 * count);
#endif

	res = imu->tread(imu->ctx, EXT_SENS_DATA_00, buf, count);

#if defined(IMU_MPU9250_MAGACCESS_RB)
	val = 0;
	imu->twrite(imu->ctx, I2C_SLV0_CTRL, &val, sizeof(val));
	imu->tread(imu->ctx, I2C_SLV0_ADDR, &val, sizeof(val));
	imu->tread(imu->ctx, I2C_SLV0_REG, &val, sizeof(val));
	imu->tread(imu->ctx, I2C_SLV0_CTRL, &val, sizeof(val));
#endif

	return res;
}

IMU_EXPORT int imu_mpu9250_twrite_mag(struct imu * imu,
 uint8_t reg, uint8_t * buf, int count)
{
	struct imu_mpu9250 * self = (struct imu_mpu9250*)imu;
	uint8_t val;

	// set to write
	val = AK8963_ADDRESS;
	imu->twrite(imu->ctx, I2C_SLV0_ADDR, &val, sizeof(val));
	imu->twrite(imu->ctx, I2C_SLV0_REG, &reg, sizeof(reg));
	imu->twrite(imu->ctx, I2C_SLV0_DO, &buf[0], 1);
	val = 0x80 | count;
	imu->twrite(imu->ctx, I2C_SLV0_CTRL, &val, sizeof(val));

#if defined(IMU_MPU9250_MAGACCESS_OPTIMIZE_FAVOR_R)
	// go back to read mode
	val = AK8963_ADDRESS | BIT(7);
	imu->twrite(imu->ctx, I2C_SLV0_ADDR, &val, sizeof(val));
#endif

	//TODO readback?
	return 0;
}


IMU_EXPORT int imu_mpu9250_read_mag(struct imu * imu)
{
	struct imu_mpu9250 * self = (struct imu_mpu9250*)imu;
	int res = 0;

	do {
		uint8_t val;
		imu_mpu9250_tread_mag(imu, AK8963_ST1, &val, sizeof(val));
		if ((val & BIT(0)) == 0) {
			printf("Mag data not ready yet\n");
			//continue;
		}
		break;
	} while (1);

	uint8_t rawData[7];
	// must read ST2 at end of data acquisition
	imu_mpu9250_tread_mag(imu, 0x03, &rawData[0], sizeof(rawData));

	uint8_t st2 = rawData[6]; /* check for overflow */
#if 1
	if ((st2 & BIT(4)) == 0) {
		fprintf(stderr, "Invalid st2 register contents (0x%02x), expecting 16-bit config\n", st2);
		return -1;
	}
#endif
	if ((st2 & BIT(3)) != 0) {
		fprintf(stderr, "Magnetic field overflow\n");
		return -2;
	}

	self->mag_data[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);
	self->mag_data[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]);
	self->mag_data[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]);

	return 0;
}

int16_t imu_mpu9250_read_temp(struct imu_mpu9250 * self)
{
	int res = -ENOSYS;
	return res;
}


void imu_mpu9250_reset(struct imu * imu)
{
	// reset device
	imu->twrite(imu->ctx, PWR_MGMT_1, (uint8_t[]){0x80}, 1); // Write a one to bit 7 reset bit; toggle reset device
	imu->sleep(imu->ctx, 100000000);
}


static int imu_mpu9250_initialize(struct imu * imu)
{
	struct imu_mpu9250 * self = (struct imu_mpu9250*)imu;
	int res;

	printf("Check who am I\n");
	uint8_t val = 0;
	{
		res = imu->tread(imu->ctx, WHO_AM_I_MPU9250, &val, sizeof(val));
		if (res != 0) {
			return -2;
		}
		if (val != 0x71) {
			return -1;
		}
	}

	printf("Wake up device, reset everything\n");
	val = BIT(7);
	imu->twrite(imu->ctx, PWR_MGMT_1, &val, sizeof(val));

	/* default value is fine */
	//val = BIT(0);
	//imu->twrite(imu->ctx, PWR_MGMT_1, &val, sizeof(val));

	val = 0;
	imu->twrite(imu->ctx, PWR_MGMT_2, &val, sizeof(val));

	printf("Enable interrupts for raw data only\n");
	val = 0;
	val |= BIT(0); /* enable raw data ready interrupt only */
	imu->twrite(imu->ctx, INT_ENABLE, &val, 1);

	/*
	   Wait for something curious to clear...
	*/
	for (int i = 0; i < 100; i++) {
		imu->tread(imu->ctx, INT_STATUS, &val, sizeof(val));
		if (val != 0) {
			fprintf(stderr, "INT_STATUS (%2x)?\n", val);
		}
		if ((val & BIT(2)) == 0) {
			break;
		}
	}


	/*
	  Note: in SPI with I2C master proxy,
	  BYPASS_EN does *not* seem to be mandatory,
	  but we set it nevertheless.
	 */
	val = 0;
	val |= BIT(5); /* LATCH_INT_EN */
	val |= BIT(1); /* I2C master pins bypass mode if disabled */
	imu->twrite(imu->ctx, INT_PIN_CFG, &val, sizeof(val));

	printf("Configure internal clock as we don't care about DMP\n");
	/* Ref: PS § 4.1 clocking / RM § 4.34 Register 107 – Power Management 1*/
	val = 1; /* internal clock */
	imu->twrite(imu->ctx, PWR_MGMT_1, &val, sizeof(val));

	printf("Configure Gyro\n");
	/*
	  Ref: RS § 4.5 Register 26 – Configuration
	  - DLPF_CFG: don't care, we'll use the maximum available rate
	*/
	val = 0;
	imu->twrite(imu->ctx, CONFIG, &val, sizeof(val));

	/*
	  Ref: RS § 4.4 Register 25 – Sample Rate Divider
	 - max sample rate (don't care, as fchoice will be ...)
	*/
	val = 0;
	imu->twrite(imu->ctx, SMPLRT_DIV, &val, sizeof(val));


	/*
	  Ref: RS § 4.6 Register 27 – Gyroscope Configuration
	*/
	val = 0;
	val |= self->Gscale << 3; /* range */
	val |= BIT(1) | BIT(0); /* set FCHOICE for no DLPF */
	imu->twrite(imu->ctx, GYRO_CONFIG, &val, 1);


	printf("Configure Accelerometer\n");
	/*
	  Ref:
	*/
	val = 0;
	val |= self->Ascale <<3; /* range */
	imu->twrite(imu->ctx, ACCEL_CONFIG, &val, 1);

	val = 0;
	val |= BIT(1) | BIT(0); /* accel_fchoice_b */
	imu->twrite(imu->ctx, ACCEL_CONFIG2, &val, 1);



	{
		printf("Check interrupt workability\n");

		imu->tread(imu->ctx, INT_STATUS, &val, sizeof(val));
		imu->tread(imu->ctx, INT_STATUS, &val, sizeof(val));

#if 0
		if (val != 0) {
			fprintf(stderr, "Why isn't INT_STATUS 0 after 2 reads (%2x)?\n", val);
			return -3;
		}
#endif
	}

	/*
	  Prepare communication with magnetometer
	 */
	{
		printf("Enable I2C master\n");
		val = 0;
		val |= BIT(5); /* I2C_MST_EN */
		val |= BIT(4); /* Disable I2C slave */
		// (https://www.invensense.com/developers/forums/topic/how-can-i-access-magak8953-with-spi/#post-36017)
		imu->twrite(imu->ctx, USER_CTRL, &val, 1);

		imu->sleep(imu->ctx, (imu_abstime_t)100000000);

		/*
		  WAIT_FOR_ES is not mandatory to get built-in I2C master function,
		  TODO clarify what it would do.
		 */
		val = 0;
		val |= BIT(6); /* WAIT_FOR_ES */
		val |= BIT(4); /* start/msg/stop */
		val |= 13; /* 400 kHz */
		imu->twrite(imu->ctx, I2C_MST_CTRL, &val, 1);

#if defined(IMU_MPU9250_I_REALLY_WANT_TO_RESET_I2C_MST)
		imu->tread(imu->ctx, USER_CTRL, &val, 1);
		val |= BIT(1); /* I2C_MST_RST */
		imu->twrite(imu->ctx, USER_CTRL, &val, 1);
#endif

		printf("Slave address + read\n");

#if defined(IMU_MPU9250_MAGACCESS_OPTIMIZE_FAVOR_R)
		/*
		  Enable read of slave, by default.
		  Writes are occasional and the write operation should reset
		  I2C_SLV0_ADDR for a read.
		*/
		val = AK8963_ADDRESS | BIT(7);
		imu->twrite(imu->ctx, I2C_SLV0_ADDR, &val, 1);
#endif

#if 1
		int err_count = 0;
		for (int i = 0; i < 10; i++) {
			printf("Read whoami\n");
			imu_mpu9250_tread_mag(imu, WHO_AM_I_AK8963, &val, 1);
			if (val != 0x48) {
				printf("NG %d\n", val);
				err_count++;
			}
			else {
				printf("\x1B[33;1mOK!\x1B[0m\n");
			}
		}
		if (err_count > 0) {
			return -3;
		}
#endif
	}


	{
		/*
		  Configure the magnetometer.
		 */

		printf("Reset\n");
		val = BIT(0);
		imu_mpu9250_twrite_mag(imu, AK8963_CNTL2, &val, sizeof(val));

		printf("Continuous measurement in 16bit\n");
		val = 0;
		val |= BIT(4); /* 16-bit */
		val |= BIT(2)|BIT(1); /* continuous measurement mode 100 Hz */
		imu_mpu9250_twrite_mag(imu, AK8963_CNTL1, &val, sizeof(val));

#if 0
		printf("Configure automatic readout by MPU-9250\n");
		val = AK8963_ADDRESS | BIT(7); // Enable read
		imu->twrite(imu->ctx, I2C_SLV0_ADDR, &val, sizeof(val));
		val = AK8963_ST1;
		imu->twrite(imu->ctx, I2C_SLV0_REG, &val, sizeof(val));
		val = 0x80 | 7;
		imu->twrite(imu->ctx, I2C_SLV0_CTRL, &val, sizeof(val));
		val = 0x81;
		imu->twrite(imu->ctx, I2C_MST_DELAY_CTRL, &val, sizeof(val));
#endif

	}

	{
		self->acc_res = imu_mpu9250_get_accres(self->Ascale);
		for (int i = 0; i < 3; i++) {
			self->acc_bias[i] = 0.f;
			self->acc_gain[i] = self->acc_res;
		}

		self->gyr_res = imu_mpu9250_get_gyrres(self->Gscale);
		for (int i = 0; i < 3; i++) {
			self->gyr_bias[i] = 0.f;
			self->gyr_gain[i] = self->gyr_res;
		}

		self->mag_res = imu_mpu9250_get_magres(self->Mscale);
		for (int i = 0; i < 3; i++) {
			self->mag_bias[i] = 0.f;
			self->mag_gain[i] = self->mag_res;
		}
	}

	return res;
}


IMU_EXPORT int imu_mpu9250_poll(struct imu * imu, unsigned flags)
{
	struct imu_mpu9250 * self = (struct imu_mpu9250*)imu;
	int ret = 0;
	int res;

	if ((flags & BIT(IMU_READ_ACC)) != 0) {
		imu->now(imu->ctx, &self->t_acc);
		res = imu_mpu9250_read_acc(imu);
		if (res != 0) {
			ret = res;
		}
	}

	if ((flags & BIT(IMU_READ_GYR)) != 0) {
		imu->now(imu->ctx, &self->t_gyr);
		res = imu_mpu9250_read_gyr(imu);
		if (res != 0) {
			ret = res;
		}
	}

	if ((flags & BIT(IMU_READ_MAG)) != 0) {
		imu->now(imu->ctx, &self->t_mag);
		res = imu_mpu9250_read_mag(imu);
		if (res != 0) {
			ret = res;
		}
	}

	return ret;
}


IMU_EXPORT int imu_mpu9250_get_acc(struct imu * imu,
 imu_abstime_t * at, float * ax, float * ay, float * az)
{
	struct imu_mpu9250 * self = (struct imu_mpu9250*)imu;
	*at = self->t_acc;
	*ax = (self->acc_data[0] + self->acc_bias[0]) * self->acc_gain[0];
	*ay = (self->acc_data[1] + self->acc_bias[1]) * self->acc_gain[1];
	*az = (self->acc_data[2] + self->acc_bias[2]) * self->acc_gain[2];
	return 0;
}

IMU_EXPORT int imu_mpu9250_get_gyr(struct imu * imu,
  imu_abstime_t * gt, float * gx, float * gy, float * gz)
{
	struct imu_mpu9250 * self = (struct imu_mpu9250*)imu;
	*gt = self->t_gyr;
	*gx = (self->gyr_data[0] + self->gyr_bias[0]) * self->gyr_gain[0];
	*gy = (self->gyr_data[1] + self->gyr_bias[1]) * self->gyr_gain[1];
	*gz = (self->gyr_data[2] + self->gyr_bias[2]) * self->gyr_gain[2];
	return 0;
}

IMU_EXPORT int imu_mpu9250_get_mag(struct imu * imu,
 imu_abstime_t * mt, float * mx, float * my, float * mz)
{
	struct imu_mpu9250 * self = (struct imu_mpu9250*)imu;
	*mt = self->t_mag;
	*mx = (self->mag_data[0] + self->mag_bias[0]) * self->mag_gain[0];
	*my = (self->mag_data[1] + self->mag_bias[1]) * self->mag_gain[1];
	*mz = (self->mag_data[2] + self->mag_bias[2]) * self->mag_gain[2];
	return 0;
}

IMU_EXPORT int imu_mpu9250_init(struct imu * imu)
{
	struct imu_mpu9250 * self = (struct imu_mpu9250*)imu;
	int res = 0;

	// Configuration
	self->Ascale = AFS_2G;
	self->Gscale = GFS_250DPS;
	self->Mscale = MFS_16BITS;
	self->Mmode = 0x06;

	// Initialize device
	res = imu_mpu9250_initialize(imu);

	// Setup imu interface
	imu->poll = &imu_mpu9250_poll;
	imu->get_acc = &imu_mpu9250_get_acc;
	imu->get_gyr = &imu_mpu9250_get_gyr;
	imu->get_mag = &imu_mpu9250_get_mag;

	return res;
}

