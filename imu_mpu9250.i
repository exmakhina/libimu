#include <math.h>
#include <errno.h>

#if !defined(IMU_EXPORT)
# define IMU_EXPORT static inline
#endif

#include "imu_mpu9250.h"

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
	res = imu->tread(imu->ctx, ACCEL_XOUT_H, &rawData[0], 6);
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
	res = imu->tread(imu->ctx, GYRO_XOUT_H, &rawData[0], 6);
	self->gyr_data[0] = ((int16_t)rawData[0] << 8) | rawData[1];
	self->gyr_data[1] = ((int16_t)rawData[2] << 8) | rawData[3];
	self->gyr_data[2] = ((int16_t)rawData[4] << 8) | rawData[5];
	return res;
}

IMU_EXPORT int imu_mpu9250_read_mag(struct imu * imu)
{
	struct imu_mpu9250 * self = (struct imu_mpu9250*)imu;
	int res = -ENOSYS;
	return res;
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


	// wake up device
	// Clear sleep mode bit (6), enable all sensors
	imu->twrite(imu->ctx, PWR_MGMT_1, (uint8_t[]){0x00}, 1);


	imu->sleep(imu->ctx, 100000000);
	// Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

	// Check "who am I"
	{
		uint8_t val = 0;
		res = imu->tread(imu->ctx, WHO_AM_I_MPU9250, &val, 1);
		if (res != 0) {
			return -2;
		}
		if (val != 0x71) {
			return -1;
		}
	}


	// get stable time source
	imu->twrite(imu->ctx, PWR_MGMT_1, (uint8_t[]){0x01}, 1);
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001


	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	imu->twrite(imu->ctx, CONFIG, (uint8_t[]){0x03}, 1);

	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	// Use a 200 Hz rate; the same rate set in CONFIG above
	imu->twrite(imu->ctx, SMPLRT_DIV, (uint8_t[]){0x04}, 1);

	{
		// Set gyroscope full scale range
		// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
		uint8_t reg = GYRO_CONFIG;
		uint8_t c;
		imu->tread(imu->ctx, reg, &c, 1);
		c = c & ~ (BIT(7) | BIT(6) | BIT(5)); // Clear self-test bits [7:5]
		c = c & ~ (BIT(4) | BIT(3)); // Clear AFS bits [4:3]
		c = c | (self->Gscale << 3); // Configure gyro range
		imu->twrite(imu->ctx, GYRO_CONFIG, &c, 1);
	}

	{
		// Set accelerometer configuration
		uint8_t reg = ACCEL_CONFIG;
		uint8_t val;
		imu->tread(imu->ctx, reg, &val, 1);
		val = val & ~(BIT(7)|BIT(6)|BIT(5));// Clear self-test bits [7:5]
		val = val & ~(BIT(4)|BIT(3));// Clear AFS bits [4:3]
		val = val | (self->Ascale <<3);// Set full scale range for the accelerometer
		imu->twrite(imu->ctx, reg, &val, 1);

		// Set accelerometer sample rate configuration
		// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
		// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
		reg = ACCEL_CONFIG2;
		imu->tread(imu->ctx, reg, &val, 1);
		val = val & ~(BIT(3) | BIT(2)|BIT(1)|BIT(0)); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
		val = val | 0x03; // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
		imu->twrite(imu->ctx, reg, &val, 1);
	}

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	{
		// Configure Interrupts and Bypass Enable
		// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
		// can join the I2C bus and all can be controlled by the Arduino as master
		imu->twrite(imu->ctx, INT_PIN_CFG, (uint8_t[]){0x22}, 1);
		imu->twrite(imu->ctx, INT_ENABLE, (uint8_t[]){0x01}, 1);  // Enable data ready (bit 0) interrupt
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

