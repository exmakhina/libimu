/*
  \file

  \brief Example using a raspberry Pi and a I2C-connected MPU-9250

*/

#include <inttypes.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */
#include <linux/i2c-dev.h>

#include <imu_mpu9250.i>

//#define DEBUG_I2C
//#define DEBUG_SLEEP

struct ctx {
	int fd_i2c;
	int addr;
};

static int imut_sleep(imu_ctx_t ctx, int64_t dt)
{
	int res;
#if defined(DEBUG_SLEEP)
	printf("sleep %" PRIu64 " µs\n", dt/(uint64_t)1000);
#endif
	struct timespec request;
	request.tv_sec = dt / (uint64_t)1000000000;
	request.tv_nsec = dt % (uint64_t)1000000000;
	struct timespec remain;
	res = clock_nanosleep(CLOCK_MONOTONIC, 0, &request, &remain);
	return res;
}

static int imut_i2c_set_slave(imu_ctx_t _ctx, int address)
{
	struct ctx * ctx = (struct ctx*)_ctx;
	int res = 0;

	if (ctx->addr != address) {
		res = ioctl(ctx->fd_i2c, I2C_SLAVE, address);
		if (res < 0) {
			fprintf(stderr, "Setting address failed: %d %d\n", res, errno);
		}
		else {
			ctx->addr = address;
		}
	}

	return res;

}


static int imut_read(imu_ctx_t _ctx, int addr, int reg, uint8_t * buf, int buflen)
{
	struct ctx * ctx = (struct ctx*)_ctx;
	int res;

#if defined(DEBUG_I2C)
	printf("i2cget(%p) 0x%02x 0x%02x", ctx, addr, reg);
#endif

	res = imut_i2c_set_slave(_ctx, addr);
	if (res != 0) {
		return -3;
	}

	uint8_t const txbuf[] = { reg };
	res = write(ctx->fd_i2c, txbuf, sizeof(txbuf));
	if (res != 1) {
		fprintf(stderr, "Cannot write register\n");
		return -1;
	}
	res = read(ctx->fd_i2c, buf, buflen);
	if (res != buflen) {
		fprintf(stderr, "Cannot read data\n");
		return -2;
	}
#if defined(DEBUG_I2C)
	printf(" (%d)", res);
	for (int i = 0; i < buflen; i++) {
		printf(" 0x%02x", buf[i]);
	}
	printf("\n");
#endif
	return 0;
}


static int imut_write(imu_ctx_t _ctx, int addr, int reg, uint8_t const * buf, int buflen)
{
	struct ctx * ctx = (struct ctx*)_ctx;
	int res;

#if defined(DEBUG_I2C)
	printf("i2cset(%p) 0x%02x 0x%02x", ctx, addr, reg);
#endif

	res = imut_i2c_set_slave(_ctx, addr);
	if (res != 0) {
		return -3;
	}

	uint8_t * buf_tx = malloc(buflen+1);
	buf_tx[0] = reg;
	memcpy(&buf_tx[1], buf, buflen);
	res = write(ctx->fd_i2c, buf_tx, buflen+1);
	if (res != buflen+1) {
		fprintf(stderr, "Cannot write data\n");
		return -1;
	}
	free(buf_tx);
#if defined(DEBUG_I2C)
	for (int i = 0; i < buflen; i++) {
		printf(" 0x%02x", buf[i]);
	}
#endif

//#define DEBUG_READBACK
#if defined(DEBUG_READBACK)
	{
		uint8_t const txbuf[] = { reg };
		printf(" rb ");
		//usleep(100000);
		uint8_t * buf_rb = malloc(buflen);
		res = write(ctx->fd_i2c, &txbuf[0], sizeof(txbuf));
		res = read(ctx->fd_i2c, buf_rb, buflen);
		res = 0;
		printf(" (%d)", res);
		for (int i = 0; i < buflen; i++) {
			if (buf_rb[i] != buf[i]) {
				printf("\x1B[31;1m");
			}
			printf(" 0x%02x", buf_rb[i]);
			printf("\x1B[0m");
		}
		free(buf_rb);
	}
#endif

#if defined(DEBUG_I2C)
	printf("\n");
#endif
	return 0;
}


static int imut_now(imu_ctx_t ctx, imu_abstime_t * now)
{
	struct timespec ts;
	int res;
	res = clock_gettime(CLOCK_MONOTONIC, &ts);
	*now = ((uint64_t)ts.tv_sec * (uint64_t)1000000000) + ts.tv_nsec;
	return res;
}



int main(int argc, char **argv)
{
	int res;
	struct imu_mpu9250 theimu;
	struct imu * imu = (struct imu*)&theimu;

	struct ctx ctx = {
	 .fd_i2c = -1,
	 .addr = 0,
	};

	ctx.fd_i2c = open("/dev/i2c-1", O_RDWR);
	if (ctx.fd_i2c < 0) {
		fprintf(stderr, "Cannot open I2C device file\n");
		return -1;
	}
	printf("fd: %d\n", ctx.fd_i2c);

	imu->ctx = &ctx;
	imu->sleep = imut_sleep;
	imu->tread = imut_read;
	imu->twrite = imut_write;
	imu->now = imut_now;
	(&theimu)->flags = BIT(IMU_MPU9250_I2C_SHARED);
	(&theimu)->flags |= BIT(IMU_MPU9250_MAG_SINGLE);
	(&theimu)->flags |= BIT(IMU_MPU9250_MAG_WAIT);

	res = imu_mpu9250_init(imu);
	if (res != 0) {
		fprintf(stderr, "Chip unreachable: %d\n", res);
		goto end;
	}

	printf("poll\n");
	while (1) {
		unsigned flags = 0;
		flags |= BIT(IMU_READ_GYR);
		flags |= BIT(IMU_READ_ACC);
		flags |= BIT(IMU_READ_MAG);
		res = imu->poll(imu, flags);

		if (res != 0) {
			/* continue; */
		}

		if (0) {
			imu_mpu9250_read_temp(imu);
			float temp;
			imu_mpu9250_get_temp(imu, NULL, &temp);
			printf(" %7.3f °C", temp);
		}

		if ((flags & BIT(IMU_READ_ACC)) != 0) {
			imu_abstime_t t;
			float ax, ay, az;
			res = imu->get_acc(imu, &t, &ax, &ay, &az);
			printf(" t=%" PRIimuabstime " ax=%7.3f ay=%7.3f az=%7.3f (m/s²)",
			 t, ax, ay, az);
		}

		if ((flags & BIT(IMU_READ_GYR)) != 0) {
			imu_abstime_t t;
			float gx, gy, gz;
			res = imu->get_gyr(imu, &t, &gx, &gy, &gz);
			printf(" t=%" PRIimuabstime " gx=%7.3f gy=%7.3f gz=%7.3f (rad/s)",
			 t, gx, gy, gz);
		}

		if ((flags & BIT(IMU_READ_MAG)) != 0) {
			imu_abstime_t t;
			float mx, my, mz;
			res = imu->get_mag(imu, &t, &mx, &my, &mz);
			printf(" t=%" PRIimuabstime " mx=%7.3f my=%7.3f mz=%7.3f (µT)",
			 t, 1e6*mx, 1e6*my, 1e6*mz);
		}
		printf("\n");

	}

end:

	printf("Returning %d\n", res);
	return res;
}
