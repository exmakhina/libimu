/*
  \file

  \brief Example using libftdii2c and a I2C-connected MPU-9250

  http://www.ftdichip.com/Support/Documents/AppNotes/AN_113_FTDI_Hi_Speed_USB_To_I2C_Example.pdf

*/

#include <inttypes.h>
#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <ftdii2c.h>

#include <imu_mpu9250.i>

#define DEBUG_I2C
#define DEBUG_SLEEP

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


int imut_read(imu_ctx_t ctx, int addr, int reg, uint8_t * buf, int buflen)
{
	struct fi2c_context * fic = (struct fi2c_context*)ctx;
	int res;

#if defined(DEBUG_I2C)
	printf("i2cget 0x%02x 0x%02x", addr, reg);
#endif
	uint8_t const txbuf[] = { reg };
	fic->slv = addr | 0;
	res = fi2c_wr_rd(fic, (uint8_t*)txbuf, sizeof(txbuf), buf, buflen);
#if defined(DEBUG_I2C)
	printf(" (%d)", res);
	for (int i = 0; i < buflen; i++) {
		printf(" 0x%02x", buf[i]);
	}
	printf("\n");
#endif
	return res;
}


int imut_write(imu_ctx_t ctx, int addr, int reg, uint8_t const * buf, int buflen)
{
	struct fi2c_context * fic = (struct fi2c_context*)ctx;
	int res;

#if defined(DEBUG_I2C)
	printf("i2cset 0x%02x 0x%02x", addr, reg);
#endif

	fic->slv = addr;
	uint8_t * buf_tx = malloc(buflen+1);
	buf_tx[0] = reg;
	memcpy(&buf_tx[1], buf, buflen);
	res = fi2c_wr_rd(fic, buf_tx, sizeof(buf_tx), NULL, 0);
	free(buf_tx);

#if defined(DEBUG_I2C)
	for (int i = 0; i < buflen; i++) {
		printf(" 0x%02x", buf[i]);
	}
#endif

#if defined(DEBUG_READBACK)
	{
		uint8_t const txbuf[] = { reg };
		printf(" rb ");
		//usleep(100000);
		uint8_t * buf_rb = malloc(buflen);
		res = fi2c_write_read(fsc, txbuf, sizeof(txbuf), buf_rb, buflen, 0);
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
	return res;
}


int imut_now(imu_ctx_t ctx, imu_abstime_t * now)
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
	struct ftdi_context fc;
	struct fi2c_context fic;
	memset(&theimu, 0, sizeof(theimu));

	imu->ctx = &fic;
	imu->sleep = imut_sleep;
	imu->tread = imut_read;
	imu->twrite = imut_write;
	imu->now = imut_now;
	(&theimu)->flags = BIT(IMU_MPU9250_I2C_SHARED);

	int i;

	struct ftdi_common_args fargs = {
	 .vendor_id=0x403,
	 .product_id=0x6010,
	 .dev_id=-1,
	 .interface=INTERFACE_A,
	 .serialname=NULL,
	};

	if (ftdi_init(&fc) < 0) {
		fprintf(stderr, "ftdi_init failed\n");
		return 1;
	}
	i = ftdi_usb_open(&fc, 0x0403, 0x6010);
	if (i < 0 && i != -5) {
		fprintf(stderr, "OPEN: %s\n", ftdi_get_error_string(&fc));
		exit(-1);
	}

	fi2c_init(&fic, &fc);
	fi2c_open(&fic, &fargs);
	fi2c_setclock(&fic, 100000);

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
			continue;
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
	fi2c_close(&fic);

	printf("Returning %d\n", res);
	return res;
}
