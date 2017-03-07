/*
  \file

  \brief Example using libftdispi and a SPI-connected MPU-9250
  (to make things interesting).

*/

#include <inttypes.h>
#include <time.h>
#include <stdio.h>
#include <stdint.h>
#include <ftdispi.h>

#include <imu_mpu9250.i>

//#define DEBUG_SPI

static int imut_sleep(imu_ctx_t ctx, int64_t dt)
{
	int res;
	printf("sleep %" PRIu64 " µs\n", dt/(uint64_t)1000);
	struct timespec request;
	request.tv_sec = dt / (uint64_t)1000000000;
	request.tv_nsec = dt % (uint64_t)1000000000;
	struct timespec remain;
	res = clock_nanosleep(CLOCK_MONOTONIC, 0, &request, &remain);
	return res;
}


int imut_read(imu_ctx_t ctx, int reg, uint8_t * buf, int buflen)
{
	struct ftdispi_context * fsc = (struct ftdispi_context*)ctx;
	int res;

#if defined(DEBUG_SPI)
	printf("spiget 0x%02x", reg);
#endif
	uint8_t const txbuf[] = { (1<<7) | reg };
	res = ftdispi_write_read(fsc, txbuf, sizeof(txbuf), buf, buflen, 0);
#if defined(DEBUG_SPI)
	for (int i = 0; i < buflen; i++) {
		printf(" 0x%02x", buf[i]);
	}
	printf("\n");
#endif
	return res;
}


int imut_write(imu_ctx_t ctx, int reg, uint8_t const * buf, int buflen)
{
	struct ftdispi_context * fsc = (struct ftdispi_context*)ctx;
	int res;

#if defined(DEBUG_SPI)
	printf("spiset 0x%02x", reg);
#endif

#if defined(DOESNT_WORK)
	uint8_t const txbuf[] = { (0<<7) | reg };
	res = ftdispi_write(fsc, txbuf, sizeof(txbuf), 0);
	res = ftdispi_write(fsc, buf, buflen, 0);
#else
	uint8_t * buf_tx = malloc(buflen+1);
	buf_tx[0] = (0<<7) | reg;
	memcpy(&buf_tx[1], buf, buflen);
	res = ftdispi_write(fsc, buf_tx, buflen+1, 0);
	free(buf_tx);
#endif

#if defined(DEBUG_SPI)
	for (int i = 0; i < buflen; i++) {
		printf(" 0x%02x", buf[i]);
	}
#endif

#if defined(DEBUG_READBACK)
	{
		uint8_t const txbuf[] = { (1<<7) | reg };
		printf(" rb ");
		//usleep(100000);
		uint8_t * buf_rb = malloc(buflen);
		res = ftdispi_write_read(fsc, txbuf, sizeof(txbuf), buf_rb, buflen, 0);
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

#if defined(DEBUG_SPI)
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
	struct ftdispi_context fsc;
	imu->ctx = &fsc;
	imu->sleep = imut_sleep;
	imu->tread = imut_read;
	imu->twrite = imut_write;
	imu->now = imut_now;

	int i;

	if (ftdi_init(&fc) < 0) {
		fprintf(stderr, "ftdi_init failed\n");
		return 1;
	}
	i = ftdi_usb_open(&fc, 0x0403, 0x6010);
	if (i < 0 && i != -5) {
		fprintf(stderr, "OPEN: %s\n", ftdi_get_error_string(&fc));
		exit(-1);
	}

	// TODO mode 3 CPOL=1 CPHA=1... but I have to use this to make it work
	ftdispi_open(&fsc, &fc, INTERFACE_A);
	ftdispi_setmode(&fsc, /*cs*/1, /*cpol*/0, /*cpha*/0, /*lsbfirst*/0, /*bitmod*/0, /*gpo*/ 0);
	ftdispi_setclock(&fsc, 1000000);
	ftdispi_setloopback(&fsc, 0);


	res = imu_mpu9250_init(imu);
	if (res != 0) {
		fprintf(stderr, "Chip unreachable: %d\n", res);
		goto end;
	}

	printf("poll\n");
	while (1)
	{
		unsigned flags = 0;
		flags |= BIT(IMU_READ_GYR);
		flags |= BIT(IMU_READ_ACC);
		flags |= BIT(IMU_READ_MAG);
		res = imu->poll(imu, flags);

		{
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
	ftdispi_close(&fsc, 1);

	printf("Returning %d\n", res);
	return res;
}
