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


static int imut_sleep(imu_ctx_t ctx, int64_t dt)
{
	int res;
	printf("sleep %" PRIu64 "\n", dt);
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

#if 0
	printf("Read %p %02x\n", fsc, reg);
#endif
	uint8_t const txbuf[] = { (1<<7) | reg };
	res = ftdispi_write_read(fsc, txbuf, sizeof(txbuf), buf, buflen, 0);
#if 0
	for (int i = 0; i < buflen; i++) {
		printf(" %02x", buf[i]);
	}
	printf("\n");
#endif
	return res;
}


int imut_write(imu_ctx_t ctx, int reg, uint8_t const * buf, int buflen)
{
	struct ftdispi_context * fsc = (struct ftdispi_context*)ctx;
	int res;

	uint8_t const txbuf[] = { (1<<7) | reg };
	printf("Write %02x\n", reg);
	res = ftdispi_write(fsc, txbuf, sizeof(txbuf), 0);
	res = ftdispi_write(fsc, buf, buflen, 0);
	for (int i = 0; i < buflen; i++) {
		printf(" %02x", buf[i]);
	}
	printf("\n");
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
		res = imu->poll(imu, BIT(IMU_READ_GYR)|BIT(IMU_READ_ACC));

		{
			imu_abstime_t t;
			float ax, ay, az;
			res = imu->get_acc(imu, &t, &ax, &ay, &az);
			printf("t=%" PRIimuabstime " ax=%7.3f ay=%7.3f az=%7.3f (m/s²)\n", t, ax, ay, az);
		}

#if 0
		{
			imu_abstime_t t;
			float gx, gy, gz;
			res = imu->get_gyr(imu, &t, &gx, &gy, &gz);
			printf("t=%" PRIimuabstime " gx=%7.3f gy=%7.3f gz=%7.3f (rad/s)\n", t, gx, gy, gz);
		}
#endif
	}

end:
	ftdispi_close(&fsc, 1);

	printf("Returning %d\n", res);
	return res;
}
