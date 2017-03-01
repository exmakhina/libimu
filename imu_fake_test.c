#include <time.h>
#include <stdio.h>
#include <inttypes.h>

#include <imu_fake.i>


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
	return 0;
}


int imut_write(imu_ctx_t ctx, int reg, uint8_t const * buf, int buflen)
{
	return 0;
}


int imut_now(imu_ctx_t ctx, int64_t * now)
{
	struct timespec ts;
	int res;
	res = clock_gettime(CLOCK_MONOTONIC, &ts);
	*now = ((uint64_t)ts.tv_sec * (uint64_t)1000000000) + ts.tv_nsec;
	return res;
}


int main(void)
{
	int res;

	struct imu_fake theimu;
	struct imu * imu = (struct imu*)&theimu;

	imu->sleep = imut_sleep;
	imu->tread = imut_read;
	imu->twrite = imut_write;
	imu->now = imut_now;

	imu_fake_init(imu);

	res = imu->poll(imu, 1<<IMU_READ_ACCEL);

	float ax, ay, az;
	imu_abstime_t t;
	res = imu->get_acc(imu, &t, &ax, &ay, &az);

	printf("t=%" PRIimuabstime " ax=%7.3f ay=%7.3f az=%7.3f\n", t, ax, ay, az);

}
