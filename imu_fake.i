#if !defined(IMU_EXPORT)
#define IMU_EXPORT static inline
#endif

#include "imu_fake.h"

IMU_EXPORT int imu_read(struct imu * imu, unsigned flags)
{
	struct imu_fake * fake = (struct imu_fake*)imu;
	printf("Fake read\n");
	imu->now(imu->ctx, &fake->t_cap);
}

IMU_EXPORT int imu_get_acc(struct imu * imu, imu_abstime_t * at, float * ax, float * ay, float * az)
{
	struct imu_fake * fake = (struct imu_fake*)imu;
	*at = fake->t_cap;
	*ax = 0.f;
	*ay = 0.f;
	*az = 9.81f;
	return 0;
}

IMU_EXPORT int imu_get_gyr(struct imu * imu, imu_abstime_t * gt, float * gx, float * gy, float * gz)
{
	struct imu_fake * fake = (struct imu_fake*)imu;
	*gt = fake->t_cap;
	*gx = *gy = *gz = 0;
	return 0;
}

IMU_EXPORT int imu_get_mag(struct imu * imu, imu_abstime_t * mt, float * mx, float * my, float * mz)
{
	struct imu_fake * fake = (struct imu_fake*)imu;
	*mt = fake->t_cap;
	*mx = 5e-4f;
	*my = 0.f;
	*mz = 0.f;
	return 0;
}


IMU_EXPORT int imu_fake_init(struct imu * imu)
{
	struct imu_fake * fake = (struct imu_fake*)imu;
	printf("Fake init\n");

	imu->poll = &imu_read;
	imu->get_acc = &imu_get_acc;
	imu->get_gyr = &imu_get_gyr;
	imu->get_mag = &imu_get_mag;

	imu->sleep(imu->ctx, 0);
}

