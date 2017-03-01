#ifndef LIBIMU_IMU_FAKE_H
#define LIBIMU_IMU_FAKE_H

#include <stdio.h>

#if !defined(IMU_EXPORT)
#define IMU_EXPORT
#endif

#include "imu.h"

struct imu_fake {
	struct imu base;
	imu_abstime_t t_cap;
};

IMU_EXPORT int imu_fake_init(struct imu * imu);

#endif // LIBIMU_IMU_FAKE_H

