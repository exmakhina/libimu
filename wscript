#! /usr/bin/env python
# -*- coding: utf-8 vi:noet

def options(opt):
	opt.load("compiler_c")
	opt.add_option("--build-fake", action="store_true", default=False)
	opt.add_option("--build-mpu9250", action="store_true", default=False)


def configure(conf):
	conf.load("compiler_c")
	if conf.options.build_fake:
		conf.env.BUILD_FAKE = True
	if conf.options.build_mpu9250:
		conf.env.BUILD_MPU9250 = True

def build(bld):
	bld.install_files(
	 "${PREFIX}/include/libimu",
	 [
	  "imu.h",
	 ],
	)
	if bld.env.BUILD_FAKE:
		bld.install_files(
		 "${PREFIX}/include/libimu",
		 [
		  "imu_fake.h",
		  "imu_fake.i",
		 ],
		)
		bld(
		 target="imu_fake",
		 features="c cshlib",
		 includes=[bld.path.abspath()],
		 source="imu_fake.c",
		)
		bld(
		 target="imu_fake_testi",
		 features="c cprogram",
		 includes=[bld.path.abspath()],
		 source="imu_fake_test.c",
		)

	if bld.env.BUILD_MPU9250:
		bld.install_files(
		 "${PREFIX}/include/libimu",
		 [
		  "imu_mpu9250.h",
		  "imu_mpu9250.i",
		 ],
		)
		bld(
		 target="imu_mpu9250",
		 features="c cshlib",
		 includes=[bld.path.abspath()],
		 source="imu_mpu9250.c",
		)

		if 0:
			bld(
			 target="imu_mpu9250_test",
			 features="c cprogram",
			 includes=[bld.path.abspath()],
			 source=["imu_mpu9250_test.c", "ftdispi.c"],
			 lib="ftdi",
			)

		if 0:
			bld(
			 target="imu_mpu9250_test_i2c",
			 features="c cprogram",
			 includes=[bld.path.abspath()],
			 source=["imu_mpu9250_test_i2c.c", "ftdii2c.c", "ftdicommon.c"],
			 lib="ftdi",
			)

		if 0:
			bld(
			 target="imu_mpu9250_test_i2c_rpi",
			 features="c cprogram",
			 includes=[bld.path.abspath()],
			 source=["imu_mpu9250_test_i2c_rpi.c"],
			)
