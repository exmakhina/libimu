#! /usr/bin/env python
# -*- coding: utf-8 vi:noet

def options(opt):
	opt.load("compiler_c")
	opt.add_option("--build-fake", action="store_true", default=False)


def configure(conf):
	conf.load("compiler_c")
	if conf.options.build_fake:
		conf.env.BUILD_FAKE = True

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
