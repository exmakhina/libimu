######
libimu
######


This is yet another library to access MARG devices, but this one is meant
to be portable, ie. it shouldn't depend on much more than a not-too-bad libc,
shouldn't contain tranport code, and yes it doesn't contain sensor fusion
stuff.


Design
######

Sensors are providing a structure deriving from ``struct imu``, and implement
the functions in there.

Clients provide some OS functions (how to sleep, how to get a timestamp,
how to access the sensor using OS/system facilities (typ. I2C or SPI).

Clients retrieve the engineering values, and use them to perform their
sensor fusion.

The clients can either build the ``.c`` files, or include the ``.i`` files
in their own C files to have inline code.


Installation
############

The library can be installed on a UNIX system and you can
``#include imu/imu_${type}.{h,i}``.

Regular build uses the waf build system, but you're free to use another one
if you feel like it, it should be simple enough.


Usage
#####

See the ``_test.c`` files.


License
#######

MIT License, with attribution of your contributed bits.



