# KukaYouBot

The Intelligent Robotics Lab's YouBot framework.

## libsoem

libsoem is a fork of the Simple and Open EtherCAT Master library
http://soem.berlios.de, which is the most fundamental piece within the
framework as it allows us to detect and configure the slave devices of the
robot via the EtherCAT specification.

This fork introduces the following features:
 - The use of autotools as its build system, rather than the Makefile and
   BAT-script voodoo it used to be.

Use of this fork is recommended, as everything within the framework will be
built against this fork rather than the original work. 

## libskro

libskro or the Skeletal Robot library is the library that is used as an
abstraction on top of libsoem to offer control over the robot by the use of a
skeletal representation (i.e. through the use of joints and such). The library
mostly acts as a replacement for the youbot-driver.

The following specifications are useful when developing libskro:
 - http://www.technohands.co.jp/products/pdf/TMCL_reference.pdf

