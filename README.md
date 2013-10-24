# KukaYouBot

The Intelligent Robotics Lab's YouBot framework.

## libsoem

libsoem is a fork of the Simple and Open EtherCAT Master library
(http://soem.berlios.de), which implements the EtherCAT specification, a
specification that describes a master/slave model over ethernet. libsoem is used
to control the motors of the Kuka YouBot robot, which are hooked up by the use of
RJ45-cables as slave devices.

This fork introduces the following features:
 - The use of the more standardised autotools as its build system, rather than
   the Makefile and the BAT-script voodoo that was used to build the library
   before.
 - The legacy code has been deleted, as it was poorly written, and there is no
   need for it to remain around any longer.

Use of this fork is recommended, as everything within the framework will be
built against this fork rather than the original work. 

## libtmcl

libtmcl is a library that implements the Trinamic Motion Control Language
protocol, as described by:
 - http://www.technohands.co.jp/products/pdf/TMCL_reference.pdf
 - http://www.trinamic.com/tmctechlibcd/modules/TMCM-1310/TMCM-1310_TMCL_firmware_manual.pdf

The TMCL protocol specifies both the high-level Assembly language that is used
to control the motors, and the communication protocol between the master and the
slave devices. The TMCL protocol is (mostly) physical layer independent, i.e. it
can be used over ethernet using the EtherCAT specification, over USB using the
USB specification, or directly over RS232/RS485.

## libskro

libskro or the Skeletal Robot library is the library that is used as an
abstraction on top of libsoem to offer control over the robot by the use of a
skeletal representation (i.e. through the use of joints and such). The library
mostly acts as a replacement for the youbot-driver.

The following specifications are useful when developing libskro:
 - http://www.technohands.co.jp/products/pdf/TMCL_reference.pdf

