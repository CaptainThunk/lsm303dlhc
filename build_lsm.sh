#!/bin/bash

# Builds lsm303dlhc.o (interface to the lsm303dlhc mag/accel/temp sensor) and
# hansi_check (exploitation program) files.

# For this compilation to succeed, those packages must be installed :
# sudo apt-get install i2c-tools
# http://ftp.fr.debian.org/debian/pool/main/i/i2c-tools/libi2c0_4.1-1_armhf.deb
# sudo apt-get install libi2c-dev

# BEWARE ! The /usr/include/linux/i2c-dev.h file MUST be this one :
# https://github.com/torvalds/linux/blob/master/include/uapi/linux/i2c-dev.h

CFLAGS="-fpic -D_REENTRANT -O2 -Wall -Wextra -pedantic"
gcc ${CFLAGS} -c lsm303dlhc.cpp -o lsm303dlhc.o
g++ ${CFLAGS} check_lsm.cpp -o check_lsm lsm303dlhc.o /usr/lib/arm-linux-gnueabihf/libi2c.so.0
strip check_lsm

rm *.o

exit 0
