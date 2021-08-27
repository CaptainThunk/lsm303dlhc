/*  
    This program checks a Adafruit LSM303DLHC sensor device plugged into a Raspberry Pi.
    Works with sensor LSM303DLHC (dicontinued) (https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/which-lsm303-do-i-have) It's the blue one.
    LSM303DLHC Specsheet Reference https://cdn-shop.adafruit.com/datasheets/LSM303DLHC.PDF

    Stand-alone compilation
    -----------------------
    libi2c.so.0.1.0/0.1.1 must exist on the system.
    See build_cp.sh for building details.

    Copyright (C) 2021 Captain Thunk

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
    MA 02110-1301 USA.
*/

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <iostream>
#include <exception>
#include <new>
#include <time.h>
#include "lsm303dlhc.h"

#define OK		"OK !\n"
#define NOK		"NOK !\n"
#define ONESECOND	1000
#define FIFTYMILLIS	50

using namespace std;
using namespace lsm303dlhcns;

// Suspend this thread for ms milliseconds.
int lsleep(long ms)
{
    int result = 0;
    struct timespec ts_sleep, ts_remaining;
    ts_remaining.tv_sec = (time_t)(ms / 1000);
    ts_remaining.tv_nsec = (long)(ms % 1000) * 1000000;
    do
    {
	ts_sleep = ts_remaining;
        result = nanosleep(&ts_sleep, &ts_remaining);
    }
    while ((errno == EINTR) && (result == -1));
    if (result == -1)
    {
        cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << " nanosleep() failed" << endl;
    }
    return result;
}

int main()
{
    int err = -1;

    try
    {
		lsm303dlhc * lsm = new lsm303dlhc();
		
		printf("\n[INFO] Checking LSM303DLHC Module...\n\n");
		
		if (!lsm->setup())
		{
			cerr << "[FATAL] Could not initialize LSM303DLHC module. Aborting.\n" << endl;
			goto error;
		}
		
		lsm->update();
		// lsm->getAccel();
		// lsm->getTemperature();
		
		printf("mX: %d, mY: %d, mZ: %d \r\n", lsm->mag_xyz[X], lsm->mag_xyz[Y], lsm->mag_xyz[Z]);
		printf("aX: %f, aY: %f, aZ: %f \r\n", lsm->accel_xyz[X] / pow(2, 15) * ACCEL_SCALE, lsm->accel_xyz[Y] / pow(2, 15) * ACCEL_SCALE, lsm->accel_xyz[Z] / pow(2, 15) * ACCEL_SCALE);
		printf("Temp: %d, Temp C: %.1f \r\n", lsm->temperature, lsm->temperatureDegrees);
		printf("Heading: %.4f \r\n", lsm->headingDegrees);
		printf("Tilt Heading: %.4f \r\n", lsm->tiltHeadingDegrees);
error:
		// Get rid of LSM object and frees memory.
		if (lsm) delete lsm;
		
		
		
	// Getting to this point asserts no error occurred
	err = 0;
	printf("\n[SUCCESS] LSM303DLHC module is fully operational.\n\n");


	} catch(exception& e) {
		cerr << "[FATAL] could not create LSM303DLHC instance class : "
			<< e.what()
			<< "Aborting.\n" << endl;
			exit(-1);
	}

    return err;
}
