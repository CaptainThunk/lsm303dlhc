/*  
    C++ interface for controlling a Adafruit LSM303DLHC sensor on a Raspberry Pi.
    Class built upon SMBus and i2C standard APIs.

    Works with sensor LSM303DLHC (dicontinued) (https://learn.adafruit.com/lsm303-accelerometer-slash-compass-breakout/which-lsm303-do-i-have) It's the blue one.
    LSM303DLHC Specsheet Reference https://cdn-shop.adafruit.com/datasheets/LSM303DLHC.PDF

    Stand-alone compilation
    -----------------------
    libi2c.so.0.1.0/0.1.1 must exist on the system.
    See build_lsm.sh for building details.

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

#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <cmath>
#include <sys/wait.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <errno.h>
#include "smbus.h"
#include "lsm303dlhc.h"

using namespace std;

namespace lsm303dlhcns {

    // Constructor
    lsm303dlhc::lsm303dlhc()
    {
    }
    // Destructor
    lsm303dlhc::~lsm303dlhc()
    {
		if (_i2c_accel != 0) {
			close(_i2c_accel);
		}
		
		if (_i2c_mag != 0) {
			close(_i2c_mag);
		}
    }

    // public functions
    bool lsm303dlhc::setup()
    {
		if (_is_setup) return true;
		
		// setup communication with magnetometer
		if (_i2c_mag == 0) {
			if ((_i2c_mag = open("/dev/i2c-1", O_RDWR)) < 0) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to open the i2c bus." << endl;
				return false;
			}
			if (ioctl(_i2c_mag, I2C_SLAVE, LSM303_ADDRESS_MAG) < 0) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to access the i2c slave at 0x15." << endl;
				return false;
			}
			// Check functionalities
			unsigned long mFuncs;
			if (ioctl(_i2c_mag, I2C_FUNCS, &mFuncs) < 0) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ 
					<< ": could not get the adapter functionality matrix (errno " 
					<< strerror(errno) << ")."
					<< endl;
				return false;
			}
			if (!(mFuncs & I2C_FUNC_SMBUS_READ_BYTE)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
					<< MISSING_FUNC_1
					<< "SMBus receive byte" << MISSING_FUNC_2 << endl;
				return false;
			}
			if (!(mFuncs & I2C_FUNC_SMBUS_WRITE_BYTE)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
						<< MISSING_FUNC_1
						<< "SMBus send byte" << MISSING_FUNC_2 << endl;
				return false;
			}
			if (!(mFuncs & I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
						<< MISSING_FUNC_1
						<< "SMBus read byte" << MISSING_FUNC_2 << endl;
				return false;
			}
			if (!(mFuncs & I2C_FUNC_SMBUS_READ_WORD_DATA)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
						<< MISSING_FUNC_1
						<< "SMBus read word" << MISSING_FUNC_2 << endl;
				return false;
			}
		}
		
		// setup communication with accelerometer
		if (_i2c_accel == 0) {
			if ((_i2c_accel = open("/dev/i2c-1", O_RDWR)) < 0) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to open the i2c bus." << endl;
				return false;
			}
			if (ioctl(_i2c_accel, I2C_SLAVE, LSM303_ADDRESS_ACCEL) < 0) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to access the i2c slave at 0x15." << endl;
				return false;
			}
			// Check functionalities
			unsigned long aFuncs;
			if (ioctl(_i2c_accel, I2C_FUNCS, &aFuncs) < 0) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ 
					<< ": could not get the adapter functionality matrix (errno " 
					<< strerror(errno) << ")."
					<< endl;
				return false;
			}
			if (!(aFuncs & I2C_FUNC_SMBUS_READ_BYTE)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
					<< MISSING_FUNC_1
					<< "SMBus receive byte" << MISSING_FUNC_2 << endl;
				return false;
			}
			if (!(aFuncs & I2C_FUNC_SMBUS_WRITE_BYTE)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
						<< MISSING_FUNC_1
						<< "SMBus send byte" << MISSING_FUNC_2 << endl;
				return false;
			}
			if (!(aFuncs & I2C_FUNC_SMBUS_READ_BYTE_DATA)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
						<< MISSING_FUNC_1
						<< "SMBus read byte" << MISSING_FUNC_2 << endl;
				return false;
			}
			if (!(aFuncs & I2C_FUNC_SMBUS_READ_WORD_DATA)) {
				cerr << __FILE__ << ":" << __LINE__ << ":" << __func__
						<< MISSING_FUNC_1
						<< "SMBus read word" << MISSING_FUNC_2 << endl;
				return false;
			}
		}

		
		// initialise the Accelerometer - all axis enabled 50hz
		if (!i2c_write_byte(_i2c_accel, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x47))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to initialise accelerometer." << endl;
		
		// set Accelerometer to continuous updating
		if (!i2c_write_byte(_i2c_accel, LSM303_REGISTER_ACCEL_CTRL_REG4_A, 0x00))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to set accelerometer to continuous update." << endl;
		
		// enable temperature sensor at 75hz
		if (!i2c_write_byte(_i2c_mag, LSM303_REGISTER_MAG_CRA_REG_M, 0x98))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to initialise temperature sensor." << endl;
		
		// initialise the Magnetometer
		if (!i2c_write_byte(_i2c_mag, LSM303_REGISTER_MAG_MR_REG_M, 0x00))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to initialise magnetometer." << endl;
				
		_is_setup = true;
		return true;
    }

    void lsm303dlhc::getMag() {
		// self.mag[X] = twos_comp(bus.read_byte_data(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_OUT_X_H_M) << 8 | 
                          // bus.read_byte_data(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_OUT_X_L_M), 16)
		
		unsigned char mHi;
		unsigned char mLo;
		
		// x
		if (!i2c_read_byte(_i2c_mag, LSM303_REGISTER_MAG_OUT_X_H_M, &mHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get magnetometer x hi byte." << endl; 
		if (!i2c_read_byte(_i2c_mag, LSM303_REGISTER_MAG_OUT_X_L_M, &mLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get magnetometer x lo byte." << endl;
		mag_xyz[X] = twos_comp((mHi << 8 | mLo), 16);
		
		// y
		if (!i2c_read_byte(_i2c_mag, LSM303_REGISTER_MAG_OUT_Y_H_M, &mHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get magnetometer y hi byte." << endl; 
		if (!i2c_read_byte(_i2c_mag, LSM303_REGISTER_MAG_OUT_Y_L_M, &mLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get magnetometer y lo byte." << endl;
		mag_xyz[Y] = twos_comp((mHi << 8 | mLo), 16);
		
		// z
		if (!i2c_read_byte(_i2c_mag, LSM303_REGISTER_MAG_OUT_Z_H_M, &mHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get magnetometer z hi byte." << endl; 
		if (!i2c_read_byte(_i2c_mag, LSM303_REGISTER_MAG_OUT_Z_L_M, &mLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get magnetometer z lo byte." << endl;
		mag_xyz[Z] = twos_comp((mHi << 8 | mLo), 16);
		
		// printf("X: %d, Y: %d, Z: %d \r\n", mag_xyz[0], mag_xyz[1], mag_xyz[2]); 				  
	}
	
    void lsm303dlhc::getAccel() {
		unsigned char aHi;
		unsigned char aLo;
		
		// x
		if (!i2c_read_byte(_i2c_accel, LSM303_REGISTER_ACCEL_OUT_X_H_A, &aHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get accelerometer x hi byte." << endl; 
		if (!i2c_read_byte(_i2c_accel, LSM303_REGISTER_ACCEL_OUT_X_L_A, &aLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get accelerometer x lo byte." << endl;
		accel_xyz[X] = twos_comp((aHi << 8 | aLo), 16);
		
		// y
		if (!i2c_read_byte(_i2c_accel, LSM303_REGISTER_ACCEL_OUT_Y_H_A, &aHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get accelerometer y hi byte." << endl; 
		if (!i2c_read_byte(_i2c_accel, LSM303_REGISTER_ACCEL_OUT_Y_L_A, &aLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get accelerometer y lo byte." << endl;
		accel_xyz[Y] = twos_comp((aHi << 8 | aLo), 16);

		// z
		if (!i2c_read_byte(_i2c_accel, LSM303_REGISTER_ACCEL_OUT_Z_H_A, &aHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get accelerometer z hi byte." << endl; 
		if (!i2c_read_byte(_i2c_accel, LSM303_REGISTER_ACCEL_OUT_Z_L_A, &aLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get accelerometer z lo byte." << endl;
		accel_xyz[Z] = twos_comp((aHi << 8 | aLo), 16);

		// printf("X: %d, Y: %d, Z: %d \r\n", accel_xyz[0], accel_xyz[1], accel_xyz[2]);
		
	}
	
	void lsm303dlhc::getTemperature() {
		unsigned char tHi;
		unsigned char tLo;
		
		if (!i2c_read_byte(_i2c_mag, LSM303_REGISTER_MAG_TEMP_OUT_H_M, &tHi))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get accelerometer z hi byte." << endl; 
		if (!i2c_read_byte(_i2c_mag, LSM303_REGISTER_MAG_TEMP_OUT_L_M, &tLo))
			cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": unable to get accelerometer z lo byte." << endl;
		
		temperature = twos_comp(((tHi << 8) | tLo) >> 4, 12);
		
		temperatureDegrees = (temperature / 8.0) + TEMPERATURE_CELSIUS_OFFSET;
	}
	
	void lsm303dlhc::getHeading() { 
	
		heading = 180 * atan2(mag_xyz[Y], mag_xyz[X]) / M_PI;
        if(heading < 0) {
            heading += 360;
		}

        headingDegrees = heading;

        double mtMagX = mag_xyz[X] / 450 * 100.0;
        double mtMagY = mag_xyz[Y] / 450 * 100.0;
        // double mtMagZ = mag_xyz[Z] / 400 * 100.0;	// unused

        heading = atan2(mtMagX, mtMagY);
		
		/* original code - it's 90 deg out */
        // heading = atan2(mag_xyz[X], mag_xyz[Y]);

        // if(heading < 0) {
            // heading += 2*M_PI;
		// }
        // if(heading > 2*M_PI) {
            // heading -= 2*M_PI;
		// }

        
		// headingDegrees = heading * (180.0 / M_PI); 		// radians to degrees
		
	}
	
	void lsm303dlhc::getTiltHeading() {
		double truncate[3] {0, 0, 0};
		double pitch, roll;
		
		truncate[X] = copysign(min(fabs(accel_xyz[X] / pow(2, 15) * ACCEL_SCALE), 1.0), accel_xyz[X] / pow(2, 15) * ACCEL_SCALE);
		truncate[Y] = copysign(min(fabs(accel_xyz[Y] / pow(2, 15) * ACCEL_SCALE), 1.0), accel_xyz[Y] / pow(2, 15) * ACCEL_SCALE);
		truncate[Z] = copysign(min(fabs(accel_xyz[Z] / pow(2, 15) * ACCEL_SCALE), 1.0), accel_xyz[Z] / pow(2, 15) * ACCEL_SCALE);
		// printf("TruncX: %.2f, TruncY: %.2f, TruncZ: %.2f \r\n",truncate[X], truncate[Y], truncate[Z]);
        try
		{	
            pitch = asin(-1*truncate[X]);
            
			// roll = math.asin(truncate[Y]/math.cos(pitch)) if abs(math.cos(pitch)) >= abs(truncate[Y]) else 0
			
			if(abs(cos(pitch)) >= abs(truncate[Y])) {
				roll = asin(truncate[Y]/cos(pitch));
			} else {
				roll = 0.0;
			}
			// printf("Roll: %.4f \r\n",roll);
            // set roll to zero if pitch approaches -1 or 1

            tiltcomp[X] = mag_xyz[X] * cos(pitch) + mag_xyz[Z] * sin(pitch);
            tiltcomp[Y] = mag_xyz[X] * sin(roll) * sin(pitch) + mag_xyz[Y] * cos(roll) - mag_xyz[Z] * sin(roll) * cos(pitch);
            tiltcomp[Z] = mag_xyz[X] * cos(roll) * sin(pitch) + mag_xyz[Y] * sin(roll) + mag_xyz[Z] * cos(roll) * cos(pitch);
            tiltHeading = atan2(tiltcomp[Y], tiltcomp[X]);

            if(tiltHeading < 0) {
                tiltHeading += 2*M_PI;
			}
            if(tiltHeading > (2*M_PI)) {
				heading -= 2*M_PI;
                // tiltHeading -= 2*M_PI;
			}

            tiltHeadingDegrees = tiltHeading * (180.0 / M_PI);

			tiltHeadingDegrees += TILT_HEADING_OFFSET;
			if(tiltHeadingDegrees < 0) {
				tiltHeadingDegrees += 360;
			}
			if(tiltHeadingDegrees >= 360) {
				tiltHeadingDegrees -= 360;
			}
			
        } catch(exception& e) {
			printf("AccelX: %.2f, AccelY: %.2f \r\n",accel_xyz[X] / pow(2, 15) * ACCEL_SCALE, accel_xyz[Y] / pow(2, 15) * ACCEL_SCALE);
			printf("TruncX: %.2f, TruncY: %.2f \r\n",truncate[X], truncate[Y]);
			// printf("Pitch: %.2f, cos(pitch): %.2f, Bool(cos(pitch)): %s \r\n",pitch, cos(pitch, bool(cos(pitch))));
			printf("Pitch: %.2f, cos(pitch): %.2f, Bool(cos(pitch)): %i  \r\n", pitch, cos(pitch), bool(cos(pitch)));
		}
	}
	
	void lsm303dlhc::update() {
		setup();
		getMag();
		getAccel();
		getTemperature();
		getHeading();
		getTiltHeading();
	}

   
	// Private functions

	int lsm303dlhc::twos_comp(int val, int bits)
	{
		// # Calculate the 2s complement of int:val #
		if ((val&(1 << (bits-1))) != 0)
		{
			val = val - (1 << bits);
		}

		return val;
	}
	
	bool lsm303dlhc::isMagReady() 
	{
		return true;
	}
	
    // Write one byte to the i2c device
    bool lsm303dlhc::i2c_write_byte(int addr, int reg, unsigned char c)
    {
		if (!check_int_range(c, 0, 0xFF)) return false;
		
        for (int i = 0; i < _i2c_retries; i++)
        {
            if (i2c_smbus_write_byte_data(addr, reg, c) >= 0) return true;
            lsleep(_i2c_retry_time);
        }
        cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to write byte to the i2c device." << endl;
        return false;
    }

    // Read one byte from the i2c device
    bool lsm303dlhc::i2c_read_byte(int addr, int reg, unsigned char * c)
    {
		int res = -1;
        for (int i = 0; i < _i2c_retries; i++)
		{
			if ((res = i2c_smbus_read_byte_data(addr, reg)) >= 0)
			{
			*c = (res & 0xFF);
			return true;
			}
				lsleep(_i2c_retry_time);
		}
		cerr << __FILE__ << ":" << __LINE__ << ":" << __func__ << ": failed to read byte from the i2c device." << endl;
		return false;
    }

    // Suspend this thread for ms milliseconds.
    int lsm303dlhc::lsleep(long int ms)
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

	// Check bounds of an int value.
    bool lsm303dlhc::check_int_range(int value, int value_min, int value_max)
    {
        return ((value >= value_min) && (value <= value_max));
    }
} //namespace lsm303dlhcns
