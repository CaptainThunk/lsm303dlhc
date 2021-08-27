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

#ifndef LSM303DLHC_H
#define LSM303DLHC_H

#include <iostream>
#include <cstdio>
#include <cstdint>

#define MISSING_FUNC_1  "Error: Adapter does not have "
#define MISSING_FUNC_2  " capability.\n"

// LSM303 Address ###
#define LSM303_ADDRESS_ACCEL 	0x19 // Default Accel address
#define LSM303_ADDRESS_MAG 		0x1e // Default Mag address

// Accel registers

#define LSM303_REGISTER_ACCEL_CTRL_REG1_A 		0x20        // 00000111   rw
#define LSM303_REGISTER_ACCEL_CTRL_REG2_A 		0x21        // 00000000   rw
#define LSM303_REGISTER_ACCEL_CTRL_REG3_A 		0x22        // 00000000   rw
#define LSM303_REGISTER_ACCEL_CTRL_REG4_A 		0x23        // 00000000   rw
#define LSM303_REGISTER_ACCEL_CTRL_REG5_A 		0x24        // 00000000   rw
#define LSM303_REGISTER_ACCEL_CTRL_REG6_A 		0x25        // 00000000   rw
#define LSM303_REGISTER_ACCEL_REFERENCE_A 		0x26        // 00000000   r
#define LSM303_REGISTER_ACCEL_STATUS_REG_A		0x27        // 00000000   r
#define LSM303_REGISTER_ACCEL_OUT_X_L_A 		0x28
#define LSM303_REGISTER_ACCEL_OUT_X_H_A 		0x29
#define LSM303_REGISTER_ACCEL_OUT_Y_L_A			0x2A
#define LSM303_REGISTER_ACCEL_OUT_Y_H_A			0x2B
#define LSM303_REGISTER_ACCEL_OUT_Z_L_A			0x2C
#define LSM303_REGISTER_ACCEL_OUT_Z_H_A			0x2D
#define LSM303_REGISTER_ACCEL_FIFO_CTRL_REG_A	0x2E
#define LSM303_REGISTER_ACCEL_FIFO_SRC_REG_A 	0x2F
#define LSM303_REGISTER_ACCEL_INT1_CFG_A		0x30
#define LSM303_REGISTER_ACCEL_INT1_SOURCE_A		0x31
#define LSM303_REGISTER_ACCEL_INT1_THS_A 		0x32
#define LSM303_REGISTER_ACCEL_INT1_DURATION_A	0x33
#define LSM303_REGISTER_ACCEL_INT2_CFG_A 		0x34
#define LSM303_REGISTER_ACCEL_INT2_SOURCE_A		0x35
#define LSM303_REGISTER_ACCEL_INT2_THS_A		0x36
#define LSM303_REGISTER_ACCEL_INT2_DURATION_A	0x37
#define LSM303_REGISTER_ACCEL_CLICK_CFG_A 		0x38
#define LSM303_REGISTER_ACCEL_CLICK_SRC_A		0x39
#define LSM303_REGISTER_ACCEL_CLICK_THS_A		0x3A
#define LSM303_REGISTER_ACCEL_TIME_LIMIT_A		0x3B
#define LSM303_REGISTER_ACCEL_TIME_LATENCY_A 	0x3C
#define LSM303_REGISTER_ACCEL_TIME_WINDOW_A 	0x3D

//Mag registers

#define LSM303_REGISTER_MAG_CRA_REG_M			0x00
#define LSM303_REGISTER_MAG_CRB_REG_M 			0x01
#define LSM303_REGISTER_MAG_MR_REG_M 			0x02
#define LSM303_REGISTER_MAG_OUT_X_H_M			0x03
#define LSM303_REGISTER_MAG_OUT_X_L_M 			0x04
#define LSM303_REGISTER_MAG_OUT_Z_H_M 			0x05
#define LSM303_REGISTER_MAG_OUT_Z_L_M			0x06
#define LSM303_REGISTER_MAG_OUT_Y_H_M			0x07
#define LSM303_REGISTER_MAG_OUT_Y_L_M			0x08
#define LSM303_REGISTER_MAG_SR_REG_Mg			0x09
#define LSM303_REGISTER_MAG_IRA_REG_M 			0x0A
#define LSM303_REGISTER_MAG_IRB_REG_M 			0x0B
#define LSM303_REGISTER_MAG_IRC_REG_M			0x0C
#define LSM303_REGISTER_MAG_TEMP_OUT_H_M		0x31
#define LSM303_REGISTER_MAG_TEMP_OUT_L_M 		0x32

//LSM303_REGISTER_MAG_CRB_REG_M (Mag Gain) values

#define LSM303_MAGGAIN_1_3						0x20,  // +/- 1.3 
#define LSM303_MAGGAIN_1_9						0x40,  // +/- 1.9
#define LSM303_MAGGAIN_2_5						0x60,  // +/- 2.5
#define LSM303_MAGGAIN_4_0 						0x80,  // +/- 4.0
#define LSM303_MAGGAIN_4_7						0xA0,  // +/- 4.7
#define LSM303_MAGGAIN_5_6						0xC0,  // +/- 5.6
#define LSM303_MAGGAIN_8_1						0xE0   // +/- 8.1

#define WHO_AM_I 0x0F

#define ACCEL_SCALE 2 // +/- 2g
#define TEMPERATURE_CELSIUS_OFFSET 14	// offset for temperature calc (guesswork) originally 18
#define TILT_HEADING_OFFSET 15

#define X 0
#define Y 1
#define Z 2

namespace lsm303dlhcns {

    // Communicates with Pan-Tilt HAT over i2c
    // to control pan, tilt and light functions

    class lsm303dlhc
    {

		public:
			lsm303dlhc();
			~lsm303dlhc();

			bool setup();
			void getMag();
			void getAccel();
			void getTemperature();
			void getHeading();
			void getTiltHeading();
			void update();
			
			int accel_xyz[3] {0, 0, 0};
			int mag_xyz[3] {0, 0, 0};
			double tiltcomp[3] {0, 0, 0};
			double heading = 0;
			double headingDegrees = 0;
			double tiltHeading = 0;
			double tiltHeadingDegrees = 0;
			int temperature = 0;
			double temperatureDegrees = 0;

		private:
			bool i2c_write_byte(int addr, int reg, unsigned char c);
			bool i2c_read_byte(int addr, int reg, unsigned char * c);
			bool check_int_range(int value, int value_min, int value_max);
			int  lsleep(long int ms);
			int twos_comp(int val, int bits);
			bool isMagReady();


			bool _is_setup = false;
			unsigned int _idle_timeout = 500;
			int _i2c_retries = 10;
			unsigned int _i2c_retry_time = 50;
			int _i2c_accel = 0, _i2c_mag = 0, _length = 0;
			unsigned char _buffer[60] = {0};
			
			
	}; // class lsm303dlhc

} // namespace lsm303dlhcns

#endif

