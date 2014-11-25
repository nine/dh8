/*
 *  dh8 
 *
 *  Control software for the Diavite DH-8 surface roughness measurement 
 *  device.
 *
 *
 *  Copyright (C) 2012 Erwin Nindl <nine-dh8@wirdorange.org>
 *
 *  This file is part of DH8.
 *
 *  DH8 is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  any later version.
 *
 *  DH8 is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with DH8.  If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef DH8_DEFS_h_INCLUDED
#define DH8_DEFS_h_INCLUDED


#define APPNAME "dh8 - DH-8 remote control"
#define VERSION "0.1"


// gain factor between z-values and 
// real valuses in micro meter
#define DH8_Z_FACTOR              128


// serial commands:
//--------------------------------------
// terminal characters
#define ESC '\x1b'
#define ETX '\x03'
#define ACK '\x06'
#define EOT '\x04'

#define DH8_READ_DEVICE    "0"
#define DH8_GET_R_VALUES  "@"

#define DH8_START_MEASURE         ">"
#define DH8_READ_Z_VALUES         "<"
#define DH8_READ_FILT_Z_VALUES    "\x2b"
#define DH8_READ_PRO_VALUES       "I"
#define DH8_READ_Z_REFERENCE      "\x3b"
#define DH8_READ_CAL_Z            "\x45"
#define DH8_READ_VERSION          "\x31"
#define DH8_READ_UNIT             "\x32"
#define DH8_READ_NORM             "\x33"
#define DH8_READ_TOLERANCE        "\x34"

#define DH8_SET_FMT_BIN           "U0"
#define DH8_SET_FMT_ASCII         "U1"
#define DH8_SET_LT                "P"
#define DH8_SET_LC                "Q"

#define DH8_SET_CAL_Z             "\x65"



//--------------------------------------

#endif // guards  
