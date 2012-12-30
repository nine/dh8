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

#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <unistd.h>

class Serial
{
private:

  //serial port
  int serial;
	
  /*!<File descriptor for the serial port we use*/
	
public:
  Serial();
  ~Serial();
	
  /** Initialize the Serial interface */
  int InitSerial(std::string device, int BaudRate, int Parity, int StopBits);
	
  /** Read a byte from the serial interface */
  int WaitForSingleByte(unsigned char & RxByte, double & RxTime, int T_sec = 0, int T_usec = 100000);

  int SendByte(unsigned char TxByte, int T_sec = 0, int T_usec = 100000);
};
