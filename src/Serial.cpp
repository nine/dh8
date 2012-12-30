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



#include "Serial.h"
#include <stdio.h>
#include <fstream>
#include <sstream>
#include <strings.h>   // bzero()

Serial::Serial()
{
}

Serial::~Serial()
{
}


/*                                                                  */
/*																																	*/
/* void InitSerial(..)																							*/
/*																																	*/
/*------------------------------------------------------------------*/
/* Input values:																										*/
/*  	BaudRate =  4800 9600 | 19200 | 38400 | 57600 | 115200 | 230400 		*/
/*		Parity = not implemented (yet)																*/
/*    Stopbits = not implemented (yet)															*/
/*																																	*/
/*------------------------------------------------------------------*/
/* Return values:																										*/
/*   1 : its allright, serial port was initialized									*/
/*   0 : serial port couldn´t be opended 													  */
/*  -1 : invalid baudrate																						*/
/*																																	*/
/*                                                                  */


int Serial::InitSerial(std::string device, int BaudRate, int Parity, int StopBits)
{
  struct termios newtio;
  int baud;

  switch (BaudRate)
		{
		case 4800:
			baud = B4800;
			break;
    case 9600:
      baud = B9600;
      break;
    case 19200:
      baud = B19200;
      break;
    case 38400:
      baud = B38400;
      break;
    case 57600:
      baud = B57600;
      break;
    case 115200:
      baud = B115200;
      break;
    case 230400:
      baud = B230400;
      break;
    default:
      //DEBUG_ERROR("Serial: Invalid baudrate.\n");
      return -1;
  }


  serial = open(device.c_str(), O_RDWR | O_NOCTTY);
	
  if (serial < 0)
		{
			//DEBUG_ERROR("Serial: Serial interface couldn´t be opened.\n");
			return 0;
		}
	
  bzero(& newtio, sizeof(termios));
  newtio.c_cflag = baud | /*CRTSCTS |*/ CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
	
  newtio.c_cc[VTIME] = 0; //inter-character timer unused
  newtio.c_cc[VMIN] = 1; //blocking read until 1 chars received
	
  tcflush(serial, TCIFLUSH);
  tcsetattr(serial, TCSANOW, & newtio);
	
	
  return 1;
}


/*                                                                  */
/*																																	*/
/* void WaitForSingleByte(..)																				*/
/*																																	*/
/*------------------------------------------------------------------*/
/* Input values: 																										*/
/*			int T_sec: time (secondspart) to wait for a byte						*/
/*			int T_usec: time (millisecondspart) to wait for a byte			*/
/*																																	*/
/*------------------------------------------------------------------*/
/* Output values:																										*/
/*			unsigned char &RxByte: Received Byte												*/
/*      double &RxTime: How long did it last?												*/
/*																																	*/
/*------------------------------------------------------------------*/
/* Return values:																										*/
/*   -1 : an error occured 																					*/
/*    0 : no byte was read							 													  */
/*	  1 : one byte was read																					*/
/*																																	*/
/*                                                                  */

int Serial::WaitForSingleByte(unsigned char & RxByte, double & RxTime, int T_sec, int T_usec)
{
  fd_set read_fds;
  struct timeval timeout;
  int ret;


  timeout.tv_sec = T_sec;
  timeout.tv_usec = T_usec;

  FD_ZERO(& read_fds); // clear read_fds set
  FD_SET(serial, & read_fds); // add m_FD to set

  ret = select(serial + 1, & read_fds, NULL, NULL, & timeout); // wait for data available on descriptor m_FD or timeout
  if (ret > 0 && FD_ISSET(serial, & read_fds)) return read(serial, & RxByte, 1);
  else return -1;
  return 1;
}

/*                                                                  */
/*																																	*/
/* void SendByte(..)																								*/
/*																																	*/
/*------------------------------------------------------------------*/
/* Input values: 																										*/
/*			int T_sec: time (secondspart) to wait for a byte						*/
/*			int T_usec: time (millisecondspart) to wait for a byte			*/
/*			char TxByte: Byte that should be transmitted								*/
/*																																	*/
/*------------------------------------------------------------------*/
/* Return values:																										*/
/*   -1 : an error occured 																					*/
/*    0 : no byte was written						 													  */
/*	  1 : one byte was written																			*/
/*																																	*/
/*                                                                  */

int Serial::SendByte(unsigned char TxByte, int T_sec, int T_usec)
{
  fd_set write_fds;
  struct timeval timeout;
  int ret;


  //std::cout << hex << static_cast<int>(TxByte) << std::endl;
  timeout.tv_sec = T_sec;
  timeout.tv_usec = T_usec;

  FD_ZERO(& write_fds); // clear read_fds set
  FD_SET(serial, & write_fds); // add m_FD to set

  ret = select(serial + 1, NULL, & write_fds, NULL, & timeout); // wait for data available on descriptor m_FD or timeout
  if (ret > 0 && FD_ISSET(serial, & write_fds)) return write(serial, (void *) & TxByte, 1);
  else return ret;
}
