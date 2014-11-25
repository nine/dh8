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


#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <unistd.h>
#include <sstream>

#include "SerialCmd.h"
#include "dh8Data.h"
#include "dh8defs.h"
#include "myException.h"


SerialCmd::SerialCmd() : 
  SerialPort_("/dev/ttyUSB0"), 
  Cmd_(""),
  TimeOut_(200),                 // default timeout
  BdRate_(115200),
  blocking_(false),              // don't wait for completion 
  desc_("Allowed options"),
  OutFile_("measure.m"),
  Lt_("15"),
  Lc_("0")
{

  Lc_map_["0"] =    0;
  Lc_map_["0.08"] = 1;
  Lc_map_["0.25"] = 2;
  Lc_map_["0.8"] =  3;
  Lc_map_["2.5"] =  4;

  Lt_map_["0.48"] = 0;
  Lt_map_["1.5"]  = 1;
  Lt_map_["4.8"]  = 2;
  Lt_map_["15"]   = 3;

}

SerialCmd::~SerialCmd()
{
}


void
SerialCmd::configure(int argc, char **argv) 
{
  desc_.add_options()
    ("help,h",
     "produce help message")
    ("dev,d", po::value(&SerialPort_)->default_value(SerialPort_),
     "select the serial device" )
    ("bdrate,b", po::value<int>(&BdRate_)->default_value(BdRate_),
     "set the baudrate. NOTE: other serial parameters default to 8N1" )
    ("timeout,t", po::value<int>(&TimeOut_)->default_value(TimeOut_), 
     "set the allowed timeout in miliseconds" )
    ("lt", po::value(&Lt_)->default_value(Lt_),
     "set the traversing length in mm. possible values: 0.48, 1.5, 4.8, 15")
    ("lc", po::value(&Lc_)->default_value(Lc_),
     "set the cutoff length in mm. possible values: 0.08, 0.25, 0.8, 2.5")
    ("file,f", po::value(&OutFile_)->default_value(OutFile_),
     "write results to specific filename")
    ("readout-only,r", 
     "perform value readout only")
    ;

  po::store(po::parse_command_line(argc, argv, desc_), args_);
  po::notify(args_);
  
  if( args_.count("help") ) {
    std::cout << "This is " << APPNAME << " version " << VERSION << std::endl;
    std::cout << "Usage: options_description [options]" << std::endl;
    std::cout << desc_ << std::endl;
    throw(MyException("All OK", E_SUCCESS));
  }

  if( args_.count("lt") && !Lt_map_.count(Lt_) ) {
    throw(MyException("Option lt out of range", DH8_ERROR_PARAMNOTALLOWED));
  }
  if( args_.count("lc") && !Lc_map_.count(Lc_) ) {
    throw(MyException("Option lc out of range", DH8_ERROR_PARAMNOTALLOWED));
  } 
  Data_.Lt_ = Lt_;
  Data_.Lc_ = Lc_;
}


void SerialCmd::initDevice(void)
{
  // Initialize DH8
  //---------------------------------------------------------------------
  if(serial_.InitSerial(SerialPort_, BdRate_, 0, 1) < 1) {
    throw(MyException("Opening the serial device", DH8_ERROR_SERIALPORT));
  }
  if( sendCommandCheckReply(DH8_READ_DEVICE, Data_.Device_, TimeOut_) < 0 ) {
    throw(MyException("Wrong measurement device", DH8_ERROR_DEVICE));
  }
  if( sendCommand( DH8_READ_VERSION, Data_.Version_, TimeOut_ ) < 0 ) {
    throw(MyException("Reading DH8 device version", DH8_ERROR_DEVICEVERSION));
  }
  if( sendCommand( DH8_READ_UNIT, Data_.Unit_, TimeOut_ ) < 0 ) {
    throw(MyException("Reading measurement unit from DH8", DH8_ERROR_EXECCMD));
  }
  if( sendCommand( DH8_READ_NORM, Data_.Norm_, TimeOut_ ) < 0 ) {
    throw(MyException("Reading norm from DH8", DH8_ERROR_EXECCMD));
  }
  if( sendCommand( DH8_READ_TOLERANCE, Data_.Tolerance_, TimeOut_ ) < 0 ) {
    throw(MyException("Reading tolerance from DH8", DH8_ERROR_EXECCMD));
  }

  if( !args_.count("readout-only") ) {
    // set measure params
    std::string parm = DH8_SET_LT;
    parm.push_back(Lt_map_[Lt_]);
    if( sendCommandCheckReply(parm, "", TimeOut_) < 0 ) {
      throw(MyException("Setting Lt on DH8", DH8_ERROR_EXECCMD));
    }

    parm = DH8_SET_LC;
    parm.push_back(Lc_map_[Lc_]);
    if( sendCommandCheckReply(parm, "", TimeOut_) < 0 ) {
      throw(MyException("Setting Lc on DH8", DH8_ERROR_EXECCMD));
    }
  }
}


void
SerialCmd::runApplication(void)
{
  // Perform measurement
  if( !args_.count("readout-only") ) {
    if( sendCommandCheckReply(DH8_START_MEASURE, "", TimeOut_) < 0 ) {
      throw(MyException("Starting measurent", DH8_ERROR_PERFORM_MEASUREMENT));
    }
  }
  // comma-separated string
  std::string response; 
  if( sendCommand( DH8_GET_R_VALUES, response, TimeOut_ ) < 0 ) {
    throw(MyException("Reading R-values from DH8", DH8_ERROR_READDATA));
  }
  Data_.parseValues( response );
  
  if( sendCommand(DH8_READ_Z_REFERENCE, Data_.zReference_, TimeOut_) < 0 ) {
    throw(MyException("Reading z reference from DH8", DH8_ERROR_READDATA));
  }
  if( sendCommand(DH8_READ_CAL_Z, Data_.calZ_, TimeOut_) < 0 ) {
    throw(MyException("Reading cal_z from DH8", DH8_ERROR_READDATA));
  }

  if( sendCommandCheckReply(DH8_SET_FMT_BIN, "", TimeOut_) < 0 ) {
    throw(MyException("Setting transmission format", DH8_ERROR_EXECCMD));
  }
  if( sendCommandReadValues(DH8_READ_Z_VALUES, Data_.zValues_, TimeOut_) < 0) {
    throw(MyException("Reading Z values from DH8", DH8_ERROR_READDATA));
  }
  if( sendCommandReadValues(DH8_READ_FILT_Z_VALUES, Data_.zValuesFilt_, TimeOut_) < 0) {
    throw(MyException("Reading filtered Z values from DH8", DH8_ERROR_READDATA));
  }
  // Not used:
  //  if( sendCommandReadValues2(DH8_READ_PRO_VALUES, proValues_, proPositions_, TimeOut_) < 0) {
  //    std::cerr << "error reading PRO values" << std::endl;
  //    exit(-1);
  //  }

  // write to file
  Data_.writeMFile( OutFile_ );
}


void
SerialCmd::sendStringWithDelimiters(std::string const & tx_buffer) {
  // send bytes, no echo
  serial_.SendByte( ESC );
  for( auto const& itSend: tx_buffer ) {
    serial_.SendByte( itSend );
  }
  serial_.SendByte( ETX );
}


int 
SerialCmd::sendCommandReadValues2(std::string const & stringToSend, std::vector<int> & valuesReceived, std::vector<int> & posReceived, int TimeOut) {
  unsigned char RxByte;
  unsigned int gain = 0;
  int valCount = 0;
  unsigned int valBuffer = 0;
  double RxTime;

  sendStringWithDelimiters( stringToSend );

  // response format: <ACK [1]><GAIN [2]><VAL_COUNT [2]> values <EOT>
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);  // Timeout on receiving <ACK>
  if( RxByte != ACK ) return(-1);  // received no <ACK>
 
  // gain
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1); 
  gain = static_cast<unsigned int>(RxByte);  // convert char to int
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);
  gain += static_cast<unsigned int>(RxByte)<<8;
//  std::cout << "gain: " << gain << std::endl;
  
  // count 
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);
  valCount = static_cast<int>(RxByte);
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);
  valCount += static_cast<int>(RxByte)<<8;
//  std::cout << "count: " << valCount << std::endl;

  // values
  for( int i=0; i<valCount; i++ ) {
    if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);
    valBuffer = static_cast<int>(RxByte);
    if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);
    valBuffer += static_cast<int>(RxByte)<<8;

    valuesReceived.push_back(valBuffer);
  }
  
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) == -1) return(-1);  // Timeout on receiving <EOT>
  gain = static_cast<unsigned int>(RxByte);  // convert char to int
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);
  gain += static_cast<unsigned int>(RxByte)<<8;
//  std::cout << "gain: " << gain << std::endl;
  
  // count [2]
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);
  valCount = static_cast<int>(RxByte);
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);
  valCount += static_cast<int>(RxByte)<<8;
//  std::cout << "count: " << valCount << std::endl;
  
  // values [2]
  for( int i=0; i<valCount; i++ ) {
    if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);
    valBuffer = static_cast<int>(RxByte);
    if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);
    valBuffer += static_cast<int>(RxByte)<<8;

    posReceived.push_back(valBuffer);
  }
//  std::cout << "all values parsed" << std::endl;
  
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) == -1) return(-1);  // Timeout on receiving <EOT>
//  std::cout << "eot: " << static_cast<int>(RxByte) << std::endl;
  if( RxByte != EOT ) return(-1);  // received no <EOT>

  return 0;
}


int 
SerialCmd::sendCommandReadValues(std::string const & stringToSend, std::vector<int> & valuesReceived, int TimeOut) {
  unsigned char RxByte;
  unsigned int gain = 0;
  int valCount = 0;
  unsigned int valBuffer = 0;
  double RxTime;

  sendStringWithDelimiters( stringToSend );
  
  // response format: <ACK [1]><GAIN [2]><VAL_COUNT [2]> values <EOT>
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);  // Timeout on receiving <ACK>
  if( RxByte != ACK ) return(-1);  // received no <ACK>
 
  // gain
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1); 
  gain = static_cast<unsigned int>(RxByte);  // convert char to int
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);
  gain += static_cast<unsigned int>(RxByte)<<8;
//  std::cout << "gain: " << gain << std::endl;
  
  // count 
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);
  valCount = static_cast<int>(RxByte);
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);
  valCount += static_cast<int>(RxByte)<<8;
//  std::cout << "count: " << valCount << std::endl;

  // values
  for( int i=0; i<valCount; i++ ) {
    if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);
    valBuffer = static_cast<int>(RxByte);
    if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) < 0 ) return(-1);
    valBuffer += static_cast<int>(RxByte)<<8;

    valuesReceived.push_back(valBuffer);
  }
//  std::cout << "all values parsed" << std::endl;
  
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut_, 0) == -1) return(-1);  // Timeout on receiving <EOT>
//  std::cout << "eot: " << static_cast<int>(RxByte) << std::endl;
  if( RxByte != EOT ) return(-1);  // received no <EOT>

  return 0;
}


int 
SerialCmd::sendCommand(std::string const & stringToSend, std::string & stringReceived, int TimeOut)
{
  unsigned char RxByte;
  double RxTime;
  std::stringstream rxBuffer;
  const int count = 400;
 
  sendStringWithDelimiters( stringToSend );
  
  // response format: <ACK> answer <EOT>
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut, 0) == -1) return(-1);  // Timeout on receiving <ACK>
  if( RxByte != ACK ) return(-1);  // received no <ACK>

  // read the response
  int i=0;
  for( i=0; i<count; i++ ) {
    if(serial_.WaitForSingleByte(RxByte, RxTime, TimeOut, 0) == -1) return(-1);  // Timeout on receiving 
    if( RxByte == EOT ) break;  // <EOT> end of transmission
    rxBuffer << static_cast<const char>(RxByte);
  }
  if( i > count-2 ) return(-1);  // ultra-long response

  stringReceived = rxBuffer.str();

  return 0;
}


int
SerialCmd::sendCommandCheckReply(std::string const & stringToSend, std::string const & responseToCheck, int TimeOut)
{
  unsigned char RxByte;
  double RxTime;
  std::stringstream rxBuffer;
  
  sendStringWithDelimiters( stringToSend );
  
  // response format: <ACK> answer <EOT>
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut, 0) == -1) return(-1);  // Timeout on receiving <ACK>
  if( RxByte != ACK ) return(-1);  // received no <ACK>

	// check the response
  for( auto const& itRec: responseToCheck ) {
    if(serial_.WaitForSingleByte(RxByte, RxTime, TimeOut, 0) == -1) return(-1);  // Timeout on receiving the echoed byte
    if(RxByte != (itRec)) return(-1);   // illegal character received
  }
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut, 0) == -1) return(-1);  // Timeout on receiving <EOT>
  if( RxByte != EOT ) return(-1);  // received no <EOT>

  return 0;
}

/**
 * send a given string and append a CR character. Wait at most TimeOut seconds for the
 * response. If ResponseToWaitFor is empty, the function does not wait at all.
 *
 * NOTE: the receiver is assumed to ECHO each character!
 *
 *
 * Return value:  0 - succ. operation
 *               -1 - timeout occured
 */
int
SerialCmd::sendBufferAwaitCompletion(std::string const & StringToSend, std::string const & ResponseToWaitFor, int TimeOut)
{
	unsigned char RxByte;
	double RxTime;
	
  for( auto const& itSend: StringToSend ) {
    serial_.SendByte( itSend );
    if(serial_.WaitForSingleByte(RxByte, RxTime, TimeOut, 0) == -1) return(-1);  // Timeout on receiving the echoed byte
    if(RxByte != (itSend)) return(-1);   // illegal character received
  }

	if(blocking_ == false) {
    serial_.WaitForSingleByte(RxByte, RxTime, 1, 0);
    return(0);
  }
	
	// wait for proper CR/LF in the other cases
	if(serial_.WaitForSingleByte(RxByte, RxTime, TimeOut, 0) == -1) return(-1);  // Timeout on receiving the echoed byte
	if(serial_.WaitForSingleByte(RxByte, RxTime, TimeOut, 0) == -1) return(-1);  // Timeout on receiving the echoed byte

	//DEBUG_MESSAGE("waiting for response." << std::endl);

	// wait for the response
	for(std::string::const_iterator itRec = ResponseToWaitFor.begin();
			itRec !=ResponseToWaitFor.end(); itRec++)
		{
			//DEBUG_MESSAGE("Expecting: " << *itRec << std::endl);
			if(serial_.WaitForSingleByte(RxByte, RxTime, TimeOut, 0) == -1) return(-1);  // Timeout on receiving the echoed byte
			//DEBUG_MESSAGE("Received: [" << RxByte << "]" << std::endl);

			if(RxByte != (*itRec)) return(-1);   // illegal character received
		}

	//DEBUG_MESSAGE("Response received." << std::endl);
	return 0;
}


