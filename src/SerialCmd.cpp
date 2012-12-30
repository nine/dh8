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


#define DEBUG_ERROR(txt) std::cerr << txt

#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <unistd.h>
#include <fstream>     // file io
#include <iostream>    // file io
#include <clocale>     // time
#include <ctime>       // time
#include <cctype>      // isspace
#include <sstream>

#include "SerialCmd.h"
#include "dh8defs.h"


SerialCmd::SerialCmd() : 
  SerialPort_("/dev/ttyUSB0"), 
  Cmd_(""),
  TimeOut_(200),                 // default timeout
  BdRate_(115200),
  blocking_(false),              // don't wait for completion 
  desc_("Allowed options"),
  OutFile_("measure.m"),
  Lt_("15"),
  Lc_("0"),
  Device_("Diavite DH-8"), 
  Version_("-1"),
  Unit_("-1"),
  Norm_("-1"),
  Tolerance_("-1"),
  zReference_("-1"),
  calZ_("-1")
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

	//DEBUG_INFO("SerialCmd::SerialCmd() entered." << std::endl);
}

SerialCmd::~SerialCmd()
{
	//DEBUG_INFO("SerialCmd::~SerialCmd() entered." << std::endl);
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

}


void
SerialCmd::runApplication(void)
{

//  std::cout << "params" << std::endl;
//  std::cout << "SerialPort_ " << SerialPort_ << std::endl;
//  std::cout << "Cmd_" << Cmd_ << std::endl;
//  std::cout << "Reply_" << Reply_ << std::endl; 
//  std::cout << "TimeOut_ " << TimeOut_ << std::endl; 
//  std::cout << "BdRate_ " << BdRate_ << std::endl; 
//  std::cout << "blocking_ " << blocking_ << std::endl;


  if (args_.count("help")) {
    std::cout << "This is " << APPNAME << " version " << VERSION << std::endl;
    std::cout << "Usage: options_description [options]" << std::endl;
    std::cout << desc_ << std::endl;
    exit(0);
  }

  if( args_.count("lt") ) {
    if( !Lt_map_.count(Lt_) ) {
      std::cout << "Usage error: option lt out of range" << std::endl;
      exit(DH8_ERROR_PARAMNOTALLOWED);
    }
  }
  if( args_.count("lc") ) {
    if( !Lc_map_.count(Lc_) ) {
      std::cout << "Usage error: option lc out of range" << std::endl;
      exit(DH8_ERROR_PARAMNOTALLOWED);
    }
  }

	// and off we go ...
  // no parity, 1 stop-bit
  if(serial_.InitSerial(SerialPort_, BdRate_, 0, 1) < 1)
  {
    DEBUG_ERROR("Error opening the serial device." << std::endl);
    exit(DH8_ERROR_SERIALPORT);
  }

  if( sendCommandCheckReply(DH8_READ_DEVICE, Device_, TimeOut_) < 0 ) {
    std::cerr << "error wrong measurement device" << std::endl;
    exit(DH8_ERROR_DEVICE);
  }
  if( sendCommand( DH8_READ_VERSION, Version_, TimeOut_ ) < 0 ) {
    std::cerr << "error reading device version" << std::endl;
    exit(DH8_ERROR_DEVICEVERSION);
  }
  if( sendCommand( DH8_READ_UNIT, Unit_, TimeOut_ ) < 0 ) {
    std::cerr << "error reading unit" << std::endl;
    exit(DH8_ERROR_EXECCMD);
  }
  if( sendCommand( DH8_READ_NORM, Norm_, TimeOut_ ) < 0 ) {
    std::cerr << "error reading norm" << std::endl;
    exit(DH8_ERROR_EXECCMD);
  }
  if( sendCommand( DH8_READ_TOLERANCE, Tolerance_, TimeOut_ ) < 0 ) {
    std::cerr << "error reading tolerance" << std::endl;
    exit(DH8_ERROR_EXECCMD);
  }
  if( !args_.count("readout-only") ) {
    // set measure params
    std::string parm = DH8_SET_LT;
    parm.push_back(Lt_map_[Lt_]);
    if( sendCommandCheckReply(parm, "", TimeOut_) < 0 ) {
      std::cerr << "error setting Lt" << std::endl;
      exit(DH8_ERROR_EXECCMD);
    }

    parm = DH8_SET_LC;
    parm.push_back(Lc_map_[Lc_]);
    if( sendCommandCheckReply(parm, "", TimeOut_) < 0 ) {
      std::cerr << "error setting Lc" << std::endl;
      exit(DH8_ERROR_EXECCMD);
    }

    // perform measurement
    if( sendCommandCheckReply(DH8_START_MEASURE, "", TimeOut_) < 0 ) {
      std::cerr << "error starting measure" << std::endl;
      exit(DH8_ERROR_PERFORM_MEASUREMENT);
    }
  }
  std::string rep; 
  if( sendCommand( DH8_GET_R_VALUES, rep, TimeOut_ ) < 0 ) {
    std::cerr << "error reading r-values" << std::endl;
    exit(DH8_ERROR_READDATA);
  }

  // parsing csv
  std::stringstream tmp(rep);
  std::stringstream keys, vals;
  std::string key, val;
  std::getline(tmp, key);
  keys << key;
  std::getline(tmp, val);
  vals << val;
  while( std::getline(keys, key, ',') ) {
    if( !std::getline(vals, val, ',') ) break;
    if( key.empty() || val.empty() || key==" " || std::isspace(val.at(0)) ) continue;
    Dh8Values_[key] = val;
  }
  if( sendCommand(DH8_READ_Z_REFERENCE, zReference_, TimeOut_) < 0 ) {
    std::cerr << "error reading z reference" << std::endl;
    exit(DH8_ERROR_READDATA);
  }
  if( sendCommand(DH8_READ_CAL_Z, calZ_, TimeOut_) < 0 ) {
    std::cerr << "error reading cal_z" << std::endl;
    exit(DH8_ERROR_READDATA);
  }

  if( sendCommandCheckReply(DH8_SET_FMT_BIN, "", TimeOut_) < 0 ) {
    std::cerr << "error setting transmission format" << std::endl;
    exit(DH8_ERROR_READDATA);
  }
  if( sendCommandReadValues(DH8_READ_Z_VALUES, zValues_, TimeOut_) < 0) {
    std::cerr << "error reading Z values" << std::endl;
    exit(DH8_ERROR_READDATA);
  }
  if( sendCommandReadValues(DH8_READ_FILT_Z_VALUES, zValuesFilt_, TimeOut_) < 0) {
    std::cerr << "error reading filtered Z values" << std::endl;
    exit(DH8_ERROR_READDATA);
  }
//  if( sendCommandReadValues2(DH8_READ_PRO_VALUES, proValues_, proPositions_, TimeOut_) < 0) {
//    std::cerr << "error reading PRO values" << std::endl;
//    exit(-1);
//  }

  // write to file
  writeDataFile();

}


int 
SerialCmd::writeDataFile() {

  char hostname[255];
  std::string in("  ");   // line indent
  std::string ml(" ...");  // multi line
  
  try {
    // output to file
    std::ofstream outfile( OutFile_.c_str() );
    outfile.exceptions( std::ofstream::badbit | std::ofstream::failbit ); // throw exception on error
    
    // Use compiler's native locale.
    std::setlocale( LC_TIME, "" );
    std::time_t t = std::time( 0 );


    outfile << "% file generated: " << std::ctime( &t ); 
    if( gethostname( hostname, 254 )==0 ) {
      outfile << "% on: " << hostname << std::endl << std::endl;
    }

    outfile << "Surface = struct( " << ml << std::endl;

    // raw values
    outfile << in << "'Lt', " << Lt_ << "," << ml << std::endl;
    outfile << in << "'Lc', " << Lc_ << "," << ml << std::endl;
    outfile << in << "'dh8', struct( " << ml << std::endl;
    outfile << in<<in<< "'device', '" << Device_ << "', " << ml << std::endl;
    outfile << in<<in<< "'version', '" << Version_ << "', " << ml << std::endl;
    outfile << in<<in<< "'unit', '" << Unit_ << "', " << ml << std::endl;
    outfile << in<<in<< "'norm', '" << Norm_ << "', " << ml << std::endl;
    outfile << in<<in<< "'torlerance', '" << Tolerance_ << "' " << ml << std::endl;
    outfile << in << ")," << ml << std::endl;

    outfile << in << "'dh8_R_values', struct( " << ml << std::endl;
    outfile << in<<in<< "'"<<Dh8Values_.begin()->first<<"'," << Dh8Values_.begin()->second;
    Dh8Values_.erase( Dh8Values_.begin() );
    for( std::map<std::string,std::string>::const_iterator itVal = Dh8Values_.begin();
         itVal != Dh8Values_.end(); ++itVal ) {
      outfile << "," << ml << std::endl;
      outfile << in<<in<< "'"<<itVal->first<<"'," << itVal->second;
    }
    outfile << ml << std::endl << in << ")," << ml << std::endl;
    outfile << in << "'cal_z'," << calZ_ <<","<<ml << std::endl;
    outfile << in << "'z_reference'," << zReference_ <<","<<ml << std::endl;

    outfile << in << "'z_values', [ " << ml << std::endl;
    outfile << static_cast<float>(zValues_[0])/DH8_Z_FACTOR;
    for( std::vector<int>::iterator itOut = zValues_.begin()+1; itOut != zValues_.end(); itOut++ ) {
      outfile << ",";
      outfile << static_cast<float>(*itOut)/DH8_Z_FACTOR;
    }
    outfile << in << "]," << ml << std::endl,
    outfile << in << "'z_values_filt', [ " << ml << std::endl;
    outfile << static_cast<float>(zValuesFilt_[0])/DH8_Z_FACTOR;
    for( std::vector<int>::iterator itOut = zValuesFilt_.begin()+1; itOut != zValuesFilt_.end(); itOut++ ) {
      outfile << ",";
      outfile << static_cast<float>(*itOut)/DH8_Z_FACTOR;
    }
    outfile << in << "]" << ml << std::endl,
    
    outfile << ");" << std::endl;
    
    outfile.close();
  } 
  catch( std::ios_base::failure &e ) {
    std::cerr << "error writing data to file: " << e.what() << std::endl;
    exit(DH8_ERROR_WRITEFILE);
  }

  return 0;
}


int 
SerialCmd::sendCommandReadValues2(std::string const & stringToSend, std::vector<int> & valuesReceived, std::vector<int> & posReceived, int TimeOut) {
  unsigned char RxByte;
  unsigned int gain = 0;
  int valCount = 0;
  unsigned int valBuffer = 0;
  double RxTime;

  std::string txBuffer = appendDelimiters( stringToSend );

  // send bytes, no echo
  for( std::string::const_iterator itSend = txBuffer.begin();
      itSend != txBuffer.end(); itSend++ ) {
    serial_.SendByte(*itSend);
  }
  
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

  std::string txBuffer = appendDelimiters( stringToSend );

  // send bytes, no echo
  for( std::string::const_iterator itSend = txBuffer.begin();
      itSend != txBuffer.end(); itSend++ ) {
    serial_.SendByte(*itSend);
  }
  
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


std::string
SerialCmd::appendDelimiters(std::string const & str) const {
  std::string ret = ESC+str+ETX;
  return ret;
}

int 
SerialCmd::sendCommand(std::string const & stringToSend, std::string & stringReceived, int TimeOut)
{
  unsigned char RxByte;
  double RxTime;
  std::stringstream rxBuffer;
  const int count = 400;
 
  std::string txBuffer = appendDelimiters(stringToSend);
  // send bytes, no echo
  for( std::string::const_iterator itSend = txBuffer.begin();
       itSend != txBuffer.end(); itSend++ ) {
    serial_.SendByte(*itSend);
  }
  
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
  std::string txBuffer = appendDelimiters(stringToSend);
  
  // send bytes, no echo
  for( std::string::const_iterator itSend = txBuffer.begin();
       itSend != txBuffer.end(); itSend++ ) {
    serial_.SendByte(*itSend);
  }
  
  // response format: <ACK> answer <EOT>
  if( serial_.WaitForSingleByte(RxByte, RxTime, TimeOut, 0) == -1) return(-1);  // Timeout on receiving <ACK>
  if( RxByte != ACK ) return(-1);  // received no <ACK>

	// check the response
	for(std::string::const_iterator itRec = responseToCheck.begin();
			itRec !=responseToCheck.end(); itRec++) {
    if(serial_.WaitForSingleByte(RxByte, RxTime, TimeOut, 0) == -1) return(-1);  // Timeout on receiving the echoed byte
    if(RxByte != (*itRec)) return(-1);   // illegal character received
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
	//DEBUG_INFO("SerialCmd::sendBufferAwaitCompletion() entered." << std::endl);
	
	for(std::string::const_iterator itSend = StringToSend.begin();
			itSend !=StringToSend.end(); itSend++)
		{
			//DEBUG_MESSAGE("Sending: " << *itSend << std::endl);
			serial_.SendByte(*itSend);
			if(serial_.WaitForSingleByte(RxByte, RxTime, TimeOut, 0) == -1) return(-1);  // Timeout on receiving the echoed byte
			if(RxByte != (*itSend)) return(-1);   // illegal character received
		}

	if(blocking_ == false)
  {
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


