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

#ifndef SERIALCMD_H
#define SERIALCMD_H


#include <boost/program_options.hpp>
namespace po = boost::program_options;

#include <stdint.h>
#include <string>
#include "Serial.h"
#include "dh8Data.h"

class SerialCmd
{
  public:
    SerialCmd();
    virtual ~SerialCmd();

    // application interface
    virtual void configure(int argc, char **argv);
    virtual void initDevice(void);
    virtual void runApplication(void);

  private:
    std::string SerialPort_;            // Serial port used by the DH8
    std::string Cmd_;                   // command sent to the DH8
    std::string Reply_;                 // reply to wait for (blocking commands)
    int TimeOut_;                       // max. time to wait for a response
    int BdRate_;                        // Baud-Rate of the transmission
    bool blocking_;                     // allow for blocking commands
    Serial serial_;                     // interface object
    po::variables_map args_;            // commandline arguments
    po::options_description desc_;      // commandline argument descriptions

    std::string OutFile_;               // .m matlab output file
    std::string Lt_;                    // traverse length
    std::string Lc_;                    // filter cutoff length
    std::map<std::string, unsigned char> Lt_map_;  // map length to parameter
    std::map<std::string, unsigned char> Lc_map_;  // map length to parameter

    // DH8 data
    Dh8Data Data_;

    // methods
    void sendStringWithDelimiters(std::string const & tx_buffer);
    int sendCommandReadValues(std::string const & stringToSend, std::vector<int> & valuesReceived, int TimeOut);
    int sendCommandReadValues2(std::string const & stringToSend, std::vector<int> & valuesReceived, std::vector<int> & posReceived, int TimeOut);
    int sendCommand(std::string const & stringToSend, std::string & stringReceived, int TimeOut);
    int sendCommandCheckReply(std::string const & stringToSend, std::string const & responseToCheck, int TimeOut);
    int sendBufferAwaitCompletion(std::string const & StringToSend, std::string const & ResponseToWaitFor, int TimeOut);

};

#endif // guards
