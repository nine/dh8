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

#include "SerialCmd.h"
#include "myException.h"

#include <iostream>
#include <iostream>
#include <iterator>
#include <string>
#include <exception>



int main(int ac, char* av[])
{
  SerialCmd myApp;
  try {

    // parse the command line
    myApp.configure(ac,av);
    // initialize the devicie
    myApp.initDevice();
    // perform the measurement 
    myApp.runApplication();

  }
  catch(MyException& e) {
    if( e.getErrorNumber() != E_SUCCESS ) {
      std::cerr << "Error: " << e.what() << " (Exit value: " << e.getErrorNumber() << ")" << std::endl;
      return e.getErrorNumber();
    }
  }
  catch(std::exception& e) {
    std::cerr << "Error: " << e.what() << std::endl;
    return 1;
  }
  catch(...) {
    // any exception should end up here
    std::cerr << "Exception of unknown type!" << std::endl;
  }

  return 0;
}


//EOF
