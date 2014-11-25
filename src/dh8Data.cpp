

#include <unistd.h>
#include <fstream>     // file io
#include <iostream>    // file io
#include <clocale>     // time
#include <ctime>       // time
#include <cctype>      // isspace
#include <sstream>
#include <iterator> 

#include "SerialCmd.h"
#include "dh8defs.h"
#include "myException.h"


Dh8Data::Dh8Data() :
  Device_("Diavite DH-8"),
  Version_("-1"),
  Lt_(""),
  Lc_(""),
  Unit_("-1"),
  Norm_("-1"),
  Tolerance_("-1"),
  zReference_("-1"),
  calZ_("-1")
{

}

void Dh8Data::parseValues(const std::string &buffer) 
{
  // comma-separated string
  std::stringstream tmp(buffer);
  std::stringstream keys, vals;
  std::string key, val;
  std::getline(tmp, key);
  keys << key;
  std::getline(tmp, val);
  vals << val;
  while( std::getline(keys, key, ',') ) {
    if( !std::getline(vals, val, ',') ) break;
    if( key.empty() || val.empty() || key==" " || std::isspace(val.at(0)) ) continue;
    Values_[key] = val;
  }
}

void Dh8Data::writeMFile(const std::string &file) const 
{
  char hostname[255];
  const std::string in("  ");   // line indent
  const std::string ml(" ...");  // multi lin
  
  try {
    // output to file
    std::ofstream outfile( file.c_str() );
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
    outfile << in<<in<< "'"<<Values_.begin()->first<<"'," << Values_.begin()->second;
    for( auto itVal = std::next(Values_.begin()); itVal != Values_.end(); ++itVal ) {
      outfile << "," << ml << std::endl;
      outfile << in<<in<< "'"<<itVal->first<<"'," << itVal->second;
    }
    outfile << ml << std::endl << in << ")," << ml << std::endl;
    outfile << in << "'cal_z'," << calZ_ <<","<<ml << std::endl;
    outfile << in << "'z_reference'," << zReference_ <<","<<ml << std::endl;

    outfile << in << "'z_values', [ " << ml << std::endl;
    outfile << static_cast<float>(zValues_[0])/DH8_Z_FACTOR;
    for( auto itOut = std::next(zValues_.begin()); itOut != zValues_.end(); itOut++ ) {
      outfile << ",";
      outfile << static_cast<float>(*itOut)/DH8_Z_FACTOR;
    }
    outfile << in << "]," << ml << std::endl,
    outfile << in << "'z_values_filt', [ " << ml << std::endl;
    outfile << static_cast<float>(zValuesFilt_[0])/DH8_Z_FACTOR;
    for( auto itOut = std::next(zValuesFilt_.begin()); itOut != zValuesFilt_.end(); itOut++ ) {
      outfile << ",";
      outfile << static_cast<float>(*itOut)/DH8_Z_FACTOR;
    }
    outfile << in << "]" << ml << std::endl,
    
    outfile << ");" << std::endl;
    
    outfile.close();
  } 
  catch( std::ios_base::failure &e ) {
    std::ostringstream descr;
    descr << "Writing data to file: ";
    descr << e.what();
    throw(MyException(descr.str(), DH8_ERROR_WRITEFILE));
  }
}


//eof
