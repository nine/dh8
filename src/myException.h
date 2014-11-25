#ifndef myException_h_INCLUDED
#define myException_h_INCLUDED

#include <string>
#include <stdexcept>


// error codes
namespace
{
  const int16_t E_SUCCESS = 0;
  const int16_t E_OPTIONS = 1;
  const int16_t E_FILE = 2;
  const int16_t E_PARAMS = 3;
  const int16_t E_UNHANDLED_EXCEPTION = 9;
} // namespace 

// dh8 specific
enum DH8_ERRORNUMBERS {
  DH8_ERROR_NOERROR = 0,

  // serial port
  DH8_ERROR_SERIALPORT = 10,
  // wrong device attached
  DH8_ERROR_DEVICE = 20,
  // wrong device version
  DH8_ERROR_DEVICEVERSION = 30,
  // execution of command
  DH8_ERROR_EXECCMD = 40,
  // perform measurement
  DH8_ERROR_PERFORM_MEASUREMENT = 50,
  // read surface profile data
  DH8_ERROR_READDATA = 60,
  // write measurement file 
  DH8_ERROR_WRITEFILE = 70,

  // parameter out of range
  DH8_ERROR_PARAMNOTALLOWED = 90
};



class MyException : public std::exception {
public:

  MyException(std::string m = "Exception! ", int16_t err_num = 0) : msg_(m), error_number_(err_num) {};
  ~MyException() throw () {};

  const char* what() const throw () {
    return msg_.c_str();
  };

  int16_t getErrorNumber() {
    return error_number_;
  };
private:
  std::string msg_;
  int16_t error_number_;
};

#endif
