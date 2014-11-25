#ifndef dh8Data_h_INCLUDED
#define dh8Data_h_INCLUDED


#include <string>

class Dh8Data 
{
friend class SerialCmd;

public:
  Dh8Data();
  ~Dh8Data() {};

  void parseValues(const std::string &buffer);
  void writeMFile(const std::string &file) const;

private:
   // DH8 data
  std::string Device_;
  std::string Version_;
  std::string Lt_;                    // traverse length
  std::string Lc_;                    // filter cutoff length
  std::string Unit_;
  std::string Norm_;
  std::string Tolerance_;
  std::string zReference_;
  std::string calZ_;
  std::vector<int> zValues_;
  std::vector<int> zValuesFilt_;
  // unused, just for profile tracer!
  //std::vector<int> proValues_, proPositions_;
  std::map<std::string, std::string> Values_;

};

#endif
