// Copyright 2011, 2012,
//
// Maxime Reis
//
// JRL/LAAS, CNRS/AIST
//
// This file is part of metapod.
// metapod is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// metapod is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod.  If not, see <http://www.gnu.org/licenses/>.

/*
 * This file contains common tools and includes shared by the test suite
 */

#ifndef METAPOD_TEST_COMMON_TOOLS
# define METAPOD_TEST_COMMON_TOOLS

// Common includes
# include <string>
# include <iostream>
# include <fstream>

// metapod includes
# include "metapod/models/simple-humanoid/simple_humanoid.hh"
# include "metapod/models/simple-arm/simple_arm.hh"
# include "metapod/tools/print.hh"
# include "metapod/tools/initconf.hh"

// Boost test suite includes
# define BOOST_TEST_MODULE METAPOD_RNEA
# include <boost/test/unit_test.hpp>

// Performance test : some tests also output an average execution time of the
// tested component if METAPOD_PERF_TEST is defined
# ifdef METAPOD_PERF_TEST
#  include <sys/time.h>
# endif

// Converts a string to a double if possible, returns 0 otherwise
bool stringToDouble( const std::string& s, double& x )
 {
   std::istringstream i(s);
   i >> x;
   return !(i.fail() || i.bad());
 }

// Returns true if the normalized difference between two doubles is strictly
// inferior to the threshold parameter, false otherwise
bool compareDouble(double x, double y, double epsilon)
{
  if(fabs(y) > epsilon)
  {
    double normalized_diff = fabs((x-y)/y);
    return normalized_diff<epsilon?1:0;
  }
  else return fabs(x)>epsilon?0:1;
}

// Extracts the next double encountered in a stream file and returns it.
// Returns 0 if none is found.
bool getNextDouble( std::ifstream & is, double& x)
{
  std::string s;
  while(!is.eof())
  {
    is >> s;
    if(stringToDouble(s, x))
      return true;
  }
  return 0;
}

// Compare two log files
// This function returns nothing, but uses boost::test assertions
// to report discrepancies
// The log files should contain succession of labels and
// real values. The labels will matched from one file to the other and
// the real values will be compared. The labels cannot contain spaces
//
// Here is an example of a valid log file:
//
// spaceless_label
// 1   .1  12.2
// 1 23.    0
// mass
// 2.5
void compareLogs(
    const std::string& result_file,
    const std::string& reference_file,
    double epsilon)
{
  double result_value, reference_value;
  std::string name;
  std::string result_string, reference_string;
  std::ifstream result_stream(result_file.c_str());
  std::ifstream reference_stream(reference_file.c_str());
  while(result_stream >> result_string)
  {
    unsigned int i = 0;
    if(stringToDouble(result_string, result_value))
    {
      bool get_y_ok = getNextDouble(reference_stream, reference_value);
      BOOST_CHECK(get_y_ok);
      if(!get_y_ok)
      {
        std::cerr << "Could not read reference value "
                  << name << "(" << i << ") in " << reference_file
                  << std::endl;
      }
      else
      {
        bool compare_ok = compareDouble(result_value, reference_value, epsilon);
        BOOST_CHECK(compare_ok);
        if(!compare_ok)
          std::cerr << "Difference in result and reference files ("
                    << result_file << " " << reference_file << ")\n"
                    << name << "(" << i << ")\n\t"
                    << result_value << "\n\t"
                    << reference_value << std::endl;
      }
      ++i;
    }
    else
    {
      i = 0;
      reference_stream.clear(); reference_stream.seekg(0);
      name = result_string;
      do
      {
        reference_stream >> reference_string;
      } while(reference_string.compare(name) && !reference_stream.eof());
    }
  }

}
#endif
