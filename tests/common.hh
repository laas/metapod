// Copyright 2011, 2012,
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
// Sébastien Barthélémy (Aldebaran Robotics)
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

#ifndef METAPOD_TESTS_COMMON_HH
# define METAPOD_TESTS_COMMON_HH

// Common includes
# include <string>
# include <iostream>
# include <fstream>
# include <cmath>

// Boost test suite includes
# define BOOST_TEST_MODULE METAPOD
# include <boost/test/unit_test.hpp>

// Converts a string to a double if possible, returns false otherwise
bool stringToDouble( const std::string& s, double& x )
{
  std::istringstream i(s);
  return (i >> x) != 0;
}

// Returns true if the normalized difference between two doubles is strictly
// inferior to the threshold parameter, false otherwise
bool compareDouble(double x, double y, double epsilon)
{
  if(std::abs(y) > epsilon)
  {
    double normalized_diff = std::abs((x-y)/y);
    return normalized_diff<epsilon?1:0;
  }
  else return std::abs(x)>epsilon?0:1;
}

// Extracts the next double encountered in a stream file and returns it.
// Returns 0 if none is found.
bool getNextDouble( std::ifstream & is, double& x)
{
  std::string s;
  while(!is.eof())
  {
    is >> s;
    return stringToDouble(s, x);
  }
  return false;
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
  // ensure the reference file is really there
  BOOST_CHECK(reference_stream);
  unsigned int i = 0;
  while(reference_stream >> reference_string)
  {
    if(stringToDouble(reference_string, reference_value))
    {
      bool get_y_ok = getNextDouble(result_stream, result_value);
      BOOST_CHECK(get_y_ok);
      if(!get_y_ok)
      {
        std::cerr << "Could not read result value "
                  << name << "(" << i << ") in " << result_file
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
      result_stream.clear();
      result_stream.seekg(0);
      name = reference_string;
      do
      {
        result_stream >> result_string;
      } while(result_string.compare(name) && !result_stream.eof());
    }
  }
}

// Compare two text files
// This function returns nothing, but uses boost::test assertions
// to report discrepancies
// The text files are compared, all non-blank characters should be identical.
void compareTexts(
    const std::string& result_file,
    const std::string& reference_file)
{
  std::string result_string, reference_string;
  std::ifstream result_stream(result_file.c_str());
  std::ifstream reference_stream(reference_file.c_str());
  // ensure the reference file is really there
  BOOST_CHECK(reference_stream);
  while(reference_stream >> reference_string)
  {
    result_stream >> result_string;
    bool tokens_are_identical = result_string == reference_string;
    BOOST_CHECK(tokens_are_identical);
    if(!tokens_are_identical)
    {
      std::cerr << "Difference in result and reference files ("
                << result_file << " " << reference_file << ")" << std::endl;
      return; // No point in continuing
    }
  }
  bool end_of_result_is_reached = !(result_stream >> result_string);
  BOOST_CHECK(end_of_result_is_reached);
  if(!end_of_result_is_reached)
  {
    std::cerr << "Result file is longer than reference file:" << result_string << " ("
              << result_file << " " << reference_file << ")" << std::endl;
  }
}
#endif
