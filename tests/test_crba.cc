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
 * This test applies the crba on a test model with a reference configuration,
 * then compares the computed inertia matrix with the reference inertia matrix.
 */

// Common test tools
# include "common.hh"

using namespace metapod;
using namespace metapod::simple_humanoid;

BOOST_AUTO_TEST_CASE (test_crba)
{
  // set configuration vector q to reference value.
  Robot::confVector q;
  std::ifstream qconf(TEST_DIRECTORY "q.conf");
  initConf< Robot >::run(qconf, q);
  qconf.close();

  // Initialize the Joint-Space Inertia Matrix to Zero.
  Robot::H = matrixN::Zero(Robot::NBDOF, Robot::NBDOF);

  // Apply the CRBA to the metapod multibody and print the result in a log file
  crba< Robot, true >::run(q); // Update geometry and run the CRBA
  std::ofstream log("crba.log", std::ofstream::out);
  log << Robot::H << std::endl;
  log.close();

  // Compare results with reference file
  double epsilon = 1e-3;
  FloatType x,y;
  std::string str1, str2;
  std::ifstream result_log("crba.log");
  std::ifstream ref_log(TEST_DIRECTORY "/crba.ref");
  int i = 0, row, col;
  while(result_log >> str1)
  {
    col = i%Robot::NBDOF;
    row = i/Robot::NBDOF;
    ref_log >> str2;
    bool str1_ok = stringToDouble(str1, x);
    bool str2_ok = stringToDouble(str2, y);
    BOOST_CHECK(str1_ok && "Error reading value in crba.log");
    BOOST_CHECK(str2_ok && "Error reading value in crba.ref");
    BOOST_CHECK(compareDouble(x, y, epsilon)
      && "Difference in log and reference files (crba.log and crba.ref).");
    if(!compareDouble(x, y, epsilon))
    {
      std::cerr << "H(" << row << "," << col << ")\n\t" << x << "\n\t" << y
                << std::endl;
    }
    i++;
  }
  result_log.close();
  ref_log.close();
}
