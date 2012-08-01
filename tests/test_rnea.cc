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
 * This test applies the rnea on a test model with a reference configuration,
 * then compares the computed torques with the reference torques
 */

// Common test tools
# include "common.hh"

using namespace metapod;
using namespace metapod::simple_humanoid;

BOOST_AUTO_TEST_CASE (test_rnea)
{
  // Set configuration vectors (q, dq, ddq) to reference values.
  Robot::confVector q, dq, ddq;

  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  std::ifstream dqconf(TEST_DIRECTORY "/dq.conf");
  std::ifstream ddqconf(TEST_DIRECTORY "/ddq.conf");

  initConf< Robot >::run(qconf, q);
  initConf< Robot >::run(dqconf, dq);
  initConf< Robot >::run(ddqconf, ddq);

  qconf.close();
  dqconf.close();
  ddqconf.close();

  // Apply the RNEA to the metapod multibody and print the result in a log file.
  rnea< Robot, true >::run(q, dq, ddq);
  std::ofstream log("rnea.log", std::ofstream::out);
  printTorques<Robot::Tree>(log);
  log.close();

  // Compare results with reference file
  double epsilon = 1e-3;
  FloatType x,y;
  std::string jointname;
  std::string str1, str2;
  std::ifstream result_log("rnea.log");
  std::ifstream ref_log(TEST_DIRECTORY "/rnea.ref");
  while(result_log >> str1)
  {
    if(stringToDouble(str1, x))
    {
      bool get_y_ok = getNextDouble(ref_log, y);
      BOOST_CHECK(get_y_ok && "could not read reference value in rnea.ref");
      BOOST_CHECK(compareDouble(x, y, epsilon)
        && "Difference in log and reference files (rnea.log and rnea.ref).");
      if(!compareDouble(x, y, epsilon))
        std::cerr << jointname << "\n\t" << x << "\n\t" << y << std::endl;
    }
    else
    {
      ref_log.clear(); ref_log.seekg(0);
      jointname = str1;
      do
      {
        ref_log >> str2;
      } while(str2.compare(jointname) && !ref_log.eof());
    }
  }
  result_log.close();
  ref_log.close();
}
