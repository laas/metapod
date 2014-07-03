// Copyright 2011, 2012, 2013
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

// This test applies the rnea on a test model with a reference configuration,
// then compares the computed torques with the reference torques

// Common test tools
#include "common.hh"
#include <metapod/algos/rnea.hh>

using namespace metapod;

DEFAULT_FLOAT_TYPE;

BOOST_AUTO_TEST_CASE (test_rnea)
{
  typedef CURRENT_MODEL_ROBOT<LocalFloatType> Robot;
  // Set configuration vectors (q, dq, ddq) to reference values.
  Robot::confVector q, dq, ddq, torques, ref_torques;

  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  std::ifstream dqconf(TEST_DIRECTORY "/dq.conf");
  std::ifstream ddqconf(TEST_DIRECTORY "/ddq.conf");

  initConf< Robot >::run(qconf, q);
  initConf< Robot >::run(dqconf, dq);
  initConf< Robot >::run(ddqconf, ddq);

  qconf.close();
  dqconf.close();
  ddqconf.close();

  Robot robot;
  // Apply the RNEA to the metapod multibody and print the result in a log file.
  rnea< Robot, true >::run(robot, q, dq, ddq);
  const char result_file[] = "rnea.log";
  std::ofstream log(result_file, std::ofstream::out);
  printTorques<Robot>(robot, log);
  log.close();

  // Compare results with reference file
  compareLogs(result_file, TEST_DIRECTORY "/rnea.ref", 1e-3);

  // smoke test: torques variable value is not checked
  getTorques(robot, torques);
  std::ifstream torquesconf(TEST_DIRECTORY "/rnea.ref");
  initConf< Robot >::run(torquesconf, ref_torques);
  BOOST_CHECK(ref_torques.isApprox(torques, 1e-3));
}
