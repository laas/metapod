// Copyright 2011, 2012, 2013
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
// Sébastien Barthélémy (Aldebaran Robotics)
// Olivier Stasse (LAAS CNRS)
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

// This test solves the forward kinematics problem on a test model with a
// reference configuration, then compares the computed transforms with the
// reference ones


// Common test tools
# include "common.hh"
# include <metapod/algos/djac.hh>
# include <metapod/algos/rnea.hh>
# include <metapod/tools/jcalc.hh>

using namespace metapod;

typedef double LocalFloatType;
typedef CURRENT_MODEL_ROBOT<LocalFloatType> CURRENT_MODEL_ROBOT_LFT;

BOOST_AUTO_TEST_CASE (test_djac)
{
  // Set configuration vector to reference values.
  CURRENT_MODEL_ROBOT_LFT::confVector q,dq,ddq;
  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  std::ifstream dqconf(TEST_DIRECTORY "/dq.conf");
  std::ifstream ddqconf(TEST_DIRECTORY "/ddq.conf");

  initConf<CURRENT_MODEL_ROBOT_LFT>::run(qconf, q);
  initConf<CURRENT_MODEL_ROBOT_LFT>::run(dqconf, dq);
  initConf<CURRENT_MODEL_ROBOT_LFT>::run(ddqconf, ddq);

  qconf.close();
  dqconf.close();
  ddqconf.close();

  CURRENT_MODEL_ROBOT_LFT robot;
  jcalc< CURRENT_MODEL_ROBOT_LFT>::run(robot, q, dq);
  rnea< CURRENT_MODEL_ROBOT_LFT>::run(robot, q, dq, ddq);

  std::ofstream state_log("djac_jcalc_state.log", std::ofstream::out);
  printState<CURRENT_MODEL_ROBOT_LFT>(robot, state_log);
  state_log.close();

  typedef Eigen::Matrix<LocalFloatType,
                        6 * CURRENT_MODEL_ROBOT_LFT::NBBODIES,
                        CURRENT_MODEL_ROBOT_LFT::NBDOF> dJacobian;
  dJacobian dJ = dJacobian::Zero();
  djac< CURRENT_MODEL_ROBOT_LFT>::run(robot, dJ);

  const char result_file[] = "djac.log";
  std::ofstream log(result_file, std::ofstream::out);
  log << "derivative_of_the_kinematic_jacobian\n" << dJ << std::endl;
  log.close();

  state_log.open("djac_state.log", std::ofstream::out);
  printState<CURRENT_MODEL_ROBOT_LFT>(robot, state_log);
  state_log.close();

  // Compare results with reference file
  compareLogs(result_file, TEST_DIRECTORY "/djac.ref", 1e-3);
}
