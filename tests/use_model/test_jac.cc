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

// This test solves the forward kinematics problem on a test model with a
// reference configuration, then compares the computed transforms with the
// reference ones


// Common test tools
# include "common.hh"
# include <metapod/algos/jac.hh>
# include <metapod/tools/jcalc.hh>

using namespace metapod;

BOOST_AUTO_TEST_CASE (test_jac)
{
  // Set configuration vector to reference values.
  CURRENT_MODEL_ROBOT::confVector q;
  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  initConf<CURRENT_MODEL_ROBOT>::run(qconf, q);
  qconf.close();

  CURRENT_MODEL_ROBOT robot;
  jcalc< CURRENT_MODEL_ROBOT>::run(
      robot, q, CURRENT_MODEL_ROBOT::confVector::Zero());
  typedef Eigen::Matrix<FloatType,
                        6 * CURRENT_MODEL_ROBOT::NBBODIES,
                        CURRENT_MODEL_ROBOT::NBDOF> Jacobian;
  Jacobian J = Jacobian::Zero();
  jac< CURRENT_MODEL_ROBOT>::run(robot, J);

  const char result_file[] = "jac.log";
  std::ofstream log(result_file, std::ofstream::out);
  log << "kinematic_jacobian\n" << J << std::endl;
  log.close();

  // Compare results with reference file
  compareLogs(result_file, TEST_DIRECTORY "/jac.ref", 1e-3);
}
