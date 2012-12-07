// Copyright 2012,
//
// Antonio El Khoury (JRL/LAAS, CNRS/AIST)
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
 * This test computes the jacobian on a test model with a reference
 * configuration, then compares the computed jacobian with the
 * reference jacobian
 */

// Common test tools
# include "../common.hh"

using namespace metapod;
using namespace CURRENT_MODEL_NAMESPACE;

BOOST_AUTO_TEST_CASE (test_jac_point_chain_robot)
{
  // Set configuration vectors (q) to reference values.
  Robot::confVector q;
  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  initConf< Robot >::run(qconf, q);
  qconf.close();

  // Compute the jacobian and print the result in a log file.
  jac_point_chain_robot< Robot >::jacobian_t J =
      jac_point_chain_robot< Robot >::jacobian_t::Zero();
  jac_point_chain_robot< Robot >::run(q, J);
  const char result_file[] = "jac_point_chain_robot.log";
  std::ofstream log(result_file, std::ofstream::out);
  log << J << std::endl;;
  log.close();

  // Compare results with reference file
  compareLogs(result_file, TEST_DIRECTORY "/jac_point_chain_robot.ref", 1e-3);
}
