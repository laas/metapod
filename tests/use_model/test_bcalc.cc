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
 * This test solves the forward kinematics problem on a test model with a
 * reference configuration, then compares the computed transforms with the
 * reference ones
 */

// Common test tools
# include "../common.hh"
# include "metapod/tools/jcalc.hh"
# include "metapod/tools/bcalc.hh"

using namespace metapod;
using namespace CURRENT_MODEL_NAMESPACE;

BOOST_AUTO_TEST_CASE (test_bcalc)
{
  // Set configuration vectors (q, dq, ddq) to reference values.
  Robot::confVector q, dq;

  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  std::ifstream dqconf(TEST_DIRECTORY "/dq.conf");

  initConf< Robot >::run(qconf, q);
  initConf< Robot >::run(dqconf, dq);

  qconf.close();
  dqconf.close();

  // Apply the body calculations to the metapod multibody and print the result in a log file.
  jcalc< Robot >::run(q, dq);
  bcalc< Robot >::run(q);
  const char result_file[] = "bcalc.log";
  std::ofstream log(result_file, std::ofstream::out);
  printTransforms<Robot::Tree>(log);
  log.close();

  // Compare results with reference file
  compareLogs(result_file, TEST_DIRECTORY "/bcalc.ref", 1e-3);
}
