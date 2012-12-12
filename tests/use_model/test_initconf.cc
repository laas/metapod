// Copyright 2012,
//
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

// This test loads configuration vectors from disk then write them back and
// check their content is identical. It tests both initConf and printConf.

// Common test tools
#include "../common.hh"
#include <metapod/tools/initconf.hh>

using namespace metapod;
using namespace CURRENT_MODEL_NAMESPACE;

BOOST_AUTO_TEST_CASE (test_initconf)
{
  // Set configuration vector to reference values.
  Robot::confVector q;
  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  initConf< Robot >::run(qconf, q);
  qconf.close();

  // Write it back to a file
  std::ofstream q_log("q.log", std::ofstream::out);
  printConf<Robot::Tree>(q, q_log);
  q_log.close();

  // Compare resulting file with reference file
  compareLogs("q.log", TEST_DIRECTORY "/q.conf", 1e-5);
}
