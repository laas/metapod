// Copyright 2014
//
// Nuno Guedelha (JRL/LAAS, CNRS/AIST)
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
#include "common.hh"

using namespace metapod;

typedef double LocalFloatType;

BOOST_AUTO_TEST_CASE (test_initnufwddyn)
{
  typedef CURRENT_MODEL_ROBOT<LocalFloatType> CURRENT_MODEL_ROBOT_LFT;
  // nu(fd) computation is done at compile-time when compiling @ROBOT_CLASS_NAME@.cc
  // Write it to a log file. "true" bool value is written as "1", "false" is written as "0".
  // re-compute at runtime the FD and nu(FD) parameters, while running a depth first traversal,
  // and print result to a reference file as per same format.
  std::ofstream initnufwddyn_log("initnufwddyn.log", std::ofstream::out);
  std::ofstream initnufwddyn_ref("initnufwddyn.ref", std::ofstream::out);
  printNuFwdDyn<CURRENT_MODEL_ROBOT_LFT>(initnufwddyn_log, initnufwddyn_ref);
  initnufwddyn_log.close();
  initnufwddyn_ref.close();

  // Compare resulting file with reference file. 
  compareLogs("initnufwddyn.log", "initnufwddyn.ref", 1e-6);
}
