// Copyright 2012, 2013
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

// This (smoke) test runs printStat on the model. Since the model is not
// initialized, the output is discarded. So we only check that printState
// builds and runs smoothly.

// Common test tools
#include "common.hh"
#include <metapod/tools/print.hh>

using namespace metapod;
DEFAULT_FLOAT_TYPE;
typedef CURRENT_MODEL_ROBOT<LocalFloatType> CURRENT_MODEL_ROBOT_LFT;

BOOST_AUTO_TEST_CASE (test_printstate)
{
  CURRENT_MODEL_ROBOT_LFT robot;
  std::ofstream state_log("state.log", std::ofstream::out);
  printState<CURRENT_MODEL_ROBOT_LFT>(robot, state_log);
  state_log.close();
}
