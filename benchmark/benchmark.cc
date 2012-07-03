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
 * This file run performance tests on metapods algorithms.
 */

# include "benchmark.hh"

# include "metapod/models/simple-humanoid/robot.hh"
# include "models/model_3_dof_robot.hh"
# include "models/model_7_dof_robot.hh"
# include "models/model_15_dof_robot.hh"
# include "models/model_31_dof_robot.hh"
# include "models/model_63_dof_robot.hh"

using namespace metapod::benchmark;


int main()
{
  // Choose what to bench
  Setup::JCALC = true;
  Setup::RNEA = true;
  Setup::CRBA = true;
  Setup::RNEA_WITHOUT_JCALC = true;
  Setup::CRBA_WITHOUT_JCALC = true;


  bench< model_3_dof::Robot, Setup >::run();
  bench< model_7_dof::Robot, Setup >::run();
  bench< model_15_dof::Robot, Setup >::run();
  bench< model_31_dof::Robot, Setup >::run();
  bench< model_63_dof::Robot, Setup >::run();

  bench< simplehumanoid::Robot, Setup >::run();
}
