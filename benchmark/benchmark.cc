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

using namespace metapod::benchmark;

int main()
{
  bench< simplehumanoid::Robot, JCALC >::run();
  bench< simplehumanoid::Robot, RNEA >::run();
  bench< simplehumanoid::Robot, RNEA_WITHOUT_JCALC >::run();
  bench< simplehumanoid::Robot, CRBA >::run();
  bench< simplehumanoid::Robot, CRBA_WITHOUT_JCALC >::run();
}
