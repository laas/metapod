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
 * This file run performance tests on metapods algorithms, on several sample
 * models.
 */

#include "models/sample_3_dof/sample_3_dof.hh"
#include "models/sample_7_dof/sample_7_dof.hh"
#include "models/sample_15_dof/sample_15_dof.hh"
#include "models/sample_31_dof/sample_31_dof.hh"
#include "models/sample_63_dof/sample_63_dof.hh"
#include "metapod/models/simple-humanoid/simple_humanoid.hh"

#include "benchmark.hh"
using namespace metapod::benchmark;

int main()
{
  BENCHMARK(metapod::simple_humanoid);
  BENCHMARK(metapod::sample_3_dof);
  BENCHMARK(metapod::sample_7_dof);
  BENCHMARK(metapod::sample_15_dof);
  BENCHMARK(metapod::sample_31_dof);
  BENCHMARK(metapod::sample_63_dof);
}
