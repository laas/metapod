// Copyright 2011, 2012, 2013
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
// Olivier Stasse (JRL/LAAS, CNRS/AIST)
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
 * This file run performance tests on metapods algorithms, on several sample
 * models.
 */

#define FUSION_MAX_VECTOR_SIZE 50
#ifdef WITH_SIMPLE_HUMANOID
# include "metapod/models/simple_humanoid/simple_humanoid.hh"
#endif
#ifdef WITH_METAPOD_BINARYTREEMODEL
# include "models/sample_1/sample_1.hh"
# include "models/sample_2/sample_2.hh"
# include "models/sample_3/sample_3.hh"
# include "models/sample_4/sample_4.hh"
//# include "models/sample_5/sample_5.hh"
#endif

#include "benchmark.hh"
using namespace metapod::benchmark;

int main()
{
  typedef @METAPOD_DEFAULT_FLOAT_TYPE@ FloatType;

#ifdef WITH_SIMPLE_HUMANOID
  benchmark<metapod::simple_humanoid<FloatType> >::run();
#endif
#ifdef WITH_METAPOD_BINARYTREEMODEL
  benchmark<metapod::sample_1<FloatType> >::run();
  benchmark<metapod::sample_2<FloatType> >::run();
  benchmark<metapod::sample_3<FloatType> >::run();
  benchmark<metapod::sample_4<FloatType> >::run();
  //benchmark<metapod::sample_5>::run();
#endif
}
