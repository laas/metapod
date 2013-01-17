// Copyright 2012, 2013
//
// Olivier STASSE (LAAS/CNRS)
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
// GNU General Lesser Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod. If not, see <http://www.gnu.org/licenses/>.

// This test checks the spatial headers can be included in several compilation
// units without triggering "double definition" errors at link-time.
// Running the test is not relevant (but compiling it is).

#include "../common.hh"
#include <metapod/tools/spatial.hh>
#include "doubledef.hh"

BOOST_AUTO_TEST_CASE(test_doubledef)
{
  BOOST_CHECK(dummy());
}
