// Copyright 2011, 2012, 2013
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
 * This file contains common tools and includes shared by the test suite
 */

#ifndef METAPOD_TESTS_USE_MODEL_COMMON_HH
# define  METAPOD_TESTS_USE_MODEL_COMMON_HH

# include <tests/common.hh>

// metapod includes
# ifdef CURRENT_MODEL_IS_SIMPLE_HUMANOID
#  include <metapod/models/simple_humanoid/simple_humanoid.hh>
# endif
# ifdef CURRENT_MODEL_IS_SIMPLE_ARM
#  include <metapod/models/simple_arm/simple_arm.hh>
# endif
# include <metapod/tools/print.hh>
# include <metapod/tools/initconf.hh>

#endif
