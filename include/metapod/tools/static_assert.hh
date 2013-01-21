// Copyright 2013,
//
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
 * This file contains the includes and class definitions necessary for the
 * whole project.
 */

#ifndef METAPOD_STATIC_ASSERT_HH
# define METAPOD_STATIC_ASSERT_HH

# include <boost/config.hpp>
# include <boost/version.hpp>

# if (__cplusplus > 199711L) || !defined(BOOST_NO_STATIC_ASSERT)
// we use either a fully C++11 compliant compiler or one which supports the
// static_assert feature
#  define METAPOD_STATIC_ASSERT(x, msg) static_assert(x, msg)
# elif BOOST_VERSION > 104000
// use the one from boost. We got reports of problems with version 1.40,
// which was used in ubuntu 10.04, we thus avoid it.
#  define METAPOD_STATIC_ASSERT(x, msg) BOOST_STATIC_ASSERT_MSG(x, msg)
# else
// no-op implementation, which can be used at class scope
#  define METAPOD_STATIC_ASSERT(x, msg) typedef int metapod_static_assert_no_op_
# endif

#endif
