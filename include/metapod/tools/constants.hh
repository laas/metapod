// Copyright 2013
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

// A few constants used by metapod and metapod_robotbuilder

#ifndef METAPOD_CONSTANTS_HH
# define METAPOD_CONSTANTS_HH

namespace metapod {

// Boost::fusion vector are apparently limited to 50 elements.
// Yet, that might be compiler-dependant.
// see the following link for a possible workaround
// http://lists.boost.org/Archives/boost/2012/03/190981.php
static const int MAX_NB_JOINTS = 50;

static const int MAX_NB_CHILDREN_PER_NODE = 5;

static const int NO_PARENT = -1;
static const int NO_CHILD = -2;
static const int NO_NODE = -3;

}
#endif
