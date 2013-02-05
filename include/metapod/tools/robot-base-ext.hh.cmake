// Copyright 2013
//
// Olivier STASSE, (LAAS, CNRS)
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

#ifndef METAPOD_ROBOT_BASE_EXT_HH
# define METAPOD_ROBOT_BASE_EXT_HH

#define ABSTRACT_ROBOT_DYNAMICS_LAYER @ABSTRACT_ROBOT_DYNAMICS_FOUND@
#if ABSTRACT_ROBOT_DYNAMICS_LAYER == 1
#include <metapod/adaptors/abstract-robot-layer-robot.hh>
#endif

#endif
