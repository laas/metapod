// Copyright 2012,
//
// Sébastien Barthélémy
//
// Aldebaran Robotics
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
 * This file is part of a simple arm robot model, used for test purpose.
 * It contains the definition of all the robot joints.
 */

#ifndef METAPOD_SIMPLE_ARM_JOINT_HH
# define METAPOD_SIMPLE_ARM_JOINT_HH

# include "metapod/tools/jointmacros.hh"

namespace metapod
{
  namespace simple_arm
  {
    JOINT_REVOLUTE_AXIS_ANY(SHOULDER, 1, 0, 0);
    JOINT_REVOLUTE_AXIS_ANY(ELBOW, 1, 0, 0);
    JOINT_REVOLUTE_AXIS_ANY(WRIST, 1, 0, 0);
  } // end of namespace simple_arm
} // end of namespace metapod

#endif
