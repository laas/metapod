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
 * This file is part of a simple humanoid robot model, used for test purpose.
 * It contains the definition of all the robot joints.
 */

#ifndef METAPOD_SIMPLE_HUMANOID_JOINT_HH
# define METAPOD_SIMPLE_HUMANOID_JOINT_HH

# include "metapod/tools/jointmacros.hh"

namespace metapod
{
  namespace simple_humanoid
  {
    JOINT_FREE_FLYER(WAIST);
    JOINT_REVOLUTE_AXIS_X(WAIST_P);
    JOINT_REVOLUTE_AXIS_X(WAIST_R);
    JOINT_REVOLUTE_AXIS_X(CHEST);
    JOINT_REVOLUTE_AXIS_X(LARM_SHOULDER_P);
    JOINT_REVOLUTE_AXIS_X(LARM_SHOULDER_R);
    JOINT_REVOLUTE_AXIS_X(LARM_SHOULDER_Y);
    JOINT_REVOLUTE_AXIS_X(LARM_ELBOW);
    JOINT_REVOLUTE_AXIS_X(LARM_WRIST_Y);
    JOINT_REVOLUTE_AXIS_X(LARM_WRIST_P);
    JOINT_REVOLUTE_AXIS_X(LARM_WRIST_R);
    JOINT_REVOLUTE_AXIS_X(RARM_SHOULDER_P);
    JOINT_REVOLUTE_AXIS_X(RARM_SHOULDER_R);
    JOINT_REVOLUTE_AXIS_X(RARM_SHOULDER_Y);
    JOINT_REVOLUTE_AXIS_X(RARM_ELBOW);
    JOINT_REVOLUTE_AXIS_X(RARM_WRIST_Y);
    JOINT_REVOLUTE_AXIS_X(RARM_WRIST_P);
    JOINT_REVOLUTE_AXIS_X(RARM_WRIST_R);
    JOINT_REVOLUTE_AXIS_X(LLEG_HIP_R);
    JOINT_REVOLUTE_AXIS_X(LLEG_HIP_P);
    JOINT_REVOLUTE_AXIS_X(LLEG_HIP_Y);
    JOINT_REVOLUTE_AXIS_X(LLEG_KNEE);
    JOINT_REVOLUTE_AXIS_X(LLEG_ANKLE_P);
    JOINT_REVOLUTE_AXIS_X(LLEG_ANKLE_R);
    JOINT_REVOLUTE_AXIS_X(RLEG_HIP_R);
    JOINT_REVOLUTE_AXIS_X(RLEG_HIP_P);
    JOINT_REVOLUTE_AXIS_X(RLEG_HIP_Y);
    JOINT_REVOLUTE_AXIS_X(RLEG_KNEE);
    JOINT_REVOLUTE_AXIS_X(RLEG_ANKLE_P);
    JOINT_REVOLUTE_AXIS_X(RLEG_ANKLE_R);
  } // end of namespace simple_humanoid
} // end of namespace metapod

#endif
