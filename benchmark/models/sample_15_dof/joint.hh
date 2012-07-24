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
 * 15 dof sample model joints declaration.
 */

#ifndef METAPOD_SAMPLE_15_DOF_JOINT_HH
# define METAPOD_SAMPLE_15_DOF_JOINT_HH

# include "metapod/tools/jointmacros.hh"

namespace metapod
{
  namespace sample_15_dof
  {
    JOINT_REVOLUTE(J0);
    JOINT_REVOLUTE(J1);
    JOINT_REVOLUTE(J2);
    JOINT_REVOLUTE(J3);
    JOINT_REVOLUTE(J4);
    JOINT_REVOLUTE(J5);
    JOINT_REVOLUTE(J6);
    JOINT_REVOLUTE(J7);
    JOINT_REVOLUTE(J8);
    JOINT_REVOLUTE(J9);
    JOINT_REVOLUTE(J10);
    JOINT_REVOLUTE(J11);
    JOINT_REVOLUTE(J12);
    JOINT_REVOLUTE(J13);
    JOINT_REVOLUTE(J14);
  } // end of namespace sample_15_dof
} // end of namespace metapod

#endif
