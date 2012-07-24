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
 * 15 dof sample model bodies declaration.
 */

#ifndef METAPOD_SAMPLE_15_DOF_BODY_HH
# define METAPOD_SAMPLE_15_DOF_BODY_HH

# include "metapod/tools/bodymacros.hh"

namespace metapod
{
  namespace sample_15_dof
  {
    CREATE_BODY(B0, 0, NP, J0);
    CREATE_BODY(B1, 1, B0, J1);
    CREATE_BODY(B2, 1, B1, J2);
    CREATE_BODY(B3, 1, B2, J3);
    CREATE_BODY(B4, 1, B2, J4);
    CREATE_BODY(B5, 1, B1, J5);
    CREATE_BODY(B6, 1, B5, J6);
    CREATE_BODY(B7, 1, B5, J7);
    CREATE_BODY(B8, 1, B0, J8);
    CREATE_BODY(B9, 1, B8, J9);
    CREATE_BODY(B10, 1, B9, J10);
    CREATE_BODY(B11, 1, B9, J11);
    CREATE_BODY(B12, 1, B8, J12);
    CREATE_BODY(B13, 1, B12, J13);
    CREATE_BODY(B14, 1, B12, J14);
  } // end of namespace sample_15_dof
} // end of namespace metapod

#endif
