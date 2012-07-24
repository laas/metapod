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
 * 31 dof sample model bodies declaration.
 */

#ifndef METAPOD_SAMPLE_31_DOF_BODY_HH
# define METAPOD_SAMPLE_31_DOF_BODY_HH

# include "metapod/tools/bodymacros.hh"

namespace metapod
{
  namespace sample_31_dof
  {
    CREATE_BODY(B0, 0, NP, J0);
    CREATE_BODY(B1, 1, B0, J1);
    CREATE_BODY(B2, 1, B1, J2);
    CREATE_BODY(B3, 1, B2, J3);
    CREATE_BODY(B4, 1, B3, J4);
    CREATE_BODY(B5, 1, B3, J5);
    CREATE_BODY(B6, 1, B2, J6);
    CREATE_BODY(B7, 1, B6, J7);
    CREATE_BODY(B8, 1, B6, J8);
    CREATE_BODY(B9, 1, B1, J9);
    CREATE_BODY(B10, 1, B9, J10);
    CREATE_BODY(B11, 1, B10, J11);
    CREATE_BODY(B12, 1, B10, J12);
    CREATE_BODY(B13, 1, B9, J13);
    CREATE_BODY(B14, 1, B13, J14);
    CREATE_BODY(B15, 1, B13, J15);
    CREATE_BODY(B16, 1, B0, J16);
    CREATE_BODY(B17, 1, B16, J17);
    CREATE_BODY(B18, 1, B17, J18);
    CREATE_BODY(B19, 1, B18, J19);
    CREATE_BODY(B20, 1, B18, J20);
    CREATE_BODY(B21, 1, B17, J21);
    CREATE_BODY(B22, 1, B21, J22);
    CREATE_BODY(B23, 1, B21, J23);
    CREATE_BODY(B24, 1, B16, J24);
    CREATE_BODY(B25, 1, B24, J25);
    CREATE_BODY(B26, 1, B25, J26);
    CREATE_BODY(B27, 1, B25, J27);
    CREATE_BODY(B28, 1, B24, J28);
    CREATE_BODY(B29, 1, B28, J29);
    CREATE_BODY(B30, 1, B28, J30);
  } // end of namespace sample_31_dof
} // end of namespace metapod

#endif
