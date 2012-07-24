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
 * 63 dof sample model bodies declaration.
 */

#ifndef METAPOD_SAMPLE_63_DOF_BODY_HH
# define METAPOD_SAMPLE_63_DOF_BODY_HH

# include "metapod/tools/bodymacros.hh"

namespace metapod
{
  namespace sample_63_dof
  {
    CREATE_BODY(B0, 0, NP, J0);
    CREATE_BODY(B1, 1, B0, J1);
    CREATE_BODY(B2, 1, B1, J2);
    CREATE_BODY(B3, 1, B2, J3);
    CREATE_BODY(B4, 1, B3, J4);
    CREATE_BODY(B5, 1, B4, J5);
    CREATE_BODY(B6, 1, B4, J6);
    CREATE_BODY(B7, 1, B3, J7);
    CREATE_BODY(B8, 1, B7, J8);
    CREATE_BODY(B9, 1, B7, J9);
    CREATE_BODY(B10, 1, B2, J10);
    CREATE_BODY(B11, 1, B10, J11);
    CREATE_BODY(B12, 1, B11, J12);
    CREATE_BODY(B13, 1, B11, J13);
    CREATE_BODY(B14, 1, B10, J14);
    CREATE_BODY(B15, 1, B14, J15);
    CREATE_BODY(B16, 1, B14, J16);
    CREATE_BODY(B17, 1, B1, J17);
    CREATE_BODY(B18, 1, B17, J18);
    CREATE_BODY(B19, 1, B18, J19);
    CREATE_BODY(B20, 1, B19, J20);
    CREATE_BODY(B21, 1, B19, J21);
    CREATE_BODY(B22, 1, B18, J22);
    CREATE_BODY(B23, 1, B22, J23);
    CREATE_BODY(B24, 1, B22, J24);
    CREATE_BODY(B25, 1, B17, J25);
    CREATE_BODY(B26, 1, B25, J26);
    CREATE_BODY(B27, 1, B26, J27);
    CREATE_BODY(B28, 1, B26, J28);
    CREATE_BODY(B29, 1, B25, J29);
    CREATE_BODY(B30, 1, B29, J30);
    CREATE_BODY(B31, 1, B29, J31);
    CREATE_BODY(B32, 1, B0, J32);
    CREATE_BODY(B33, 1, B32, J33);
    CREATE_BODY(B34, 1, B33, J34);
    CREATE_BODY(B35, 1, B34, J35);
    CREATE_BODY(B36, 1, B35, J36);
    CREATE_BODY(B37, 1, B35, J37);
    CREATE_BODY(B38, 1, B34, J38);
    CREATE_BODY(B39, 1, B38, J39);
    CREATE_BODY(B40, 1, B38, J40);
    CREATE_BODY(B41, 1, B33, J41);
    CREATE_BODY(B42, 1, B41, J42);
    CREATE_BODY(B43, 1, B42, J43);
    CREATE_BODY(B44, 1, B42, J44);
    CREATE_BODY(B45, 1, B41, J45);
    CREATE_BODY(B46, 1, B45, J46);
    CREATE_BODY(B47, 1, B45, J47);
    CREATE_BODY(B48, 1, B32, J48);
    CREATE_BODY(B49, 1, B48, J49);
    CREATE_BODY(B50, 1, B49, J50);
    CREATE_BODY(B51, 1, B50, J51);
    CREATE_BODY(B52, 1, B50, J52);
    CREATE_BODY(B53, 1, B49, J53);
    CREATE_BODY(B54, 1, B53, J54);
    CREATE_BODY(B55, 1, B53, J55);
    CREATE_BODY(B56, 1, B48, J56);
    CREATE_BODY(B57, 1, B56, J57);
    CREATE_BODY(B58, 1, B57, J58);
    CREATE_BODY(B59, 1, B57, J59);
    CREATE_BODY(B60, 1, B56, J60);
    CREATE_BODY(B61, 1, B60, J61);
    CREATE_BODY(B62, 1, B60, J62);
  } // end of namespace sample_63_dof
} // end of namespace metapod

#endif
