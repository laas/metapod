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
 * This file provides ad-hoc macros for fixed-size small matrix and vectors
 */

#ifndef METAPOD_SMALL_MATRIX_MACROS_HH
# define METAPOD_SMALL_MATRIX_MACROS_HH

namespace metapod
{

  # define SKEW(v, m)                                 \
    {                                                 \
      m(0,0) =  0;    m(0,1) = -v(2); m(0,2) =  v(1); \
      m(1,0) =  v(2); m(1,1) =  0;    m(1,2) = -v(0); \
      m(2,0) = -v(1); m(2,1) =  v(0); m(2,2) =  0;    \
    }
  
  # define ROTX(angle, localRot) \
    {                            \
      localRot(0,0)=1;           \
      localRot(1,0)=0;           \
      localRot(2,0)=0;           \
                                 \
      localRot(0,1)=0;           \
      localRot(1,1)=cos(angle);  \
      localRot(2,1)=sin(angle);  \
                                 \
      localRot(0,2)=0;           \
      localRot(1,2)=-sin(angle); \
      localRot(2,2)=cos(angle);  \
    }
  
  # define EULERXYZ(qi_ang, localRot)                                \
    {                                                                \
      FloatType CosTheta, SinTheta,                                  \
                CosPhi,   SinPhi,                                    \
                CosPsi,   SinPsi;                                    \
                                                                     \
      CosPsi   = cos(qi_ang(0));                                     \
      SinPsi   = sin(qi_ang(0));                                     \
      CosTheta = cos(qi_ang(1));                                     \
      SinTheta = sin(qi_ang(1));                                     \
      CosPhi   = cos(qi_ang(2));                                     \
      SinPhi   = sin(qi_ang(2));                                     \
                                                                     \
      localRot(0,0) =  CosTheta * CosPhi;                            \
      localRot(1,0) =  CosTheta * SinPhi;                            \
      localRot(2,0) = -SinTheta;                                     \
                                                                     \
      localRot(0,1) = -CosPsi * SinPhi + CosPhi * SinPsi * SinTheta; \
      localRot(1,1) =  CosPsi * CosPhi + SinPsi * SinTheta * SinPhi; \
      localRot(2,1) =  CosTheta * SinPsi;                            \
                                                                     \
      localRot(0,2) =  CosPsi * CosPhi * SinTheta + SinPhi * SinPsi; \
      localRot(1,2) = -CosPhi * SinPsi + CosPsi * SinTheta * SinPhi; \
      localRot(2,2) =  CosPsi * CosTheta;                            \
    }

} // end of namespace metapod.

#endif
