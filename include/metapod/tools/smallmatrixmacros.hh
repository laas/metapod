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
 * This file provides ad-hoc macros for fixed-size small matrix and vectors, for enhanced performances
 */

#ifndef metapod_SMALL_MATRIX_MACROS_HH
# define metapod_SMALL_MATRIX_MACROS_HH

namespace metapod
{

  // MATRIX
  # define S3x3_MATRIX_SET_IDENTITY(matrix)   \
    {                                         \
      FloatType* rawM = matrix.data();        \
      for(int i=0; i<9; i++){ rawM[i] = 0; }  \
      for(int i=0; i<9; i+=4){ rawM[i] = 1; } \
    }
  
  # define S3x3_MATRIX_SET_ZERO(matrix)      \
    {                                        \
      FloatType* rawM = matrix.data();       \
      for(int i=0; i<9; i++){ rawM[i] = 0; } \
    }
  
  # define S6x6_MATRIX_SET_ZERO(matrix)       \
    {                                         \
      FloatType* rawM = matrix.data();        \
      for(int i=0; i<36; i++){ rawM[i] = 0; } \
    }
  
  # define S6x6_MATRIX_SET_IDENTITY(matrix)    \
    {                                          \
      FloatType* rawM = matrix.data();         \
      for(int i=0; i<36; i++){ rawM[i] = 0; }  \
      for(int i=0; i<36; i+=7){ rawM[i] = 1; } \
    }
  
  // VECTORS
  # define S3_VECTOR_SET_ZERO(vector)          \
    {                                          \
      for(int i=0; i<3; i++){ vector[i] = 0; } \
    }
  
  # define S3_VECTOR_FILL(vector, value)           \
    {                                              \
      for(int i=0; i<3; i++){ vector[i] = value; } \
    }
  
  # define S3_VECTOR_CROSS_PRODUCT(res,v1,v2) \
      if ((v1.size()==3) && (v2.size()==3))       \
        {                                         \
          res[0] = v1[1] * v2[2] - v2[1] * v1[2]; \
          res[1] = v1[2] * v2[0] - v2[2] * v1[0]; \
          res[2] = v1[0] * v2[1] - v2[0] * v1[1]; \
        }
  
  # define S6_VECTOR_SET_ZERO(vector)          \
    {                                          \
      for(int i=0; i<6; i++){ vector[i] = 0; } \
    }
  
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
      FloatType CosTheta, SinTheta,                                     \
             CosPhi, SinPhi,                                         \
             CosPsi, SinPsi;                                         \
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
