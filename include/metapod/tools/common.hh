// Copyright 2011, 2012, 2013, 2014
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
// Sébastien Barthélémy (Aldebaran Robotics)
// Nuno Guedelha (LAAS, CNRS)
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

#ifndef METAPOD_COMMON_HH
# define METAPOD_COMMON_HH

# include "metapod/config.hh"
# include "metapod/tools/fwd.hh"
# include <metapod/tools/constants.hh>
# include "metapod/macro.hh"

namespace metapod
{
// Constant 3x3 Matrix initialization method.
template <class FloatType>
inline const struct Matrix3dTpl<FloatType>::Type matrix3dMaker(
    FloatType m00, FloatType m01, FloatType m02,
    FloatType m10, FloatType m11, FloatType m12,
    FloatType m20, FloatType m21, FloatType m22)
{
  struct Matrix3dTpl<FloatType>::Type m;
  m(0,0) = m00; m(0,1) = m01; m(0,2) = m02;
  m(1,0) = m10; m(1,1) = m11; m(1,2) = m12;
  m(2,0) = m20; m(2,1) = m21; m(2,2) = m22;
  return m;
}

// Constant size 6 Vector initialization method.
template <class FloatType>
inline const struct Vector6dTpl<FloatType>::Type 
vector6dMaker(FloatType v0,
              FloatType v1,
              FloatType v2,
              FloatType v3,
              FloatType v4,
              FloatType v5)
{
  EIGEN_METAPOD_TYPEDEFS;
  Vector6d v;
  v[0] = v0; v[1] = v1; v[2] = v2; v[3] = v3;  v[4] = v4; v[5] = v5;
  return v;
}

}

# include "metapod/tools/static_assert.hh"
# include "metapod/tools/spatial.hh"
# include <fstream>
# include <iostream>

namespace metapod
{

template <typename FloatType>
class Body
{
  EIGEN_METAPOD_TRANSFORM_TYPEDEFS;
  EIGEN_METAPOD_SPATIAL_MOTION_TYPEDEF;
  EIGEN_METAPOD_SPATIAL_INERTIA_TYPEDEF;
  EIGEN_METAPOD_SPATIAL_FORCE_TYPEDEF;
 public:
  Force Fext; // input for rnea
  Transform iX0; // temporary/result used in rnea and bcalc
  Motion vi;// temporary used in rnea
  Motion ai; // temporary used in rnea
  Inertia Iic; // temporary used in crba
  Body():
      Fext(Force::Zero()),
      iX0(),
      vi(),
      ai(),
      Iic()
  {}
};

// specializations should map robot node_ids to node classes.
template <typename Robot, int id>
struct Nodes {};

template <typename FloatType, int gravity>
inline Spatial::MotionTpl<FloatType> set_gravity()
{
  const FloatType g = (FloatType)gravity/100;
  class Vector6dTpl<FloatType>::Type g_tmp;
  g_tmp << 0,0,0,0,0,g;
  return Spatial::MotionTpl<FloatType>(g_tmp);
}

template <typename FloatType, int gravity>
class GravityConstant
{
public:
  static const Spatial::MotionTpl<FloatType> minus_g;
};

template<typename FloatType, int gravity>
const Spatial::MotionTpl<FloatType> GravityConstant<FloatType, gravity>::minus_g = set_gravity<FloatType, gravity>();

template <typename FloatType>
inline class Matrix3dTpl<FloatType>::Type 
Skew(const class Vector3dTpl<FloatType>::Type & v)
{
  class Matrix3dTpl<FloatType>::Type m;
  m(0,0) = 0;     m(0,1) = -v(2); m(0,2) = v(1);
  m(1,0) = v(2);  m(1,1) = 0    ; m(1,2) = -v(0);
  m(2,0) = -v(1); m(2,1) =  v(0); m(2,2) =  0 ;
  return m;
}

// Constant Spatial::Inertia initialization method.
template <typename FloatType>
inline Spatial::InertiaTpl<FloatType> spatialInertiaMaker(const FloatType m,
                                                          const class Vector3dTpl<FloatType>::Type & CoM,
                                                          const class Matrix3dTpl<FloatType>::Type & inertia)
{
  return Spatial::InertiaTpl<FloatType>(m, CoM*m, inertia + m*(Skew<FloatType>(CoM)*Skew<FloatType>(CoM).transpose()));
}

} // end of namespace metapod.

#endif
