// Copyright 2013
//
// Maxime Reis (JRL/CNRS/AIST, LAAS/CNRS)
// Sébastien Barthélémy (Aldebaran Robotics)
// Olivier Stasse (LAAS/CNRS)
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

#ifndef METAPOD_JOINT_ABOUT_Z_AXIS_HH
# define METAPOD_JOINT_ABOUT_Z_AXIS_HH
#include <metapod/tools/common.hh>

namespace metapod
{
  template <typename FloatType>
  class RevoluteAxisZJoint
  {
    EIGEN_METAPOD_TYPEDEFS;
    EIGEN_METAPOD_TRANSFORM_TYPEDEFS;
    EIGEN_METAPOD_CM_TYPEDEFS;
    EIGEN_METAPOD_SPATIAL_MOTION_TYPEDEF;
    EIGEN_METAPOD_SPATIAL_FORCE_TYPEDEF;
    METAPOD_SPATIAL_ROTATION_TYPEDEFS;
  public:
    static const int NBDOF = 1;
    TransformZ Xj;
    Motion cj; // used in rnea
    Motion vj; // used in rnea
    Spatial::ConstraintMotionOneAxis<Spatial::AxisZ, FloatType> S;
    Force f; // used by rnea
    Vector1d torque; // used by rnea

    inline RevoluteAxisZJoint(): cj(Motion::Zero())
    {
      vj.v(Vector3d(0.0,0.0,0.0));
    }
    
    void bcalc(const Vector1d & qi)
    {
      const FloatType angle = qi[0];
      FloatType c = cos(angle), s = sin(angle);
      Xj = TransformZ
        (RotationMatrixAboutZ(c,s),
         Vector3d::Zero());
    }

    void jcalc(const Vector1d & qi,
               const Vector1d & dqi)
    {
      bcalc(qi);
      vj.w(Vector3d(0, 0, dqi[0]));
    }
  };
}
#endif

