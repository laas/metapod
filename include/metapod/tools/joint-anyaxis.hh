// Copyright 2013
//
// Sébastien Barthélémy (Aldebaran Robotics)
// Olivier Stasse (CNRS/LAAS)
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

#ifndef METAPOD_JOINT_ANY_AXIS_HH
# define METAPOD_JOINT_ANY_AXIS_HH
#include <metapod/tools/common.hh>

namespace metapod
{
  template <typename FloatType>
  class RevoluteAxisAnyJoint
  {
    EIGEN_METAPOD_TYPEDEFS;
    EIGEN_METAPOD_TRANSFORM_TYPEDEFS;
    EIGEN_METAPOD_CM_TYPEDEFS;
    EIGEN_METAPOD_SPATIAL_MOTION_TYPEDEF;
    EIGEN_METAPOD_SPATIAL_FORCE_TYPEDEF;

  public:
    static const int NBDOF = 1;
    Transform Xj;
    Motion cj; // used in rnea
    Motion vj; // used in rnea
    Spatial::ConstraintMotionAnyAxis<FloatType> S;
    Force f; // used by rnea
    Vector1d torque; // used by rnea
    Vector3d axis_;

    RevoluteAxisAnyJoint(double axis_x, double axis_y, double axis_z):
      cj(Motion::Zero()),
      S(axis_x, axis_y, axis_z),
      axis_(axis_x, axis_y, axis_z)
    {
      vj.v(Vector3d::Zero());
    }

    inline void bcalc(const Vector1d& qi)
    {
      const FloatType angle = qi[0];
      Matrix3d localR;
      localR = Eigen::AngleAxis<FloatType>(-angle, axis_);
      Xj = Transform(localR, Vector3d::Zero());
    }

    inline void jcalc(const Vector1d& qi,
                      const Vector1d& dqi)
    {
      bcalc(qi);
      vj.w(dqi[0] * axis_);
    }
  };
}
#endif /* METAPOD_JOINT_ANY_AXIS_HH */
