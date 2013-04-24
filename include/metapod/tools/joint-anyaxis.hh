// Copyright 2013
//
// Sébastien Barthélémy (Aldebaran Robotics)
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
  class RevoluteAxisAnyJoint
  {
  public:
    RevoluteAxisAnyJoint(double axis_x, double axis_y, double axis_z);
    static const int NBDOF = 1;
    Spatial::Transform Xj;
    Spatial::Motion cj; // used in rnea
    Spatial::Motion vj; // used in rnea
    Spatial::ConstraintMotionAnyAxis S;
    Spatial::Force f; // used by rnea
    Vector1d torque; // used by rnea
    Vector3d axis_;

    void bcalc(const Vector1d& qi);
    void jcalc(const Vector1d& qi, const Vector1d& dqi);
  };

  inline RevoluteAxisAnyJoint::RevoluteAxisAnyJoint(
      double axis_x, double axis_y, double axis_z):
    cj(Spatial::Motion::Zero()),
    S(axis_x, axis_y, axis_z),
    axis_(axis_x, axis_y, axis_z)
  {
    vj.v(Vector3d::Zero());
  }

  inline void RevoluteAxisAnyJoint::bcalc(const Vector1d& qi)
  {
    const FloatType angle = qi[0];
    Matrix3d localR;
    localR = AngleAxisd(-angle, axis_);
    Xj = Spatial::Transform(localR, Vector3d::Zero());
  }

  inline void RevoluteAxisAnyJoint::jcalc(const Vector1d& qi,
                                          const Vector1d& dqi)
  {
    bcalc(qi);
    vj.w(dqi[0] * axis_);
  }
}
#endif /* METAPOD_JOINT_ANY_AXIS_HH */
