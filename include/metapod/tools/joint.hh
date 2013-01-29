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

#ifndef METAPOD_JOINT_HH
# define METAPOD_JOINT_HH
#include <metapod/tools/common.hh>

namespace metapod
{

  class RevoluteAxisXJoint
  {
  public:
    RevoluteAxisXJoint();
    static const int NBDOF = 1;
    Spatial::Transform Xj;
    Spatial::Motion cj; // used in rnea
    Spatial::Motion vj; // used in rnea
    Spatial::ConstraintMotionOneAxis<Spatial::AxisX> S;
    Spatial::Force f; // used by rnea
    Vector1d torque; // used by rnea

    void bcalc(const Vector1d& qi);
    void jcalc(const Vector1d& qi, const Vector1d& dqi);
  };

  inline RevoluteAxisXJoint::RevoluteAxisXJoint():
    cj(Spatial::Motion::Zero())
  {
  }

  inline void RevoluteAxisXJoint::bcalc(const Vector1d & qi)
  {
    const FloatType angle = qi[0];
    Matrix3d localR;
    FloatType c = cos(angle), s = sin(angle);
    localR(0,0) =  1; localR(0,1) =  0; localR(0,2) =  0;
    localR(1,0) =  0; localR(1,1) =  c; localR(1,2) =  s;
    localR(2,0) =  0; localR(2,1) = -s; localR(2,2) =  c;
    Xj = Spatial::Transform(localR, Vector3d::Zero());
  }

  inline void RevoluteAxisXJoint::jcalc(const Vector1d & qi,
                                   const Vector1d & dqi)
  {
    bcalc(qi);
    vj.w(Vector3d(dqi[0], 0, 0));
  }

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
    const Vector3d axis_;

    void bcalc(const Vector1d& qi);
    void jcalc(const Vector1d& qi, const Vector1d& dqi);
  };

  inline RevoluteAxisAnyJoint::RevoluteAxisAnyJoint(
      double axis_x, double axis_y, double axis_z):
    cj(Spatial::Motion::Zero()),
    S(axis_x, axis_y, axis_z),
    axis_(axis_x, axis_y, axis_z)
  {
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

  class FreeFlyerJoint
  {
  public:
    FreeFlyerJoint();
    static const int NBDOF = 6;
    Spatial::Transform Xj;
    Spatial::Motion cj; // used in rnea
    Spatial::Motion vj; // used in rnea
    Spatial::ConstraintMotionFreeFlyer S;
    Spatial::Force f; // used by rnea
    Vector6d torque; // used by rnea

    void bcalc(const Vector6d& qi);
    void jcalc(const Vector6d& qi, const Vector6d& dqi);
  };

  inline FreeFlyerJoint::FreeFlyerJoint()
  {
  }

  inline void FreeFlyerJoint::bcalc(const Vector6d& qi)
  {
    FloatType cPsi   = cos(qi(3)), sPsi   = sin(qi(3)),
              cTheta = cos(qi(4)), sTheta = sin(qi(4)),
              cPhi   = cos(qi(5)), sPhi   = sin(qi(5));
    // localR = rx(Psi) * ry(Theta) * rz(Phi)
    Matrix3d localR;
    localR(0,0) = cTheta * cPhi;
    localR(0,1) = cTheta * sPhi;
    localR(0,2) = -sTheta;
    localR(1,0) = -cPsi * sPhi + cPhi * sPsi * sTheta;
    localR(1,1) = cPsi * cPhi + sPsi * sTheta * sPhi;
    localR(1,2) = cTheta * sPsi;
    localR(2,0) = cPsi * cPhi * sTheta + sPhi * sPsi;
    localR(2,1) = -cPhi * sPsi + cPsi * sTheta * sPhi;
    localR(2,2) = cPsi * cTheta;
    S.setlocalR(localR);
    Xj = Spatial::Transform(localR, qi.segment<3>(0));
  }

  inline void FreeFlyerJoint::jcalc(const Vector6d& qi,
                                    const Vector6d& dqi)
  {
    bcalc(qi);
    vj = Spatial::Motion(S.S()*dqi);
    Matrix6d dotS = Matrix6d::Zero();
    Matrix3d localDotR = Spatial::skew (dqi.segment<3>(3))
      * S.S().block<3,3>(0,3);
    dotS.block<3,3>(0,3) = dotS.block<3,3>(3,0) = localDotR;
    cj = Spatial::Motion(dotS*dqi);
  }
}

#endif
