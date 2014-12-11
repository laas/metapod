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

#ifndef METAPOD_JOINT_FREE_FLYER_HH
# define METAPOD_JOINT_FREE_FLYER_HH
#include <metapod/tools/common.hh>

namespace metapod
{
  template<typename FloatType>
  class FreeFlyerJoint
  {
    EIGEN_METAPOD_TYPEDEFS;
    EIGEN_METAPOD_TRANSFORM_TYPEDEFS;
    EIGEN_METAPOD_CM_TYPEDEFS;
    EIGEN_METAPOD_SPATIAL_MOTION_TYPEDEF;
    EIGEN_METAPOD_SPATIAL_FORCE_TYPEDEF;
  public:
    FreeFlyerJoint() {} ;
    static const int NBDOF = 6;
    Transform Xj;
    Motion cj; // used in rnea
    Motion vj; // used in rnea
    ConstraintMotionFreeFlyer S;
    Force f; // used by rnea
    Vector6d torque; // used by rnea

    inline void bcalc(const Vector6d& qi)
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
      Xj = Transform(localR, qi.template segment<3>(0));
    }
 
    inline void jcalc(const Vector6d& qi,
                      const Vector6d& dqi)
    {
      bcalc(qi);
      vj = Motion(S.S()*dqi);
      Matrix6d dotS = Matrix6d::Zero();
      Matrix3d localDotR = static_cast<Matrix3d>(Spatial::skew<FloatType>(dqi.template segment<3>(3))
                                                 * S.S().template block<3,3>(0,3));
      dotS.template block<3,3>(0,3) = dotS.template block<3,3>(3,0) = localDotR;
      cj = Motion(dotS*dqi);
    }
  };
}
#endif /* METAPOD_JOINT_FREE_FLYER_HH */
