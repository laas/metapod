// Copyright 2011, 2012,
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
// Antonio El Khoury (JRL/LAAS, CNRS/AIST)
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

/*
 * This file defines the macros that create the joint classes and their
 * corresponding jcalc routine.
 */

#ifndef METAPOD_JOINT_MACROS_HH
# define METAPOD_JOINT_MACROS_HH

#include "metapod/tools/spatial.hh"

namespace metapod
{

  // Create a REVOLUTE_AXIS_ANY joint class
  # define JOINT_REVOLUTE_AXIS_ANY(libname, classname, axisx, axisy, axisz)  \
    class libname ## _DLLAPI classname                               \
    {                                                               \
      public:                                                       \
      enum { NBDOF = 1 };                                           \
      static const std::string name;                                \
      static const int label;                                       \
      static const int positionInConf;                              \
      static const Spatial::Transform Xt;                           \
      static Spatial::Transform sXp;                                \
      static Spatial::Transform Xj;                                 \
      static Spatial::Motion cj;                                    \
      static Spatial::Motion vj;                                    \
      static const Spatial::ConstraintMotionAnyAxis S;		    \
      static const Vector6d dotS;                                   \
      static Spatial::Force f;                                      \
      static Vector1d torque;                                       \
      static Vector6d F;                                            \
                                                                    \
      static void bcalc(const Vector1d & qi);                       \
      static void jcalc(const Vector1d & qi, const Vector1d & dqi); \
    };                                                              \
                                                                    \
    inline void classname::bcalc(const Vector1d & qi)               \
    {                                                               \
      FloatType angle = qi[0];                                      \
      Matrix3d localR;                                              \
      localR =                                                      \
          AngleAxisd(-angle, Vector3d(axisx, axisy, axisz));        \
      Xj = Spatial::Transform(localR, Vector3d::Zero());            \
      sXp = Xj*Xt;                                                  \
    }                                                               \
                                                                    \
    inline void classname::jcalc(const Vector1d & qi,               \
                                 const Vector1d & dqi)              \
    {                                                               \
      bcalc(qi);                                                    \
      /* maj vj */                                                  \
      vj.w(Vector3d(axisx*dqi[0], axisy*dqi[0], axisz*dqi[0]));     \
    }                                                               \
    struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

  # define INITIALIZE_JOINT_REVOLUTE_AXIS_ANY(classname, axisx, axisy, axisz)\
    Spatial::Transform classname::sXp;                              \
    Spatial::Transform classname::Xj;                               \
    Spatial::Motion classname::cj;                                  \
    Spatial::Motion classname::vj;                                  \
    Spatial::Force classname::f;                                    \
    Vector1d classname::torque;                                     \
    const Spatial::ConstraintMotionAnyAxis classname::S(axisx, axisy, axisz); \
    const Vector6d classname::dotS = Vector6d::Zero();              \
    Vector6d classname::F                                           \

  // Create a REVOLUTE_AXIS_X joint class
  # define JOINT_REVOLUTE_AXIS_X(libname, classname)                \
    class libname ## _DLLAPI classname                              \
    {                                                               \
      public:                                                       \
      enum { NBDOF = 1 };                                           \
      static const std::string name;                                \
      static const int label;                                       \
      static const int positionInConf;                              \
      static const Spatial::Transform Xt;                           \
      static Spatial::Transform sXp;                                \
      static Spatial::TransformX Xj;                                \
      static Spatial::Motion cj;                                    \
      static Spatial::Motion vj;                                    \
      static const Spatial::ConstraintMotionOneAxis<Spatial::AxisX> S;	\
      static const Vector6d dotS;                                   \
      static Spatial::Force f;                                      \
      static Vector1d torque;                                       \
      static Vector6d F;                                            \
                                                                    \
      static void bcalc(const Vector1d & qi);                       \
      static void jcalc(const Vector1d & qi, const Vector1d & dqi); \
    };                                                              \
                                                                    \
    inline void classname::bcalc(const Vector1d & qi)               \
    {                                                               \
      FloatType angle = qi[0];                                      \
      Matrix3d localR;                                              \
      FloatType c = cos(angle), s = sin(angle);                     \
      localR(0,0) =  1; localR(0,1) =  0; localR(0,2) =  0;         \
      localR(1,0) =  0; localR(1,1) =  c; localR(1,2) =  s;         \
      localR(2,0) =  0; localR(2,1) = -s; localR(2,2) =  c;         \
      Xj = Spatial::TransformX(Spatial::RotationMatrixAboutX(c,s),Vector3d::Zero());	    \
      sXp = Xj*Xt;                                                  \
    }                                                               \
                                                                    \
    inline void classname::jcalc(const Vector1d & qi,               \
                                 const Vector1d & dqi)              \
    {                                                               \
      bcalc(qi);                                                    \
      /* maj vj */                                                  \
      vj.w(Vector3d(dqi[0], 0, 0));                                 \
    }                                                               \
    struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

  # define INITIALIZE_JOINT_REVOLUTE_AXIS_X(classname)              \
    Spatial::Transform classname::sXp;                              \
    Spatial::TransformX classname::Xj;                               \
    Spatial::Motion classname::cj;                                  \
    Spatial::Motion classname::vj;                                  \
    Spatial::Force classname::f;                                    \
    Vector1d classname::torque;                                     \
    const Spatial::ConstraintMotionOneAxis<Spatial::AxisX> classname::S;				    \
    const Vector6d classname::dotS = Vector6d::Zero();              \
    Vector6d classname::F                                           \

  // Create a free flyer class
  # define JOINT_FREE_FLYER(libname, classname)                     \
   class libname ## _DLLAPI classname                               \
    {                                                               \
      public:                                                       \
      enum { NBDOF = 6 };                                           \
      static const std::string name;                                \
      static const int label;                                       \
      static const int positionInConf;                              \
      static const Spatial::Transform Xt;                           \
      static Spatial::Transform sXp;                                \
      static Spatial::Transform Xj;                                 \
      static Spatial::Motion cj;                                    \
      static Spatial::Motion vj;                                    \
      static Spatial::ConstraintMotionFreeFlyer S;		    \
      static Matrix6d dotS;                                         \
      static Spatial::Force f;                                      \
      static Vector6d torque;                                       \
      static Matrix6d F;                                            \
                                                                    \
      static void bcalc(const Vector6d & qi);                       \
      static void jcalc(const Vector6d & qi, const Vector6d & dqi); \
    };                                                              \
                                                                    \
    inline void classname::bcalc(const Vector6d & qi)               \
    {                                                               \
      /* maj sXp */                                                 \
      Matrix3d localR;						    \
      FloatType cPsi   = cos(qi(3)), sPsi   = sin(qi(3)),           \
                cTheta = cos(qi(4)), sTheta = sin(qi(4)),           \
                cPhi   = cos(qi(5)), sPhi   = sin(qi(5));           \
      /* localR = rx(Psi) * ry(Theta) * rz(Phi) */                  \
      localR(0,0) = cTheta * cPhi;                                  \
      localR(0,1) = cTheta * sPhi;                                  \
      localR(0,2) = -sTheta;                                        \
      localR(1,0) = -cPsi * sPhi + cPhi * sPsi * sTheta;            \
      localR(1,1) = cPsi * cPhi + sPsi * sTheta * sPhi;             \
      localR(1,2) = cTheta * sPsi;                                  \
      localR(2,0) = cPsi * cPhi * sTheta + sPhi * sPsi;             \
      localR(2,1) = -cPhi * sPsi + cPsi * sTheta * sPhi;            \
      localR(2,2) = cPsi * cTheta;                                  \
      S.setlocalR(localR);                                          \
      Xj = Spatial::Transform(localR, qi.segment<3>(0));            \
      sXp = Xj*Xt;                                                  \
    }                                                               \
                                                                    \
    inline void classname::jcalc(const Vector6d & qi,               \
                                 const Vector6d & dqi)              \
    {                                                               \
      bcalc(qi);                                                    \
      /* maj vj */                                                  \
      vj = Spatial::Motion(S.S()*dqi);				    \
    }                                                               \
    struct e_n_d__w_i_t_h__s_e_m_i_c_o_l_o_n

  # define INITIALIZE_JOINT_FREE_FLYER(classname)                   \
    Spatial::Transform classname::sXp;                              \
    Spatial::Transform classname::Xj;                               \
    Spatial::Motion classname::cj;                                  \
    Spatial::Motion classname::vj;                                  \
    Spatial::Force classname::f;                                    \
    Vector6d classname::torque;                                     \
    Spatial::ConstraintMotionFreeFlyer classname::S;                \
    Matrix6d classname::dotS = Matrix6d::Zero();                    \
    Matrix6d classname::F                                           \

} // end of namespace metapod.

#endif
