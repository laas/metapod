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
 * This file defines the macros that create the joint classes and their
 * corresponding jcalc routine.
 */

#ifndef METAPOD_JOINT_MACROS_HH
# define METAPOD_JOINT_MACROS_HH

namespace metapod
{

  // Create a revolute joint class
  # define JOINT_REVOLUTE(classname)                                \
    class classname                                                 \
    {                                                               \
      public:                                                       \
      enum { NBDOF = 1 };                                           \
      static const std::string name;                                \
      static const int label;                                       \
      static const int nbDof;                                       \
      static const int positionInConf;                              \
      static const Transform Xt;                                    \
      static Transform sXp;                                         \
      static Transform Xj;                                          \
      static Motion cj;                                             \
      static Motion vj;                                             \
      static const vector6d S;                                      \
      static const vector6d dotS;                                   \
      static Force f;                                               \
      static FloatType torque;                                      \
                                                                    \
      static void jcalc(const vector1d & qi, const vector1d & dqi); \
    };                                                              \
    Transform classname::sXp;                                       \
    Transform classname::Xj;                                        \
    Motion classname::cj;                                           \
    Motion classname::vj;                                           \
    Force classname::f;                                             \
    FloatType classname::torque;                                    \
    const vector6d classname::S = vector6dMaker(1, 0, 0, 0, 0, 0);  \
    const vector6d classname::dotS = vector6d::Zero();              \
    void classname::jcalc(const vector1d & qi,                      \
                          const vector1d & dqi)                     \
    {                                                               \
      FloatType angle = qi[0];                                      \
      matrix3d localR;                                              \
      ROTX(angle, localR);                                          \
      localR.transposeInPlace();                                    \
      Xj = Spatial::Transform(localR, vector3d::Zero());            \
      sXp = Xj*Xt;                                                  \
                                                                    \
      /* maj vj */                                                  \
      vj.w(vector3d(dqi[0], 0, 0));                                 \
    }                                                               \
    const int classname::nbDof = 1
  
  // Create a free flyer class
  # define JOINT_FREE_FLYER(classname)                              \
    class classname                                                 \
    {                                                               \
      public:                                                       \
      enum { NBDOF = 6 };                                           \
      static const std::string name;                                \
      static const int label;                                       \
      static const int nbDof;                                       \
      static const int positionInConf;                              \
      static const Transform Xt;                                    \
      static Transform sXp;                                         \
      static Transform Xj;                                          \
      static Motion cj;                                             \
      static Motion vj;                                             \
      static matrix6d S;                                            \
      static matrix6d dotS;                                         \
      static Force f;                                               \
      static vector6d torque;                                       \
                                                                    \
      static void jcalc(const vector6d & qi, const vector6d & dqi); \
    };                                                              \
                                                                    \
    Transform classname::sXp;                                       \
    Transform classname::Xj;                                        \
    Motion classname::cj;                                           \
    Motion classname::vj;                                           \
    Force classname::f;                                             \
    vector6d classname::torque;                                     \
    matrix6d classname::S = matrix6d::Zero();                       \
    matrix6d classname::dotS = matrix6d::Zero();                    \
    void classname::jcalc(const vector6d & qi,                      \
                          const vector6d & dqi)                     \
    {                                                               \
      /* maj sXp */                                                 \
      vector3d qlin = qi.segment<3>(0),                             \
               qang = qi.segment<3>(3);                             \
      matrix3d localR;                                              \
      EULERXYZ(qang, localR);                                       \
      localR.transposeInPlace();                                    \
      S.block<3,3>(0,3) = S.block<3,3>(3,0) = localR;               \
      Xj = Transform(localR, vector3d::Zero());                     \
      sXp = Xj*Transform(matrix3d::Identity(), qlin);               \
      /* maj vj */                                                  \
      vj = Motion(S*dqi);                                           \
    }                                                               \
    const int classname::nbDof = 6

} // end of namespace metapod.

#endif
