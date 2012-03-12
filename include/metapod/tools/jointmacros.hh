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
 * This file defines the macros that create the joints classes and define their jcalc routine.
 */

#ifndef METAPOD_JOINT_MACROS_HH
# define METAPOD_JOINT_MACROS_HH

namespace metapod
{

  // Create a revolute joint class
  # define JOINT_REVOLUTE(classname)                                 \
    class classname                                                  \
    {                                                                \
      public:                                                        \
      enum { NBDOF = 1 };                                            \
      static const std::string name;                                 \
      static const int label;                                        \
      static const int nbDof;                                        \
      static const int positionInConf;                               \
      static const PluckerTransform Xt;                              \
      static PluckerTransform sXp;                                   \
      static PluckerTransform Xj;                                    \
      static vector6d cj;                                            \
      static vector6d vj;                                            \
      static const vector6d S;                                       \
      static const vector6d dotS;                                    \
      static Force f;                                                \
      static double torque;                                          \
                                                                     \
      static void jcalc(const vector1d & qi, const vector1d & dqi);  \
    }
  
  // Create a free flyer class
  # define JOINT_FREE_FLYER(classname)                               \
    class classname                                                  \
    {                                                                \
      public:                                                        \
      enum { NBDOF = 6 };                                            \
      static const std::string name;                                 \
      static const int label;                                        \
      static const int nbDof;                                        \
      static const int positionInConf;                               \
      static const PluckerTransform Xt;                              \
      static PluckerTransform sXp;                                   \
      static PluckerTransform Xj;                                    \
      static vector6d cj;                                            \
      static vector6d vj;                                            \
      static matrix6d S;                                             \
      static matrix6d dotS;                                          \
      static Force f;                                                \
      static vector6d torque;                                        \
                                                                     \
      static void jcalc(const vector6d & qi, const vector6d & dqi);  \
    }
  
  // Define Jcalc method for all revolute joint classes
  # define REVOLUTE_JOINT_JCALC                  \
    {                                            \
      FloatType angle = qi[0];                   \
      matrix3d localR;                           \
      vector3d t;                                \
      ROTX(angle, localR);                       \
      localR.transposeInPlace();                 \
      S3_VECTOR_SET_ZERO(t);                     \
      Xj = Spatial::PluckerTransform(localR, t); \
      Spatial::PluckerTransform tmp = Xt;        \
      sXp = Xj*tmp;                              \
  /* TODO: tmp variable should useless. Doesn't compile as such without it though. */\
                                                 \
      /* maj vj */                               \
      vj[3] = dqi[0];                            \
    }
  
  // Define Jcalc method for all free flyer classes (there should be only one)
  # define FREE_FLYER_JCALC                                                  \
    {                                                                        \
      /* maj sXp */                                                          \
      vector3d qang, qlin;                                                   \
      for(int i=0; i<3; i++)                                                 \
      {                                                                      \
        qlin[i] = qi[i];                                                     \
        qang[i] = qi[i+3];                                                   \
      }                                                                      \
      matrix3d localR;                                                       \
      vector3d t;                                                            \
      EULERXYZ(qang, localR);                                                \
      for(int i=0; i<3; i++)                                                 \
        for(int j=0; j<3; j++)                                               \
          S(i,j) = S(i+3, j+3) = localR(j,i);                                \
      localR.transposeInPlace();                                             \
      S3_VECTOR_SET_ZERO(t);                                                 \
      Xj = Spatial::PluckerTransform(localR, t);                             \
      matrix3d tmp1; S3x3_MATRIX_SET_IDENTITY(tmp1);                         \
      Spatial::PluckerTransform tmp = Spatial::PluckerTransform(tmp1, qlin); \
      sXp = Xj*tmp; \
  \
      /* maj vj */ \
      vj = S*dqi; \
    }
  
  // Define a "no-joint" class, used as a dummy class for end of recursion purpose.
  # define NO_JOINT(classname)                                       \
    class classname                                                  \
    {                                                                \
      public:                                                        \
      enum { NBDOF = 6 };                                            \
      static const int label;                                        \
      static const int nbDof;                                        \
      static const int positionInConf;                               \
      static const PluckerTransform Xt;                              \
      static PluckerTransform sXp;                                   \
      static PluckerTransform Xj;                                    \
      static vector6d cj;                                            \
      static vector6d vj;                                            \
      static matrixN S;                                              \
      static matrixN dotS;                                           \
      static Force f;                                                \
      static vectorN torque;                                         \
                                                                     \
      static void jcalc(const vectorN & qi, const vectorN & dqi);    \
    }

} // end of namespace metapod.

#endif
