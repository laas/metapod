// Copyright 2012,
//
// Olivier STASSE
//
// LAAS, CNRS
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
// GNU General Lesser Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod.  If not, see <http://www.gnu.org/licenses/>.(2);

#ifndef METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_ONEAXIS_HH
# define METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_ONEAXIS_HH

namespace metapod
{
  namespace Spatial
  {
    // Constraint motion for one specific axis.
    enum AxisType { AxisX=0, AxisY, AxisZ };

    template <int axis> 
    struct Vector6dMakerOneAxis 
    {
      Vector6d v;
      Vector6dMakerOneAxis()
      {
        v[axis]=1;
      }
    };

    template <int axis>
    class ConstraintMotionOneAxis
    {
      public:
        // Constructors
      ConstraintMotionOneAxis(){ }

      private:
        static const Vector6d m_S;
      public:
        const Vector6d & S() const {return m_S;}
        Vector6dt transpose() const {return m_S.transpose();}
    };

    // Constant size 6 vector initialization method.
    inline const Vector6d lvector6dMaker(FloatType v0,
                                         FloatType v1,
                                         FloatType v2,
                                         FloatType v3,
                                         FloatType v4,
                                         FloatType v5)
    {
      Vector6d v;
      v[0] = v0; v[1] = v1; v[2] = v2; v[3] = v3;  v[4] = v4; v[5] = v5;
      return v;
    }
    
    template <> const Vector6d ConstraintMotionOneAxis<AxisX>::m_S =lvector6dMaker( 1.0, 0.0, 0.0, 0.0 , 0.0, 0.0);
    template <> const Vector6d ConstraintMotionOneAxis<AxisY>::m_S =lvector6dMaker( 0.0, 1.0, 0.0, 0.0 , 0.0, 0.0);
    template <> const Vector6d ConstraintMotionOneAxis<AxisZ>::m_S =lvector6dMaker( 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0);

    typedef ConstraintMotionOneAxis<AxisX> ConstraintMotionAxisX;
    typedef ConstraintMotionOneAxis<AxisY> ConstraintMotionAxisY;
    typedef ConstraintMotionOneAxis<AxisZ> ConstraintMotionAxisZ;
    
    // Operator Inertia = Inertia * float
    Vector6d operator*(const Inertia & m,
                       const ConstraintMotionAxisX &)
    {
      Vector6d r;
      r[0] = m.I()(0); r[1] = m.I()(1);r[2] = m.I()(3);
      r[3] = 0.0; r[4] = -m.h()(2); r[5] = m.h()(1);
      return r;
    }

    Vector6d operator*(const Inertia & m,
                       const ConstraintMotionAxisY &)
    {
      Vector6d r;
      r[0] = m.I()(1); r[1] = m.I()(2);r[2] = m.I()(4);
      r[3] = m.h()(2); r[4] = 0.0; r[5] = -m.h()(0);
      return r;
    }

    Vector6d operator*(const Inertia & m,
                       const ConstraintMotionAxisZ &)
    {
      Vector6d r;
      r[0] = m.I()(3); r[1] = m.I()(4);r[2] = m.I()(5);
      r[3] = -m.h()(1); r[4] = m.h()(0); r[5] = 0.0;
      return r;
    }
    

  }
}
#endif 
