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


#ifndef METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_FWD_HH
# define METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_FWD_HH

# include "metapod/tools/fwd.hh"

namespace metapod
{

  namespace Spatial
  {

    // Constraint motion for one specific axis.
    enum AxisType:int  { AxisX=0,AxisY,AxisZ };

    template <int axis> 
    struct vector6dMakerOneAxis 
    {
      vector6d v;
      vector6dMakerOneAxis()
      {
	v[axis]=1;
      }
    };

    class Transform;

    template <int axis>
    class ConstraintMotionOneAxis
    {
      public:
        // Constructors
        ConstraintMotionOneAxis(){ 
	  m_S(axis) = 1.0;
	}

        vector6d operator*(const Spatial::Transform &X) const;
      private:
        vector6d m_S;
      public:
        const vector6d & S() const {return m_S;}
        vector6dt transpose() const {return m_S.transpose();}
    };

    typedef ConstraintMotionOneAxis<AxisX> ConstraintMotionOneAxisAxisX;
    typedef ConstraintMotionOneAxis<AxisY> ConstraintMotionOneAxisAxisY;
    typedef ConstraintMotionOneAxis<AxisZ> ConstraintMotionOneAxisAxisZ;

  } // End of spatial namespace
} // End of metapod namespace
#endif /* METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_FWD_HH */
