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


#ifndef METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_FREE_FLYER_HH
# define METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_FREE_FLYER_HH

# include "metapod/tools/fwd.hh"

namespace metapod
{

  namespace Spatial
  {
    // Class of motion constraint with a free flyer.
    class ConstraintMotionFreeFlyer
    {
      public:
        // Constructors
      ConstraintMotionFreeFlyer()
      { m_S = matrix6d::Zero(); };

      matrix6d operator*(double d) const;

      private:
        matrix6d m_S;

      public:
      void setlocalR(const matrix3d &localR)  
      {       m_S.block<3,3>(0,3) = m_S.block<3,3>(3,0) = localR; }
      const matrix6d & S() const {return m_S;}
      matrix6d transpose() const {return m_S.transpose();}
    };

    matrix6d ConstraintMotionFreeFlyer::operator*
    (double x) const
    {
      matrix6d tmp = matrix6d::Zero();
      tmp = x*m_S;                    
      return tmp;                                                   
    }

    template<>
    matrix6d OperatorMul< matrix6d, Inertia, ConstraintMotionFreeFlyer>::
      mul(const Inertia & m,
	  const ConstraintMotionFreeFlyer &a) const
    {
      matrix6d r;
      r=matrix6d::Zero();
      r.block<3,3>(0,0)=skew(m.h())*a.S().block<3,3>(3,0);
      r.block<3,3>(0,3)=m.I()*a.S().block<3,3>(0,3);
      r.block<3,3>(3,0)=m.m()*a.S().block<3,3>(3,0);
      r.block<3,3>(3,3)=-skew(m.h())*a.S().block<3,3>(0,3);
      return r;
    }
    
    matrix6d operator*(const Inertia & m,
		       const ConstraintMotionFreeFlyer &a) 
    {
      OperatorMul<matrix6d,Inertia, ConstraintMotionFreeFlyer > om;
      return om.mul(m,a);
    }
  } // End of spatial namespace
} // End of metapod namespace
#endif /* METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_FREE_FLYER_HH */
