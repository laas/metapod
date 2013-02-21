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
      ConstraintMotionFreeFlyer(): m_S(Matrix6d::Zero())
      {}

      Matrix6d operator*(FloatType x) const
      {
        return x*m_S;
      }

      Vector6d operator*(const Eigen::Matrix< FloatType, 6, 1 > &ddqi) const
      {
        Vector6d r = static_cast<Vector6d>(m_S *ddqi);
        return r;
      }

      void setlocalR(const Matrix3d &localR)
      {
        m_S.block<3,3>(0,3) = m_S.block<3,3>(3,0) = localR;
      }
      const Matrix6d & S() const { return m_S; }
      Matrix6d transpose() const { return m_S.transpose(); }

    private:
      Matrix6d m_S;
    };

    template<> inline
    Matrix6d OperatorMul< Matrix6d, Inertia, ConstraintMotionFreeFlyer>::
      mul(const Inertia & m,
          const ConstraintMotionFreeFlyer &a) const
    {
      Matrix6d r(Matrix6d::Zero());
      r.block<3,3>(0,0) = skew(m.h())*a.S().block<3,3>(3,0);
      r.block<3,3>(0,3) = m.I()*static_cast<Matrix3d>(a.S().block<3,3>(0,3));
      r.block<3,3>(3,0) = m.m()*a.S().block<3,3>(3,0);
      r.block<3,3>(3,3) = -skew(m.h())*a.S().block<3,3>(0,3);
      return r;
    }

    inline Matrix6d operator*(const Inertia & m,
                              const ConstraintMotionFreeFlyer &a)
    {
      OperatorMul<Matrix6d,Inertia, ConstraintMotionFreeFlyer > om;
      return om.mul(m,a);
    }
  } // End of spatial namespace
} // End of metapod namespace
#endif /* METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_FREE_FLYER_HH */
