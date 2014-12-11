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
    template <class FloatType>
    class ConstraintMotionFreeFlyerTpl
    {
      EIGEN_METAPOD_TYPEDEFS;
    public:
        // Constructors
      ConstraintMotionFreeFlyerTpl(): m_S(Matrix6d::Zero())
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
        m_S.template block<3,3>(0,3) = m_S.template block<3,3>(3,0) = localR;
      }
      const Matrix6d & S() const { return m_S; }
      Matrix6d transpose() const { return m_S.transpose(); }

    private:
      Matrix6d m_S;
    };

    template <class FloatType>
    inline class Matrix6dTpl<FloatType>::Type operator*(const InertiaTpl<FloatType> &m,
                                                        const ConstraintMotionFreeFlyerTpl<FloatType> &a) 
    {
      EIGEN_METAPOD_TYPEDEFS;
      Matrix6d r;
      r.template block<3,3>(0,0) = skew<FloatType>(m.h())*a.S().template block<3,3>(3,0);
      r.template block<3,3>(0,3) = m.I()*static_cast<Matrix3d>(a.S().template block<3,3>(0,3));
      r.template block<3,3>(3,0) = m.m()*a.S().template block<3,3>(3,0);
      r.template block<3,3>(3,3) = -skew<FloatType>(m.h())*a.S().template block<3,3>(0,3);
      return r;
    }

    /// Cross operator between motion and constraint motion free flyer.
    template <class FloatType>
    inline class Matrix6dTpl<FloatType>::Type operator^(const MotionTpl<FloatType> &m,
                                                        const ConstraintMotionFreeFlyerTpl<FloatType> &a) 
    {
      EIGEN_METAPOD_TYPEDEFS;
      Matrix6d r;
      r.template block<3,3>(0,0) = skew<FloatType>(m.v())*a.S().template block<3,3>(3,0);
      r.template block<3,3>(0,3) = skew<FloatType>(m.w())*a.S().template block<3,3>(3,0);
      r.template block<3,3>(3,0) = skew<FloatType>(m.w())*a.S().template block<3,3>(3,0);
      r.template block<3,3>(3,3) = Matrix3d::Zero();
      return r;
    }

  } // End of spatial namespace
} // End of metapod namespace
#endif /* METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_FREE_FLYER_HH */
