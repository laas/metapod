// Copyright 2012,
//
// Olivier Stasse
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


#ifndef METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_IDENTITY_HH
# define METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_IDENTITY_HH

namespace metapod
{

  namespace Spatial
  {
    template<class FloatType>
    struct RotationMatrixIdentityTpl
    {
      EIGEN_METAPOD_TYPEDEFS;
      FloatType m_scalar;

      RotationMatrixIdentityTpl() 
      { m_scalar = 1.0;}
      
      RotationMatrixIdentityTpl(const RotationMatrixIdentityTpl<FloatType> &aR)
      { m_scalar = aR.m_scalar; }

      RotationMatrixIdentityTpl(const FloatType &aR)
      { m_scalar = aR; }


      /** \brief Random initialization */
      void randomInit()
      {  }

      /** \brief Returns a 3x3 eigen matrix */
      Matrix3d toMatrix() const
      {
        Matrix3d r = m_scalar*Matrix3d::Identity();
	return r;
      }
      
      RotationMatrixIdentityTpl transpose() const
      {
	return RotationMatrixIdentityTpl(m_scalar);
      }
      
      Matrix3d operator*(const Matrix3d &A) const
      {
	return static_cast<Matrix3d>(A);
      }

      /** \brief Computes the multiplication between rotation matrix. 
       */
      RotationMatrixIdentityTpl operator* (const RotationMatrixIdentityTpl &arm) const
      {
	return arm;
      }

      /** \brief Computes \$f v = RM u \$f with $\f v,u \in \mathbb{R}^3 $\f 
       */
      Vector3d operator*(const Vector3d &A) const
      {
	return static_cast<Vector3d>(A);
      }


      RotationMatrixIdentityTpl operator*(FloatType a) const
      {
	return RotationMatrixIdentityTpl(a);
      }

      RotationMatrixIdentityTpl operator-() const
      {
	return RotationMatrixIdentityTpl(-m_scalar);
      }

      FloatType operator()(int x, int y) const
      {
	if (x==y)
	  return m_scalar;
	return 0.0;
      }

      friend RotationMatrixIdentityTpl operator*(FloatType a, RotationMatrixIdentityTpl alti)
      {
	return RotationMatrixIdentityTpl(alti.m_scalar *a);

      }
      
      template <typename T> friend
      T operator*(const T &aT, RotationMatrixIdentityTpl alti)
      {
	return aT * alti.m_scalar  ;
      }

      Vector3d col(int x) const
      {
	Vector3d r=Vector3d::Zero();
	r(x) = 1.0;
	return r;
      }

      /** \brief Compute the \$f EAE^{T}\$f 
       */
      Matrix3d rotGeneralMatrix(const Matrix3d &A) const
      {
	return Matrix3d(A);
      }
      

      /** \brief Compute the rotation for a symmetric matrix.
       */
      ltI<FloatType> rotTSymmetricMatrix(const ltI<FloatType> &A) const
      {
	return A;
      }

      ltI<FloatType> rotSymmetricMatrix(const ltI<FloatType> &A) const
      {
	return A;
      }

      friend std::ostream & operator<<(std::ostream &os,
				       const struct RotationMatrixIdentityTpl & aRMAX)
      {
	for(unsigned int li=0;li<3;li++)
	  {
	    for(unsigned int lj=0;lj<3;lj++)
	      {
		if (li!=lj)
		  os << " 0" ;
		else
		  os << " " << aRMAX.m_scalar ;
	      }
	    os << std::endl;
	  }
	return os;
      }

    };
  } // end Spatial namespace
#define METAPOD_SPATIAL_ROTATION_MATRIX_I_TYPEDEF \
  typedef Spatial::RotationMatrixIdentityTpl<FloatType> RotationMatrixIdentity

} // end metapod namespace

#endif // METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_IDENTITY_HH
