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

    struct RotationMatrixIdentity
    {
      
      FloatType m_scalar;

      RotationMatrixIdentity() 
      { m_scalar = 1.0;}
      
      RotationMatrixIdentity(const RotationMatrixIdentity &aR)
      { m_scalar = aR.m_scalar; }

      RotationMatrixIdentity(const FloatType &aR)
      { m_scalar = aR; }


      /** \brief Random initialization */
      void randomInit()
      {  }

      /** \brief Returns a 3x3 eigen matrix */
      Matrix3d toMatrix() const
      {
	Matrix3d r = Matrix3d::Identity();
	return r;
      }
      
      RotationMatrixIdentity transpose() const
      {
	return RotationMatrixIdentity(m_scalar);
      }
      
      Matrix3d operator*(const Matrix3d &A) const
      {
	return static_cast<Matrix3d>(A);
      }

      /** \brief Computes the multiplication between rotation matrix. 
       */
      RotationMatrixIdentity operator* (const RotationMatrixIdentity &arm) const
      {
	return arm;
      }

      /** \brief Computes \$f v = RM u \$f with $\f v,u \in \mathbb{R}^3 $\f 
       */
      Vector3d operator*(const Vector3d &A) const
      {
	return static_cast<Vector3d>(A);
      }


      RotationMatrixIdentity operator*(FloatType a) const
      {
	return RotationMatrixIdentity(a);
      }

      RotationMatrixIdentity operator-() const
      {
	return RotationMatrixIdentity(-m_scalar);
      }

      FloatType operator()(int x, int y) const
      {
	if (x==y)
	  return m_scalar;
	return 0.0;
      }

      friend RotationMatrixIdentity operator*(FloatType a, RotationMatrixIdentity alti)
      {
	return RotationMatrixIdentity(alti.m_scalar *a);

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
      lowerTriangularMatrix rotTSymmetricMatrix(const lowerTriangularMatrix &A) const
      {
	return A;
      }

      lowerTriangularMatrix rotSymmetricMatrix(const lowerTriangularMatrix &A) const
      {
	return A;
      }

      friend std::ostream & operator<<(std::ostream &os,
				       const struct RotationMatrixIdentity & aRMAX)
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
} // end metapod namespace

#endif // METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_IDENTITY_HH
