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


#ifndef METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_GENERAL_HH
# define METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_GENERAL_HH

namespace metapod
{

  namespace Spatial
  {

    struct RotationMatrix
    {
      Matrix3d m_rm;
      
      RotationMatrix() 
      { 
	m_rm = Matrix3d::Zero();
      }
      
      RotationMatrix(const Matrix3d &aR)
      {
	m_rm = aR;
      }
      /** \brief Random initialization */
      void randomInit()
      {
	boost::mt19937 rng;
	boost::uniform_real<> urd(-3.14, 3.14);

	FloatType theta_x=urd(rng);
	FloatType cx = cos(theta_x), sx = sin(theta_x);
	FloatType theta_y=urd(rng),
	  cy = cos(theta_y), sy = sin(theta_y);
	FloatType theta_z=urd(rng),
	  cz = cos(theta_z), sz = sin(theta_z);

	m_rm(0,0) =  cy*cz;
	m_rm(0,1) = -cx*sz + sx*sy*cz;
	m_rm(0,2) =  sx*sz + cx*sy*cz;
	
	m_rm(1,0) =  cy*sz;
	m_rm(1,1) =  cx*cz + sx*sy*sz;
	m_rm(1,2) = -sx*cz + cx*sy*sz;

	m_rm(2,0) = -sy;
	m_rm(2,1) =  sx*cy;
	m_rm(2,2) =  cx*cy;
	
      }

      /** \brief Returns a 3x3 eigen matrix */
      const Matrix3d & toMatrix() const 
      {
	return m_rm;
      }

      /** \brief Set the rotation matrix */
      void setRotationMatrix(const Matrix3d &rm)
      {
	m_rm = rm;
      }

      /** \brief Computes L for a general matrix A. 
       */
      void decompose(const Matrix3d &A, Matrix3_2d &L) const
      {
	L(0,0) = A(0,0) - A(2,2); L(0,1) = A(0,1);
	L(1,0) = A(1,0) ;         L(1,1) = A(1,1)-A(2,2);
	L(2,0) = A(2,0) + A(0,2); L(2,1) = A(2,1)+A(1,2);
      }

      /** \brief Computes L for a symmetric matrix A.
       */
      void decomposeltI(const class ltI &A,
			Matrix3_2d &L) const
      {
	L(0,0) = A.m_ltI(0) - A.m_ltI(5); 
	L(1,0) = L(0,1) = A.m_ltI(1);
	L(2,0) = 2*A.m_ltI(3);

	L(1,1) = A.m_ltI(2) - A.m_ltI(5); 
	L(2,1) = A.m_ltI(4) + A.m_ltI(4);
      }
      
      RotationMatrix transpose() const
      {
	return RotationMatrix(m_rm.transpose());
      }

      
      Matrix3d operator*(const Matrix3d &A) const
      {
	return static_cast<Matrix3d>(m_rm*A);
      }

      /** \brief Computes the multiplication between rotation matrix. 
       */
      RotationMatrix operator* (const RotationMatrix &arm) const
      {
	Matrix3d r;
	r.block<3,2>(0,0) = m_rm * arm.m_rm.block<3,2>(0,0);
	r(0,2) = -r(2,0)*r(1,1) + r(1,0)*r(2,1);
	r(1,2) =  r(2,0)*r(0,1) - r(0,0)*r(2,1);
	r(2,2) = -r(1,0)*r(0,1) + r(0,0)*r(1,1);
	return r;
      }

      /** \brief Computes \$f v = RM u \$f with $\f v,u \in \mathbb{R}^3 $\f 
       */
      Vector3d operator*(const Vector3d &A) const
      {
	return static_cast<Vector3d>(m_rm * A);
      }


      RotationMatrix operator*(FloatType a) const
      {
	return RotationMatrix(m_rm *a);
      }

      RotationMatrix operator-() const
      {
	return RotationMatrix(-m_rm);
      }

      FloatType operator()(int x, int y) const
      {
	return m_rm(x,y);
      }

      friend RotationMatrix operator*(FloatType a, RotationMatrix alti)
      {
	return RotationMatrix(alti.m_rm *a);
      }
      
      Vector3d col(int x) const
      {
	return m_rm.col(x);
      }
      /** \brief Compute the \$f EAE^{T}\$f 
       */
      Matrix3d rotGeneralMatrix(const Matrix3d &A) const
      {
	Vector3d r;
	Matrix3d Z;
	Matrix3_2d Y, L = Matrix3_2d::Zero();

	// 4a
	decompose(A,L);

	// Y = EL (18m + 12 a)
	Y = m_rm * L;

	// Z= YEt; (16 m + 8a)
	Z(0,1) = Y(0,0)*m_rm(1,0) + Y(0,1)*m_rm(1,1);
       	Z(0,2) = Y(0,0)*m_rm(2,0) + Y(0,1)*m_rm(2,1);
	
	Z(1,0) = Y(1,0)*m_rm(0,0) + Y(1,1)*m_rm(0,1);
	Z(1,1) = Y(1,0)*m_rm(1,0) + Y(1,1)*m_rm(1,1);
	Z(1,2) = Y(1,0)*m_rm(2,0) + Y(1,1)*m_rm(2,1);
	
	Z(2,0) = Y(2,0)*m_rm(0,0) + Y(2,1)*m_rm(0,1);
	Z(2,1) = Y(2,0)*m_rm(1,0) + Y(2,1)*m_rm(1,1);
	Z(2,2) = Y(2,0)*m_rm(2,0) + Y(2,1)*m_rm(2,1);

	// Z_11 (3a)
	Z(0,0) = L(0,0) + L(1,1) - Z(1,1) - Z(2,2);
	
	// r=Ev ( 6m + 3a)
	r(0) = -m_rm(0,0)*A(1,2) + m_rm(0,1)*A(0,2);
	r(1) = -m_rm(1,0)*A(1,2) + m_rm(1,1)*A(0,2);
	r(2) = -m_rm(2,0)*A(1,2) + m_rm(2,1)*A(0,2);

	// Z + D + (Ev)x ( 9a)
	Z(0,0) += A(2,2);  Z(0,1) +=  -r(2); Z(0,2) +=   r(1);
	Z(1,0) +=   r(2);  Z(1,1) += A(2,2); Z(1,2) +=  -r(0);
	Z(2,0) +=  -r(1);  Z(2,1) +=   r(0); Z(2,2) += A(2,2);
	
	return Matrix3d(Z);
      }
      

      /** \brief Compute the rotation for a symmetric matrix.
       */
      lowerTriangularMatrix rotTSymmetricMatrix(const lowerTriangularMatrix &A) const
      {

	Vector3d r;
	lowerTriangularMatrix Z;
	Matrix2d Y;
	Matrix3_2d L;

	// 4 a
	decomposeltI(A,L);
	
	// Y = EL (12 m + 8 a)
	Y = m_rm.transpose().block<2,3>(1,0) * L;

	// Z= YEt; (16 m + 8a)
	Z.m_ltI(1) = Y(0,0)*m_rm(0,0) + Y(0,1)*m_rm(1,0);
	Z.m_ltI(2) = Y(0,0)*m_rm(0,1) + Y(0,1)*m_rm(1,1);
	
	Z.m_ltI(3) = Y(1,0)*m_rm(0,0) + Y(1,1)*m_rm(1,0);
	Z.m_ltI(4) = Y(1,0)*m_rm(0,1) + Y(1,1)*m_rm(1,1);
	Z.m_ltI(5) = Y(1,0)*m_rm(0,2) + Y(1,1)*m_rm(1,2);

	// Z_11 (3a)
	Z.m_ltI(0) = L(0,0) + L(1,1) - Z.m_ltI(2) - Z.m_ltI(5);
	
	// r=Ev ( 6m + 3a)
	r(0) = -m_rm(0,0)*A.m_ltI(4) + m_rm(1,0)*A.m_ltI(3);
	r(1) = -m_rm(0,1)*A.m_ltI(4) + m_rm(1,1)*A.m_ltI(3);
	r(2) = -m_rm(0,2)*A.m_ltI(4) + m_rm(1,2)*A.m_ltI(3);

	// Z + D + (Ev)x ( 9a)
	Z.m_ltI(0) += A.m_ltI(5); 
	Z.m_ltI(1) += r(2); Z.m_ltI(2)+= A.m_ltI(5); 
	Z.m_ltI(3) +=-r(1); Z.m_ltI(4)+=       r(0); Z.m_ltI(5) += A.m_ltI(5);

	return lowerTriangularMatrix(Z);
      }

      lowerTriangularMatrix rotSymmetricMatrix(const lowerTriangularMatrix &A) const
      {

	Vector3d r;
	lowerTriangularMatrix Z;
	Matrix2d Y;
	Matrix3_2d L;

	// 4 a
	decomposeltI(A,L);
	
	// Y = EL (12 m + 8 a)
	Y = m_rm.block<2,3>(1,0) * L;

	// Z= YEt; (16 m + 8a)
	Z.m_ltI(1) = Y(0,0)*m_rm(0,0) + Y(0,1)*m_rm(0,1);
	Z.m_ltI(2) = Y(0,0)*m_rm(1,0) + Y(0,1)*m_rm(1,1);
	
	Z.m_ltI(3) = Y(1,0)*m_rm(0,0) + Y(1,1)*m_rm(0,1);
	Z.m_ltI(4) = Y(1,0)*m_rm(1,0) + Y(1,1)*m_rm(1,1);
	Z.m_ltI(5) = Y(1,0)*m_rm(2,0) + Y(1,1)*m_rm(2,1);

	// Z_11 (3a)
	Z.m_ltI(0) = L(0,0) + L(1,1) - Z.m_ltI(2) - Z.m_ltI(5);
	
	// r=Ev ( 6m + 3a)
	r(0) = -m_rm(0,0)*A.m_ltI(4) + m_rm(0,1)*A.m_ltI(3);
	r(1) = -m_rm(1,0)*A.m_ltI(4) + m_rm(1,1)*A.m_ltI(3);
	r(2) = -m_rm(2,0)*A.m_ltI(4) + m_rm(2,1)*A.m_ltI(3);

	// Z + D + (Ev)x ( 9a)
	Z.m_ltI(0) += A.m_ltI(5); 
	Z.m_ltI(1) += r(2); Z.m_ltI(2)+= A.m_ltI(5); 
	Z.m_ltI(3) +=-r(1); Z.m_ltI(4)+=       r(0); Z.m_ltI(5) += A.m_ltI(5);

	return lowerTriangularMatrix(Z);
      }

      friend std::ostream & operator<<(std::ostream &os,
				       const struct RotationMatrix & aRMAX)
      {
	os << aRMAX.m_rm << std::endl;
	return os;
      }

    };
  } // end Spatial namespace
} // end metapod namespace

#endif // METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_GENERAL_HH
