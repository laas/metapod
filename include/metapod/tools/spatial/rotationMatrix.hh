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


#ifndef METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_HH
# define METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_HH

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/generator_iterator.hpp>


# include "metapod/tools/fwd.hh"
# include "metapod/tools/spatial/lti.hh"


namespace metapod
{

  namespace Spatial
  {

    struct rotationMatrix
    {
      Matrix3d m_rm;
      
      rotationMatrix() 
      { 
	m_rm = Matrix3d::Zero();
      }
      
      rotationMatrix(const Matrix3d &aR)
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

      /** \brief Returns a matrix */
      const Matrix3d & toMatrix()
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
      void decomposeltI(const struct ltI &A,
			Matrix3_2d &L) const
      {
	L(0,0) = A.m_ltI(0) - A.m_ltI(5); 
	L(1,0) = L(0,1) = A.m_ltI(1);
	L(2,0) = 2*A.m_ltI(3);

	L(1,1) = A.m_ltI(2) - A.m_ltI(5); 
	L(2,1) = A.m_ltI(4) + A.m_ltI(4);
      }
      
      rotationMatrix transpose() const
      {
	return rotationMatrix(m_rm.transpose());
      }

      
      Matrix3d operator*(const Matrix3d &A) const
      {
	return static_cast<Matrix3d>(m_rm*A);
      }

      /** \brief Computes the multiplication between rotation matrix. 
       */
      rotationMatrix operator* (const rotationMatrix &arm) const
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


      rotationMatrix operator*(FloatType a) const
      {
	return rotationMatrix(m_rm *a);
      }

      rotationMatrix operator-() const
      {
	return rotationMatrix(-m_rm);
      }

      FloatType operator()(int x, int y) const
      {
	return m_rm(x,y);
      }

      friend rotationMatrix operator*(FloatType a, rotationMatrix alti)
      {
	return rotationMatrix(alti.m_rm *a);

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
				       const struct rotationMatrix & aRMAX)
      {
	os << aRMAX.m_rm << std::endl;
	return os;
      }

    };

    struct rotationMatrixAboutXAxis
    {
      double m_c,m_s;

      rotationMatrixAboutXAxis(): 
	m_c(0.0),m_s(0.0) 
      {}

      rotationMatrixAboutXAxis(const Matrix3d &aR)
      {
	m_c=aR(1,1);m_s=aR(1,2);
      }

      rotationMatrixAboutXAxis(double c, double s)
      {
	m_c=c;m_s=s;
      }


      rotationMatrixAboutXAxis operator*(FloatType a) const
      {
	return rotationMatrixAboutXAxis(a*m_c,a*m_s);
      }

      rotationMatrixAboutXAxis operator-() const
      {
	return rotationMatrixAboutXAxis(-m_c,m_s);
      }

      void set(FloatType theta)
      { m_c = cos(theta); m_s=sin(theta);}

      Matrix3d toMatrix()
      {
	Matrix3d r;
	r(0,0) = 1.0; r(0,1) = 0.0; r(0,2) = 0.0;
	r(1,0) = 0.0; r(1,1) = m_c;   r(1,2) = m_s;
	r(2,0) = 0.0; r(2,1) = -m_s;  r(2,2) = m_c;
	return r;
      }

      /** \brief Random initialization */
      void randomInit()
      {
	boost::mt19937 rng;
	boost::uniform_real<> urd(-3.14, 3.14);

	FloatType theta_x=urd(rng);
	set(theta_x);
      }

      Matrix3d  rotGeneralMatrix(const Matrix3d &A) const
      {
	Matrix3d r;

	FloatType alpha_x = m_c*m_s*(A(1,2)+A(2,1)) +
	  m_s*m_s*(A(2,2) - A(1,1));

	FloatType beta_x = m_c*m_s*(A(2,2)- A(1,1)) -
	  m_s*m_s*(A(1,2) + A(2,1));

	r(0,0) = A(0,0); 
	r(0,1) = m_c*A(0,1) + m_s*A(0,2);
	r(0,2) = m_c*A(0,2) - m_s*A(0,1);

	r(1,0) = m_c*A(1,0) + m_s*A(2,0);
	r(1,1) = A(1,1) + alpha_x;
	r(1,2) = A(1,2) + beta_x;

	r(2,0) = m_c*A(2,0) - m_s*A(1,0);
	r(2,1) = A(2,1) + beta_x;
	r(2,2) = A(2,2) - alpha_x;
	return r;
      }

      /** \brief Compute the rotation for a symmetric matrix.
       */
      struct ltI rotSymmetricMatrix(const struct ltI &A)
      {
	struct ltI r;
	FloatType alpha_x = 2*m_c*m_s*A.m_ltI(4) +
	  m_s*m_s*(A.m_ltI(5) - A.m_ltI(2));
	FloatType beta_x  = m_c*m_s*(A.m_ltI(5) - A.m_ltI(2))+
	  (1-2*m_s*m_s)*A.m_ltI(4);

	r.m_ltI(0) = A.m_ltI(0);
	r.m_ltI(1) = m_c*A.m_ltI(1) + m_s*A.m_ltI(3); 
	r.m_ltI(2) = A.m_ltI(2) + alpha_x;
	r.m_ltI(3) = m_c*A.m_ltI(3) - m_s*A.m_ltI(1);
	r.m_ltI(4) = beta_x;
	r.m_ltI(5) = A.m_ltI(5) - alpha_x;
	return r;

      }      

      Matrix3d operator*(const Matrix3d &A) const
      {
	return rotGeneralMatrix(A);
      }

      rotationMatrix operator*(const rotationMatrix &aRM) const
      {
	return rotationMatrix(rotGeneralMatrix(aRM.m_rm));
      }

      friend std::ostream & operator<<(std::ostream &os,
				       const struct rotationMatrixAboutXAxis & aRMAX)
      {
	os << "1.0 0.0 0.0" <<  endl;
	os << "0.0 " <<  aRMAX.m_c << " " << aRMAX.m_s << endl;
	os << "0.0 " << -aRMAX.m_s << " " << aRMAX.m_c << endl;
	return os;
      }
      
      
      
    };

    struct rotationMatrixAboutYAxis
    {
      double m_c,m_s;
      
      rotationMatrixAboutYAxis(): m_c(0.0),m_s(0.0) {}
      
      void set(FloatType theta)
      { m_c = cos(theta); m_s=sin(theta);}


      Matrix3d toMatrix()
      {
	Matrix3d r;
	r(0,0) = m_c;   r(0,1) = 0.0; r(0,2) = -m_s;
	r(1,0) = 0.0; r(1,1) = 1;   r(1,2) =  0.0;
	r(2,0) = m_s;   r(2,1) = 0.0;  r(2,2) =  m_c;
	return r;
      }

      /** \brief Random initialization */
      void randomInit()
      {
	boost::mt19937 rng;
	boost::uniform_real<> urd(-3.14, 3.14);

	FloatType theta_y=urd(rng);
	set(theta_y);
      }

      Matrix3d rotGeneralMatrix(const Matrix3d &A)
      {
	Matrix3d r;

	FloatType alpha_y =  m_c*m_s*(A(3,0)+A(0,2))
	  + m_s*m_s*(A(0,0) - A(2,2));

	FloatType beta_y = m_c*m_s*(A(0,0)- A(2,2)) -
	  m_s*m_s*(A(2,0) + A(0,2));

	r(0,0) = A(0,0) - alpha_y; 
	r(0,1) = m_c*A(0,1) - m_s*A(2,1);
	r(0,2) = A(0,2) + beta_y;

	r(1,0) = m_c*A(1,0) - m_s*A(1,2);
	r(1,1) = A(1,1);
	r(1,2) = m_c*A(1,2) + m_s*A(1,0);

	r(2,0) = A(2,0) + beta_y;
	r(2,1) = m_c*A(2,1) + m_s*A(0,1);
	r(2,2) = A(2,2) + alpha_y;

	
	return r;
      }


      /** \brief Compute the rotation for a symmetric matrix.
       */
      struct ltI rotSymmetricMatrix(const struct ltI &A)
      {
	struct ltI r;
	FloatType alpha_y =  2*m_c*m_s*A.m_ltI(3)
	  + m_s*m_s*(A.m_ltI(0) - A.m_ltI(5));
	FloatType beta_y = m_c*m_s*(A.m_ltI(0)- A.m_ltI(5)) +
	  (1-2*m_s*m_s)* A.m_ltI(3);

	r.m_ltI(0) = A.m_ltI(0) - alpha_y;
	r.m_ltI(1) = m_c*A.m_ltI(1) - m_s*A.m_ltI(4); 
	r.m_ltI(2) = A.m_ltI(2);
	r.m_ltI(3) = beta_y;
	r.m_ltI(4) = m_c*A.m_ltI(4) + m_s*A.m_ltI(1);
	r.m_ltI(5) = A.m_ltI(5) + alpha_y;
	return r;

      }      

      friend std::ostream & operator<<(std::ostream &os,
				       const struct rotationMatrixAboutYAxis & aRMAY)
      {
	os << aRMAY.m_c <<" 0.0 " << -aRMAY.m_s<<  endl;
	os << "0.0 1.0 0.0 " << endl;
	os << aRMAY.m_s << " 0.0 " <<  aRMAY.m_c << endl;
	return os;
      }

    };

    struct rotationMatrixAboutZAxis
    {
      double m_c,m_s;
      
      rotationMatrixAboutZAxis(): m_c(0.0),m_s(0.0) {}
      
      void set(FloatType theta)
      { m_c = cos(theta); m_s=sin(theta);}

      /** \brief Random initialization */
      void randomInit()
      {
	boost::mt19937 rng;
	boost::uniform_real<> urd(-3.14, 3.14);

	FloatType theta_z=urd(rng);
	set(theta_z);
      }

      Matrix3d rotGeneralMatrix(const Matrix3d &A)
      {
	Matrix3d r;

	FloatType alpha_z = m_c*m_s*(A(0,1) + A(1,0)) +
	  m_s*m_s*(A(1,1)-A(0,0));

	FloatType beta_z = m_c*m_s*(A(1,1)- A(0,0)) -
	  m_s*m_s*(A(0,1) + A(1,0));

	r(0,0) = A(0,0) + alpha_z; 
	r(0,1) = A(0,1) + beta_z;
	r(0,2) = m_c*A(0,2) + m_s*A(1,2);

	r(1,0) = A(1,0) + beta_z;
	r(1,1) = A(1,1) - alpha_z;
	r(1,2) = m_c*A(1,2) - m_s*A(0,2);

	r(2,0) = m_c*A(2,0) + m_s*A(2,1);
	r(2,1) = m_c*A(2,1) - m_s*A(2,0);
	r(2,2) = A(2,2) ;

	return r;
      }

      /** \brief Compute the rotation for a symmetric matrix.
       */
      struct ltI rotSymmetricMatrix(const struct ltI &A)
      {
	struct ltI r;
	typedef struct ltI sltI;

	FloatType alpha_z =  2*m_c*m_s*A.m_ltI(1)
	  + m_s*m_s*(A.m_ltI(2) - A.m_ltI(0));
	FloatType beta_z = m_c*m_s*(A.m_ltI(2)- A.m_ltI(0)) +
	  (1-2*m_s*m_s)* A.m_ltI(1);

	r.m_ltI(0) = A.m_ltI(0) + alpha_z;
	r.m_ltI(1) = beta_z ; 
	r.m_ltI(2) = A.m_ltI(2) - alpha_z;
	r.m_ltI(3) = m_c*A.m_ltI(3) + m_s*A.m_ltI(4);
	r.m_ltI(4) = m_c*A.m_ltI(4) - m_s*A.m_ltI(3);
	r.m_ltI(5) = A.m_ltI(5);
	return r;
      }      

      Matrix3d toMatrix()
      {
	Matrix3d r;
	r(0,0) = m_c;   r(0,1) = m_s;   r(0,2) = 0.0;
	r(1,0) =-m_s;   r(1,1) = m_c;   r(1,2) = 0.0;
	r(2,0) = 0.0; r(2,1) = 0.0; r(2,2) = 1.0;
	return r;
      }

      friend std::ostream & operator<<(std::ostream &os,
				       const struct rotationMatrixAboutZAxis & aRMAZ)
      {
	os << aRMAZ.m_c << " " <<  aRMAZ.m_s << " 0.0" << endl;
	os << -aRMAZ.m_s << " " << aRMAZ.m_c << " 0.0" << endl;
	os << " 0.0 0.0 1.0 " << endl;
	return os;
      }

    };
      
  }
}

#endif 
