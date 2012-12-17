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
      Matrix3_2d m_L;
      Vector3d m_D;
      Vector3d m_v;
      
      rotationMatrix() 
      { 
	m_rm = Matrix3d::Zero();
	m_L = Matrix3_2d::Zero();
	m_D = Vector3d::Zero();
	m_v = Vector3d::Zero();
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
      const Matrix3d &toMatrix()
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
      void decompose(const Matrix3d &A)
      {
	m_L(0,0) = A(0,0) - A(2,2); m_L(0,1) = A(0,1);
	m_L(1,0) = A(1,0) ;         m_L(1,1) = A(1,1)-A(2,2);
	m_L(2,0) = A(2,0) + A(0,2); m_L(2,1) = A(2,1)+A(1,2);
      }

      /** \brief Computes L for a symmetric matrix A.
       */
      void decomposeltI(const struct ltI &A)
      {
	m_L(0,0) = A.m_ltI(0) - A.m_ltI(5); 
	m_L(1,0) = m_L(0,1) = A.m_ltI(1);
	m_L(2,0) = 2*A.m_ltI(3);

	m_L(1,1) = A.m_ltI(2) - A.m_ltI(5); 
	m_L(2,1) = A.m_ltI(4) + A.m_ltI(4);
      }
      
      /** \brief Compute \$f E arm.E \$f 
       */
      Matrix3d operator*(rotationMatrix &arm)
      {
	Matrix3d r;
	r.block<3,2>(0,0) = m_rm * arm.m_rm.block<3,2>(0,0);
	r(0,2) = -r(2,0)*r(1,1) + r(1,0)*r(2,1);
	r(1,2) =  r(2,0)*r(0,1) - r(0,0)*r(2,1);
	r(2,2) = -r(1,0)*r(0,1) + r(0,0)*r(1,1);
	return r;
      }
      
      /** \brief Compute the \$f EAE^{T}\$f 
       */
      Matrix3d & rotGeneralMatrix(const Matrix3d &A)
      {
	Vector3d r;
	Matrix3d Z;
	Matrix3_2d Y;

	// 4a
	decompose(A);

	// Y = EL (18m + 12 a)
	Y = m_rm * m_L;

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
	Z(0,0) = m_L(0,0) + m_L(1,1) - Z(1,1) - Z(2,2);
	
	// r=Ev ( 6m + 3a)
	r(0) = -m_rm(0,0)*A(1,2) + m_rm(0,1)*A(0,2);
	r(1) = -m_rm(1,0)*A(1,2) + m_rm(1,1)*A(0,2);
	r(2) = -m_rm(2,0)*A(1,2) + m_rm(2,1)*A(0,2);

	// Z + D + (Ev)x ( 9a)
	Z(0,0) += A(2,2);  Z(0,1) +=  -r(2); Z(0,2) +=   r(1);
	Z(1,0) +=   r(2);  Z(1,1) += A(2,2); Z(1,2) +=  -r(0);
	Z(2,0) +=  -r(1);  Z(2,1) +=   r(0); Z(2,2) += A(2,2);
	
	return Z;
      }
      

      /** \brief Compute the rotation for a symmetric matrix.
       */
      struct ltI & rotSymmetricMatrix(const struct ltI &A)
      {

	Vector3d r;
	struct ltI Z;
	Matrix2d Y;
	
	// 4 a
	decomposeltI(A);
	
	// Y = EL (12 m + 8 a)
	Y = m_rm.block<2,3>(1,0) * m_L;

	// Z= YEt; (16 m + 8a)
	Z.m_ltI(1) = Y(0,0)*m_rm(0,0) + Y(0,1)*m_rm(0,1);
	Z.m_ltI(2) = Y(0,0)*m_rm(1,0) + Y(0,1)*m_rm(1,1);
	
	Z.m_ltI(3) = Y(1,0)*m_rm(0,0) + Y(1,1)*m_rm(0,1);
	Z.m_ltI(4) = Y(1,0)*m_rm(1,0) + Y(1,1)*m_rm(1,1);
	Z.m_ltI(5) = Y(1,0)*m_rm(2,0) + Y(1,1)*m_rm(2,1);

	// Z_11 (3a)
	Z.m_ltI(0) = m_L(0,0) + m_L(1,1) - Z.m_ltI(2) - Z.m_ltI(5);
	
	// r=Ev ( 6m + 3a)
	r(0) = -m_rm(0,0)*A.m_ltI(4) + m_rm(0,1)*A.m_ltI(3);
	r(1) = -m_rm(1,0)*A.m_ltI(4) + m_rm(1,1)*A.m_ltI(3);
	r(2) = -m_rm(2,0)*A.m_ltI(4) + m_rm(2,1)*A.m_ltI(3);

	// Z + D + (Ev)x ( 9a)
	Z.m_ltI(0) += A.m_ltI(5); 
	Z.m_ltI(1) += r(2); Z.m_ltI(2)+= A.m_ltI(5); 
	Z.m_ltI(3) +=-r(1); Z.m_ltI(4)+=       r(0); Z.m_ltI(5) += A.m_ltI(5);

	return Z;
      }
    };

    struct rotationMatrixAboutXAxis
    {
      double c,s;

      rotationMatrixAboutXAxis(): 
	c(0.0),s(0.0) 
      {}

      void set(FloatType theta)
      { c = cos(theta); s=sin(theta);}

      Matrix3d toMatrix()
      {
	Matrix3d r;
	r(0,0) = 1.0; r(0,1) = 0.0; r(0,2) = 0.0;
	r(1,0) = 0.0; r(1,1) = c;   r(1,2) = s;
	r(2,0) = 0.0; r(2,1) = -s;  r(2,2) = c;
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

      const Matrix3d & rotGeneralMatrix(const Matrix3d &A)
      {
	Matrix3d r;

	FloatType alpha_x = c*s*(A(1,2)+A(2,1)) +
	  s*s*(A(2,2) - A(1,1));

	FloatType beta_x = c*s*(A(2,2)- A(1,1)) -
	  s*s*(A(1,2) + A(2,1));

	r(0,0) = A(0,0); 
	r(0,1) = c*A(0,1) + s*A(0,2);
	r(0,2) = c*A(0,2) - s*A(0,1);

	r(1,0) = c*A(1,0) + s*A(2,0);
	r(1,1) = A(1,1) + alpha_x;
	r(1,2) = A(1,2) + beta_x;

	r(2,0) = c*A(2,0) - s*A(1,0);
	r(2,1) = A(2,1) + beta_x;
	r(2,2) = A(2,2) - alpha_x;

	
	return r;
      }

      /** \brief Compute the rotation for a symmetric matrix.
       */
      struct ltI & rotSymmetricMatrix(const struct ltI &A)
      {
	struct ltI r;
	FloatType alpha_x = 2*c*s*A.m_ltI(4) +
	  s*s*(A.m_ltI(5) - A.m_ltI(2));
	FloatType beta_x  = c*s*(A.m_ltI(5) - A.m_ltI(2))+
	  (1-2*s*s)*A.m_ltI(4);

	r.m_ltI(0) = A.m_ltI(0);
	r.m_ltI(1) = c*A.m_ltI(1) + s*A.m_ltI(3); 
	r.m_ltI(2) = A.m_ltI(2) + alpha_x;
	r.m_ltI(3) = c*A.m_ltI(3) - s*A.m_ltI(1);
	r.m_ltI(4) = beta_x;
	r.m_ltI(5) = A.m_ltI(5) - alpha_x;
	return r;

      }      

      friend std::ostream & operator<<(std::ostream &os,
				       const struct rotationMatrixAboutXAxis & aRMAX)
      {
	os << "1.0 0.0 0.0" <<  endl;
	os << "0.0 " <<  aRMAX.c << " " << aRMAX.s << endl;
	os << "0.0 " << -aRMAX.s << " " <<  aRMAX.c << endl;
      }
    };

    struct rotationMatrixAboutYAxis
    {
      double c,s;
      
      rotationMatrixAboutYAxis(): c(0.0),s(0.0) {}
      
      void set(FloatType theta)
      { c = cos(theta); s=sin(theta);}


      Matrix3d toMatrix()
      {
	Matrix3d r;
	r(0,0) = c;   r(0,1) = 0.0; r(0,2) = -s;
	r(1,0) = 0.0; r(1,1) = 1;   r(1,2) =  0.0;
	r(2,0) = s;   r(2,1) = 0.0;  r(2,2) =  c;
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

      const Matrix3d & rotGeneralMatrix(const Matrix3d &A)
      {
	Matrix3d r;

	FloatType alpha_y =  c*s*(A(3,0)+A(0,2))
	  + s*s*(A(0,0) - A(2,2));

	FloatType beta_y = c*s*(A(0,0)- A(2,2)) -
	  s*s*(A(2,0) + A(0,2));

	r(0,0) = A(0,0) - alpha_y; 
	r(0,1) = c*A(0,1) - s*A(2,1);
	r(0,2) = A(0,2) + beta_y;

	r(1,0) = c*A(1,0) - s*A(1,2);
	r(1,1) = A(1,1);
	r(1,2) = c*A(1,2) + s*A(1,0);

	r(2,0) = A(2,0) + beta_y;
	r(2,1) = c*A(2,1) + s*A(0,1);
	r(2,2) = A(2,2) + alpha_y;

	
	return r;
      }

      /** \brief Compute the rotation for a symmetric matrix.
       */
      const struct ltI & rotSymmetricMatrix(const struct ltI &A)
      {
	struct ltI r;
	FloatType alpha_y =  2*c*s*A.m_ltI(3)
	  + s*s*(A.m_ltI(0) - A.m_ltI(5));
	FloatType beta_y = c*s*(A.m_ltI(0)- A.m_ltI(5)) +
	  (1-2*s*s)* A.m_ltI(3);

	r.m_ltI(0) = A.m_ltI(0) - alpha_y;
	r.m_ltI(1) = c*A.m_ltI(1) - s*A.m_ltI(4); 
	r.m_ltI(2) = A.m_ltI(2);
	r.m_ltI(3) = beta_y;
	r.m_ltI(4) = c*A.m_ltI(4) + s*A.m_ltI(1);
	r.m_ltI(5) = A.m_ltI(5) + alpha_y;
	return r;

      }      

      friend std::ostream & operator<<(std::ostream &os,
				       const struct rotationMatrixAboutYAxis & aRMAY)
      {
	os << aRMAY.c <<" 0.0 " << -aRMAY.s<<  endl;
	os << "0.0 1.0 0.0 " << endl;
	os << aRMAY.s << " 0.0 " <<  aRMAY.c << endl;
      }

    };

    struct rotationMatrixAboutZAxis
    {
      double c,s;
      
      rotationMatrixAboutZAxis(): c(0.0),s(0.0) {}
      
      void set(FloatType theta)
      { c = cos(theta); s=sin(theta);}

      /** \brief Random initialization */
      void randomInit()
      {
	boost::mt19937 rng;
	boost::uniform_real<> urd(-3.14, 3.14);

	FloatType theta_z=urd(rng);
	set(theta_z);
      }

      const Matrix3d & rotGeneralMatrix(const Matrix3d &A)
      {
	Matrix3d r;

	FloatType alpha_z = c*s*(A(0,1) + A(1,0)) +
	  s*s*(A(1,1)-A(0,0));

	FloatType beta_z = c*s*(A(1,1)- A(0,0)) -
	  s*s*(A(0,1) + A(1,0));

	r(0,0) = A(0,0) + alpha_z; 
	r(0,1) = A(0,1) + beta_z;
	r(0,2) = c*A(0,2) + s*A(1,2);

	r(1,0) = A(1,0) + beta_z;
	r(1,1) = A(1,1) - alpha_z;
	r(1,2) = c*A(1,2) - s*A(0,2);

	r(2,0) = c*A(2,0) + s*A(2,1);
	r(2,1) = c*A(2,1) - s*A(2,0);
	r(2,2) = A(2,2) ;

	
	return r;
      }

      /** \brief Compute the rotation for a symmetric matrix.
       */
      const struct ltI & rotSymmetricMatrix(const struct ltI &A)
      {
	struct ltI r;
	typedef struct ltI sltI;

	FloatType alpha_z =  2*c*s*A.m_ltI(1)
	  + s*s*(A.m_ltI(2) - A.m_ltI(0));
	FloatType beta_z = c*s*(A.m_ltI(2)- A.m_ltI(0)) +
	  (1-2*s*s)* A.m_ltI(1);

	r.m_ltI(0) = A.m_ltI(0) + alpha_z;
	r.m_ltI(1) = beta_z ; 
	r.m_ltI(2) = A.m_ltI(2) - alpha_z;
	r.m_ltI(3) = c*A.m_ltI(3) + s*A.m_ltI(4);
	r.m_ltI(4) = c*A.m_ltI(4) - s*A.m_ltI(3);
	r.m_ltI(5) = A.m_ltI(5);
	return r;

      }      

      Matrix3d toMatrix()
      {
	Matrix3d r;
	r(0,0) = c;   r(0,1) = s;   r(0,2) = 0.0;
	r(1,0) =-s;   r(1,1) = c;   r(1,2) = 0.0;
	r(2,0) = 0.0; r(2,1) = 0.0; r(2,2) = 1.0;
	return r;
      }

      friend std::ostream & operator<<(std::ostream &os,
				       const struct rotationMatrixAboutZAxis & aRMAZ)
      {
	os << aRMAZ.c << " " <<  aRMAZ.s << " 0.0" << endl;
	os << -aRMAZ.s << " " << aRMAZ.c << " 0.0" << endl;
	os << " 0.0 0.0 1.0 " << endl;
      }

    };
      
  }
}

#endif 
