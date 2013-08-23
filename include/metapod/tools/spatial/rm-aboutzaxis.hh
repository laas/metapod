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


#ifndef METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_ABOUTZ_HH
# define METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_ABOUTZ_HH

namespace metapod
{

  namespace Spatial
  {

    /** \object RotationMatrixAboutZ
	This object implements specific operations 
	related to the rotation matrix	about the Z-axis: <br>
	\f[ 
	  rz(\theta) = 
	     \left[ 
	       \begin{matrix} 
	         c & s & 0 \\ 
		 -s & c & 0 \\ 
		 0 & 0 & 1 
	       \end{matrix} 
	     \right] 
	 \f]
    */
    struct RotationMatrixAboutZ
    {
      /// Store directly \f$ cos(\theta) \f$ and \f$ sin(\theta) \f$
      FloatType m_c,m_s;

      RotationMatrixAboutZ(): 
	m_c(0.0),m_s(0.0) 
      {}

      RotationMatrixAboutZ(const Matrix3d &aR)
      {
	m_c=aR(0,0);m_s=aR(0,1);
      }

      RotationMatrixAboutZ(FloatType c, FloatType s)
      {
	m_c=c;m_s=s;
      }

      /** \brief Random initialization */
      void randomInit()
      {
	boost::mt19937 rng;
	boost::uniform_real<> urd(-3.14, 3.14);

	FloatType theta_x=urd(rng);
	set(theta_x);
      }

      RotationMatrixAboutZ transpose() const
      {
	return RotationMatrixAboutZ(m_c,-m_s);
      }

      RotationMatrixAboutZ operator*(FloatType a) const
      {
	return RotationMatrixAboutZ(a*m_c,a*m_s);
      }

      RotationMatrixAboutZ operator-() const
      {
	return RotationMatrixAboutZ(-m_c,-m_s);
      }

      void set(FloatType theta)
      { m_c = cos(theta); m_s=sin(theta);}

      Matrix3d toMatrix() const
      {
	Matrix3d r;
	r(0,0) = m_c; r(0,1) = m_s; r(0,2) = 0.0;
	r(1,0) =-m_s; r(1,1) = m_c; r(1,2) = 0.0;
	r(2,0) = 0.0; r(2,1) = 0.0; r(2,2) = 1.0;
	return r;
      }
      /** \brief Optimized multiplication of the rotation matrix with a general 3x3 matrix.
       The total number of operations is 12m + 6a. <br>
       Matrix3d = RotationMatrixAboutZ * Matrix3d <br>
       \f$ {\bf B} = rz(\theta) {\bf A} \f$
       \f[ {\bf B} = 
         \left[ 
            \begin{matrix} 
               A(0,0)c + A(1,0)s &  
	       A(0,1)c + A(1,1)s &  
	       A(1,2)c + A(1,2)s \\
              -A(0,0)s + A(1,0)c & 
	      -A(0,1)s + A(1,1)c & 
	      -A(0,2)s + A(1,2)c \	\
               A(2,0)            &  A(2,1)            &  A(2,2) \\
            \end{matrix}
         \right] 
       \f]
      */
      Matrix3d operator*(const Matrix3d &A) const
      {
	Matrix3d r = A;
	r.block<1,3>(2,0) = A.block<1,3>(2,0);

	for(unsigned int i=0;i<3;i++)
	  r(0,i) = A(0,i) * m_c + A(1,i) * m_s;
	
	for(unsigned int i=0;i<3;i++)
	  r(1,i) =-A(0,i) * m_s + A(1,i) * m_c;
	
	return r;
      }

      /** \brief Optimized multiplication of the rotation matrix with a general 3x3 matrix.
       The total number of operations is 12m + 6a. <br>
       RotationMatrix = RotationMatrixAboutZ * RotationMatrix <br>
       \f$ {\bf B} = rz(\theta) {\bf A} \f$
       \f[ {\bf B} = 
         \left[ 
            \begin{matrix} 
               A(0,0)c + A(1,0)s & A(0,1)c + A(1,1)s & A(1,2)c + A(1,2)s \\
              -A(0,0)s + A(1,0)c &-A(0,1)s + A(1,1)c &-A(0,2)s + A(1,2)c \\               A(2,0)            &  A(2,1)            &  A(2,2) \\
            \end{matrix}
         \right] 
       \f]
      */
      RotationMatrix operator*(const RotationMatrix &aRM) const
      {
	Matrix3d r;
	r = Matrix3d::Zero();
	const Matrix3d & lrm = aRM.m_rm;
	
	r.block<1,3>(2,0) = lrm.block<1,3>(2,0);

	for(unsigned int i=0;i<3;i++)
	  r(0,i) = lrm(0,i) * m_c + lrm(1,i) * m_s;
	
	for(unsigned int i=0;i<3;i++)
	  r(1,i) =-lrm(0,i) * m_s + lrm(1,i) * m_c;

	return RotationMatrix(r);
      }
      
      
      RotationMatrixAboutZ operator*(const RotationMatrixAboutZ &aRM) const
      {
	FloatType lc,ls;
	lc = m_c * aRM.m_c - m_s * aRM.m_s;
	ls = m_c * aRM.m_s + m_s * aRM.m_c;
	return RotationMatrixAboutZ(lc,ls);

      }

      Vector3d operator*(const Vector3d &aRM) const
      {
	Vector3d r;
	r(0) = m_c*aRM(0) + m_s*aRM(1);
	r(1) =-m_s*aRM(0) + m_c*aRM(1);
	r(2) = aRM(2);
	return r;
      }

      /** \brief Optimized computation of \f$ R {\bf A} R^{\top} \f$ where \f$ {\bf A} \f$ is a generalized 3x3 matrix.
       The total number of operations is 12m + 12a.

       \f$ \alpha_z = cs (A_{01} + A_{10}) + s^2(A_{11} - A_{00}) \f$
       \f$ \beta_z  = cs (A_{11} - A_{00}) - s^2(A_{01} + A_{10}) \f$ 
       \f[ 
          rz(\theta) {\bf A} rz(\theta)^{\top} = 
            \left[ 
               \begin{matrix}
                  A_{00} + \beta_z  &  A_{01} + \beta_z  & cA_{02} + sA_{12}  \\
                  A_{10} + \beta_z  &  A_{11} - \alpha_z & A_{12} - sA_{02} \\
                  cA_{20} + sA_{21}  &  cA_{21} -sA_{20} & A_{22}\\
               \end{matrix}
            \right]
        \f]
      */
      Matrix3d  rotGeneralMatrix(const Matrix3d &A) const
      {
	Matrix3d r;

	FloatType alpha_z = m_c*m_s*(A(0,1) + A(1,0)) +
	  m_s*m_s*(A(1,1) - A(0,0));

	FloatType beta_z  = m_c*m_s*(A(1,1) - A(0,0)) -
	  m_s*m_s*(A(0,1) + A(1,0));

	r(0,0) = A(0,0) + alpha_z; 
	r(0,1) = A(0,1) + beta_z;
	r(0,2) = m_c*A(0,2) + m_s*A(1,2);

	r(1,0) = A(1,0) + beta_z;
	r(1,1) = A(1,1) - alpha_z;
	r(1,2) = m_c*A(1,2) - m_s*A(0,2);

	r(2,0) = m_c*A(2,0) + m_s*A(2,1);
	r(2,1) = m_c*A(2,1) - m_s*A(2,0);
	r(2,2) = A(2,2);
	return r;
      }

      /** \brief Optimized computation of 
	  \f$ rz(\theta) {\bf A} rz(\theta)^{\top} \f$ 
	  where \f$ {\bf A} \f$ is a 3x3 symmetric matrix.
	  \f$ \alpha_z = 2csA_{10} + s^2 (A_{11} - A_{00})\f$
	  \f$ \beta_z = cs(A_{11} - A_{00}) + (1-2s^2)A_{10} \f$
	  \f[
	    lt(rz(\theta){\bf A}rz(\theta)^{\top}) =
	       \left[
	         \begin{matrix}
		    A_{00} + \alpha_z & \cdotp & \cdotp \\
		    \beta_z & A_{11} - \alpha_z & \cdotp \\
		    cA_{20} + sA_{21} & cA_{21} -sA_{20} & A_{22} \\
		 \end{matrix}
	       \right]
	  \f]
       */
      class ltI rotSymmetricMatrix(const class ltI &A) const
      {
	class ltI r;
	FloatType alpha_z = 2*m_c*m_s*A.m_ltI(1) +
	  m_s*m_s*(A.m_ltI(2) - A.m_ltI(0));
	FloatType beta_z  = m_c*m_s*(A.m_ltI(2) - A.m_ltI(0))+
	  (1-2*m_s*m_s)*A.m_ltI(1);

	r.m_ltI(0) = A.m_ltI(0) + alpha_z;
	r.m_ltI(1) = beta_z; 
	r.m_ltI(2) = A.m_ltI(2) - alpha_z;
	r.m_ltI(3) = m_c*A.m_ltI(3) + m_s*A.m_ltI(4);
	r.m_ltI(4) = m_c*A.m_ltI(4) - m_s*A.m_ltI(3);
	r.m_ltI(5) = A.m_ltI(5);
	return r;
      }      

      /** \brief Optimized computation of 
	  \f$ rz(\theta)^{\top} {\bf A} rz(\theta) \f$ 
	  where \f$ {\bf A} \f$ is a 3x3 symmetric matrix.
	  \f$ \alpha_z = 2csA_{10} + s^2 (A_{11} - A_{00})\f$
	  \f$ \beta_z = cs(A_{11} - A_{00}) + (1-2s^2)A_{10} \f$
	  \f[
	    lt(rz(\theta){\bf A}rz(\theta)^{\top}) =
	       \left[
	         \begin{matrix}
		    A_{00} + \alpha_z & \cdotp & \cdotp \\
		    \beta_z & A_{11} - \alpha_z & \cdotp \\
		    cA_{20} + sA_{21} & cA_{21} -sA_{20} & A_{22} \\
		 \end{matrix}
	       \right]
	  \f]
       */
      class ltI rotTSymmetricMatrix(const class ltI &A) const
      {
	class ltI r;
	FloatType alpha_z = 2*m_c*m_s*A.m_ltI(1) +
	  m_s*m_s*(A.m_ltI(0) - A.m_ltI(2));
	FloatType beta_z  = m_c*m_s*(A.m_ltI(0) - A.m_ltI(2))+
	  (1-2*m_s*m_s)*A.m_ltI(1);

	r.m_ltI(0) = A.m_ltI(0) - alpha_z;
	r.m_ltI(1) = beta_z; 
	r.m_ltI(2) = A.m_ltI(2) + alpha_z;
	r.m_ltI(3) = m_c*A.m_ltI(3) - m_s*A.m_ltI(4);
	r.m_ltI(4) = m_c*A.m_ltI(4) + m_s*A.m_ltI(3);
	r.m_ltI(5) = A.m_ltI(5);
	return r;
      }      


      friend std::ostream & operator<<(std::ostream &os,
				       const struct RotationMatrixAboutZ & aRMAX)
      {
	os << aRMAX.m_c << " " << aRMAX.m_s << " 0.0" << std::endl;
	os <<-aRMAX.m_s << " " << aRMAX.m_c << " 0.0" << std::endl;
	os << " 0.0 0.0 1.0 "<< std::endl;
	return os;
      }
      
      
      
    };
  
  } // end Spatial namespace
} // end metapod namespace

#endif // METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_ABOUTY_HH
