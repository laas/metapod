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


#ifndef METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_ABOUTX_HH
# define METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_ABOUTX_HH

namespace metapod
{

  namespace Spatial
  {

    /// \object RotationMatrixAboutX
    /// This object implements specific operations related to the rotation matrix
    /// about the X-axis:
    /// \f[ rx(\theta) = \left[ \begin{matrix} 1 & 0 & 0 \\ 0 & c & s \\ 0 & -s & c \end{matrix} \right] \f]
    /// 
    struct RotationMatrixAboutX
    {
      /// Store directly \f$ cos(\theta) \f$ and \f$ sin(\theta) \f$
      FloatType m_c,m_s;

      RotationMatrixAboutX(): 
	m_c(0.0),m_s(0.0) 
      {}

      RotationMatrixAboutX(const Matrix3d &aR)
      {
	m_c=aR(1,1);m_s=aR(1,2);
      }

      RotationMatrixAboutX(FloatType c, FloatType s)
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

      RotationMatrixAboutX transpose() const
      {
	return RotationMatrixAboutX(m_c,-m_s);
      }

      Matrix3d toMatrix() const
      {
	Matrix3d r;
	r(0,0) = 1.0; r(0,1) = 0.0; r(0,2) = 0.0;
	r(1,0) = 0.0; r(1,1) = m_c;   r(1,2) = m_s;
	r(2,0) = 0.0; r(2,1) = -m_s;  r(2,2) = m_c;
	return r;
      }

      Matrix3d operator*(FloatType a) const
      {
	return Matrix3d (a*toMatrix());
      }

      RotationMatrixAboutX operator-() const
      {
	return RotationMatrixAboutX(-m_c,-m_s);
      }

      void set(FloatType theta)
      { m_c = cos(theta); m_s=sin(theta);}

      
      /** \brief Optimized multiplication of the rotation matrix with a general 3x3 matrix.
       The total number of operations is 12m + 6a. <br>
       Matrix3d = RotationMatrixAboutX * Matrix3d <br>
       \f$ {\bf B} = rx(\theta) {\bf A} \f$
       \f[ {\bf B} = 
         \left[ 
            \begin{matrix} 
               A(0,0)            &  A(0,1)            &  A(0,2) \\
               A(1,0)c_{\theta} + A(2,0)s_{\theta} &  
	       A(1,1)c_{\theta} + A(2,1)s_{\theta} &  
	       A(1,2)c_{\theta} + A(2,2)s_{\theta} \\
              -A(1,0)s_{\theta} + A(2,0)c_{\theta} & 
	      -A(1,1)s_{\theta} + A(2,1)c_{\theta} & 
	      -A(1,2)s_{\theta} + A(2,2)c_{\theta} \\
            \end{matrix}
         \right] 
       \f]
      */
      Matrix3d operator*(const Matrix3d &A) const
      {
	Matrix3d r;
	r.block<1,3>(0,0) = A.block<1,3>(0,0);

	for(unsigned int i=0;i<3;i++)
	  r(1,i) =  A(1,i) * m_c + A(2,i) * m_s;
	
	for(unsigned int i=0;i<3;i++)
	  r(2,i) = -A(1,i) * m_s + A(2,i) * m_c;
	
	return r;
      }

      /** \brief Optimized multiplication of the rotation matrix with a general 3x3 rotation matrix.
	  The total number of operations is 12m + 6a. <br>
	  RotationMatrix = RotationMatrixAboutX * RotationMatrix <br>
	  \f$ {\bf B} = rx(\theta) {\bf A} \f$
	  
	  \f[ {\bf B} = 
	    \left[ 
	      \begin{matrix} 
	         A(0,0)            &  A(0,1)            &  A(0,2) \\
                 A(1,0)c + A(2,0)s &  A(1,1)c + A(2,1)s &  A(1,2)c + A(2,2)s \\
                -A(1,0)s + A(2,0)c & -A(1,1)s + A(2,1)c & -A(1,2)s + A(2,2)c \\
              \end{matrix}
            \right] 
         \f]

      */
      RotationMatrix operator*(const RotationMatrix &aRM) const
      {
	Matrix3d r;
	const Matrix3d &lrm = aRM.m_rm;

	r.block<1,3>(0,0) = lrm.block<1,3>(0,0);

	for(unsigned int i=0;i<3;i++)
	  r(1,i) = lrm(1,i) * m_c + lrm(2,i) * m_s;
	
	for(unsigned int i=0;i<3;i++)
	  r(2,i) = -lrm(1,i) * m_s + lrm(2,i) * m_c;

	return RotationMatrix(r);
      }

      /** \brief Optimized multiplication of the rotation matrix with a rotation matrix about the X axis.
       The total number of operations is 4m + 2a. <br>
       RotationMatrixAboutX = RotationMatrixAboutX * RotationMatrixAboutX <br>
       \f$ rx(\theta_C) = rx(\theta_A)rx(\theta_B)\f$
       \f[
         rx(\theta_C) = 
	    \left[ 
	      \begin{matrix} 
	         1.0 &  0.0                                     &  0.0 \\
                 0.0 &  c_{\theta_A}c_{\theta_B} - s_{\theta_A}s_{\theta_B} & c_{\theta_A}s_{\theta_B} + s_{\theta_A}c_{\theta_B} \\
                 0.0 & -c_{\theta_A}s_{\theta_B} - s_{\theta_A}c_{\theta_B} & c_{\theta_A}c_{\theta_B} - s_{\theta_A}s_{\theta_B} \\
              \end{matrix}
            \right] 
       \f]
      */
      RotationMatrixAboutX operator*(const RotationMatrixAboutX &aRM) const
      {
	FloatType lc,ls;
	lc = m_c * aRM.m_c - m_s * aRM.m_s;
	ls = m_c * aRM.m_s + m_s * aRM.m_c;
	return RotationMatrixAboutX(lc,ls);
      }

      Vector3d operator*(const Vector3d &aRM) const
      {
	Vector3d r;
	r(0) = aRM(0);
	r(1) =  m_c * aRM(1) + m_s * aRM(2);
	r(2) = -m_s * aRM(1) + m_c * aRM(2);
	return r;
      }

      /** \brief Optimized computation of 
	  \f$ rx(\theta) {\bf A} rx(\theta)^{\top} \f$ 
	  where \f$ {\bf A} \f$ is a generalized 3x3 matrix.
       The total number of operations is 12m + 12a.

       \f$ \alpha_x = cs (A_{12} + A_{21}) + s^2(A_{22} - A_{11}) \f$
       \f$ \beta_x = cs (A_{22} - A_{11}) - s^2(A_{12} + A_{21}) \f$ 
       \f[ 
          rx(\theta) {\bf A} rx(\theta)^{\top} = 
            \left[ 
               \begin{matrix}
                  A_{00}             &  cA_{01} + sA_{02} & cA_{02} - sA_{01}  \\
                  cA_{10} + sA_{20}  &  A_{11} + \alpha_x & A_{12} + \beta_x \\
                  cA_{20} - sA_{10}  &  A_{21} + \beta_x  & A_{22} - \alpha_x
               \end{matrix}
            \right]
        \f]
      */
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

      /** \brief Optimized computation of 
	  \f$ rx(\theta) {\bf A} rx(\theta)^{\top} \f$ 
	  where \f$ {\bf A} \f$ is a 3x3 symmetric matrix.
	  \f$ \alpha_x = 2csA_{21} + s^2 (A_{22} - A_{11})\f$
	  \f$ \beta_x = cs(A_{22} - A_{11}) + (1-2s^2)A_{21} \f$
	  \f[
	    lt(rx(\theta){\bf A}rx(\theta)^{\top}) =
	       \left[
	         \begin{matrix}
		    A_{00} & \cdotp & \cdotp \\
		    cA_{10} + sA_{20} & A_{11} + \alpha_x & \cdotp \\
		    cA_{20} - sA_{10} & \beta_x & A_{22} - \alpha_x 
		 \end{matrix}
	       \right]
	  \f]
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

      friend std::ostream & operator<<(std::ostream &os,
				       const struct RotationMatrixAboutX & aRMAX)
      {
	os << "1.0 0.0 0.0" << std::endl;
	os << "0.0 " <<  aRMAX.m_c << " " << aRMAX.m_s << std::endl;
	os << "0.0 " << -aRMAX.m_s << " " << aRMAX.m_c << std::endl;
	return os;
      }

    };
  
  } // end Spatial namespace
} // end metapod namespace

#endif // METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_ABOUTX_HH
