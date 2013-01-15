// Copyright 2013,
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


#ifndef METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_CHGAXIS_HH
# define METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_CHGAXIS_HH

namespace metapod
{

  
  namespace Spatial
  {
    
    template <int IX, int IY, int IZ, int LX, int LY, int LZ>
    struct rmca_traits
    {
      static int const ind_X = IX; 
      static int const value_X = LX;
      static int const ind_Y = IY; 
      static int const value_Y = LY;
      static int const ind_Z = IZ; 
      static int const value_Z = LZ;
    };

    namespace Internal
    {
      template < typename traits, int I>
      struct rmca_traits_ref;
      
      template < typename traits>
      struct rmca_traits_ref<traits, 0>
      { static int const ind = traits::ind_X; 
        static int const value = traits::value_X;
      };
      
      template < typename traits>
      struct rmca_traits_ref<traits, 1>
      { static int const ind = traits::ind_Y; 
        static int const value = traits::value_Y;
      };
      
      template < typename traits>
      struct rmca_traits_ref< traits,  2>
      { static int const ind = traits::ind_Z; 
        static int const value = traits::value_Z;
      };

    }

    template < typename traits_op1,
               typename traits_op2>
    struct rmca_traits_mul
    {
      typedef rmca_traits<Internal::rmca_traits_ref<traits_op1,traits_op2::ind_X>::ind,
          Internal::rmca_traits_ref<traits_op1,traits_op2::ind_Y>::ind,
          Internal::rmca_traits_ref<traits_op1,traits_op2::ind_Z>::ind,
          traits_op2::value_X*Internal::rmca_traits_ref<traits_op1,traits_op2::ind_X>::value,
          traits_op2::value_Y*Internal::rmca_traits_ref<traits_op1,traits_op2::ind_Y>::value,
          traits_op2::value_Z*Internal::rmca_traits_ref<traits_op1,traits_op2::ind_Z>::value >
          ltraits;
      static int const ind_X = ltraits::ind_X;
      static int const value_X = ltraits::value_X;
      static int const ind_Y = ltraits::ind_Y; 
      static int const value_Y = ltraits::value_Y;
      static int const ind_Z = ltraits::ind_Z; 
      static int const value_Z = ltraits::value_Z;
    };

    template < typename traits_op1>
    struct rmca_traits_transpose
    {
      typedef rmca_traits<
          traits_op1::ind_Y ==0 ? 1 : (traits_op1::ind_Z == 0 ? 2 : 0) ,
          traits_op1::ind_X == 1 ? 0 : (traits_op1::ind_Z == 1 ? 2 : 1),
          traits_op1::ind_X == 2 ? 0 : (traits_op1::ind_Y == 2 ? 1 : 2),
          traits_op1::value_X,
          traits_op1::value_Y,
          traits_op1::value_Z>
      traits_transpose;
      static int const ind_X = traits_transpose::ind_X;
      static int const value_X = traits_transpose::value_X;
      static int const ind_Y = traits_transpose::ind_Y;
      static int const value_Y = traits_transpose::value_Y;
      static int const ind_Z = traits_transpose::ind_Z;
      static int const value_Z = traits_transpose::value_Z;
    };

    template <typename traits_op1>
    struct rmca_traits_minus
    {
      typedef rmca_traits<
          traits_op1::ind_X,traits_op1::ind_Y,traits_op1::ind_Z,
          -traits_op1::value_X,-traits_op1::value_Y,-traits_op1::value_Z>
          traits_opp;
      static int const ind_X = traits_opp::ind_X; 
      static int const value_X = traits_opp::value_X;
      static int const ind_Y = traits_opp::ind_Y; 
      static int const value_Y = traits_opp::value_Y;
      static int const ind_Z = traits_opp::ind_Z; 
      static int const value_Z = traits_opp::value_Z;
    };

    /** \object Rotation matrix as axis permutation matrix.
        This \f$ 3 \times 3 \f$ matrix \f$ A\f$ is defined by 7 parameters:
        \f$ iX, iY, iZ, lX, lY, lZ, rc\f$.
        \f$ lX, lY, lZ \in \{ -1,1 \}, rc \in \{0,1\} \f$
        \f[
          \begin{matrix}
          A(0,iX) = lX & A(0,j) = 0 \text{ if } j \neq iX \\
          A(1,iY) = lY & A(1,j) = 0 \text{ if } j \neq iY \\
          A(2,iZ) = lZ & A(2,j) = 0 \text{ if } j \neq iZ \\
        \f]
     */
    template < typename traits >
    struct RotationMatrixChangeAxis
    {
      typedef traits Traits;
      RotationMatrixChangeAxis()
      {
      }

      RotationMatrixChangeAxis(const RotationMatrixChangeAxis &)
      {
      }

      RotationMatrixChangeAxis transpose() const
      {
        typedef rmca_traits<
            traits::ind_Y ==0 ? 1 : (traits::ind_Z == 0 ? 2 : 0) ,
            traits::ind_X == 1 ? 0 : (traits::ind_Z == 1 ? 2 : 1),
            traits::ind_X == 2 ? 0 : (traits::ind_Y == 2 ? 1 : 2),
            traits::value_X,
            traits::value_Y,
            traits::value_Z> traits_transpose;
        return RotationMatrixChangeAxis<traits_transpose>();
      }

      Matrix3d toMatrix()
      {
        Matrix3d r=Matrix3d::Zero();
        r(0,traits::ind_X) = traits::value_X;
        r(1,traits::ind_Y) = traits::value_Y;
        r(2,traits::ind_Z) = traits::value_Z;
        return r;
      }


      Matrix3d operator*(FloatType a) const
      {
        Matrix3d r=toMatrix();
        return Matrix3d(a*r);
      }

      void randomInit() { }

      RotationMatrixChangeAxis operator-() const
      {
        return RotationMatrixChangeAxis<rmca_traits_minus<traits> > ();
      }

      Matrix3d operator*(const Matrix3d &A) const
      {
        Matrix3d r;
        r.block<1,3>(0,0) = traits::value_X*A.block<1,3>(traits::ind_X,0);
        r.block<1,3>(1,0) = traits::value_Y*A.block<1,3>(traits::ind_Y,0);
        r.block<1,3>(2,0) = traits::value_Z*A.block<1,3>(traits::ind_Z,0);
        return r;
      }

      template< typename traits_rhs>
      RotationMatrixChangeAxis< rmca_traits_mul<traits,traits_rhs> >
      operator*(const RotationMatrixChangeAxis<traits_rhs> &)
      {
        RotationMatrixChangeAxis< rmca_traits_mul<traits,traits_rhs> > r;
        return r;
      }

      friend Matrix3d operator*(const Matrix3d &A,
                                const RotationMatrixChangeAxis &aRMCA)
      {
        Matrix3d r;
        r.block<3,1>(0,0) = traits::value_X*A.block<3,1>(0,traits::ind_X);
        r.block<3,1>(0,1) = traits::value_Y*A.block<3,1>(0,traits::ind_Y);
        r.block<3,1>(0,2) = traits::value_Z*A.block<3,1>(0,traits::ind_Z);
        return r;
      }


      RotationMatrix operator*(const RotationMatrix &aRM) const
      {
        const Matrix3d & lrm = aRM.m_rm;
        Matrix3d r = (*this) * lrm;
        return RotationMatrix(r);
      }

      RotationMatrix operator*(const RotationMatrixAboutX &aRM) const
      {
        Matrix3d r=Matrix3d::Zero();
        r(traits::ind_X,0)=  traits::value_X;
        r(traits::ind_Y,1)=  traits::value_Y*aRM.m_c;
        r(traits::ind_Y,2)= traits::value_Y*aRM.m_s;
        r(traits::ind_Z,1)= -traits::value_Z*aRM.m_s;
        r(traits::ind_Z,2)= traits::value_Z*aRM.m_c;
        return RotationMatrix(r);
      }

      RotationMatrix operator*(const RotationMatrixAboutY &aRM) const
      {
        Matrix3d r=Matrix3d::Zero();
        r(traits::ind_X,0)=  traits::value_X*aRM.m_c;
        r(traits::ind_X,2)= -traits::value_X*aRM.m_s;
        r(traits::ind_Y,1)=  traits::value_Y;
        r(traits::ind_Z,0)=  traits::value_Z*aRM.m_s;
        r(traits::ind_Z,2)=  traits::value_Z*aRM.m_c;
        return RotationMatrix(r);
      }

      RotationMatrix operator*(const RotationMatrixAboutZ &aRM) const
      {
        Matrix3d r=Matrix3d::Zero();
        r(traits::ind_X,0)=  traits::value_X*aRM.m_c;
        r(traits::ind_X,1)= traits::value_X*aRM.m_s;

        r(traits::ind_Y,1)= -traits::value_Y*aRM.m_s;
        r(traits::ind_Y,2)= traits::value_Y*aRM.m_c;;
        r(traits::ind_Z,2)=  traits::value_Z;
        return RotationMatrix(r);
      }

      Matrix3d rotGeneralMatrix(const Matrix3d &A) const
      {
        Matrix3d r;
        static const int lv[3] = { traits::value_X, traits::value_Y, traits::value_Z };

        r(0,0) = lv[traits::ind_X] * lv[traits::ind_X] * A(traits::ind_X,traits::ind_X);
        r(0,1) = lv[traits::ind_X] * lv[traits::ind_Y] * A(traits::ind_X,traits::ind_Y);
        r(0,2) = lv[traits::ind_X] * lv[traits::ind_Z] * A(traits::ind_X,traits::ind_Z);

        r(1,0) = lv[traits::ind_Y] * lv[traits::ind_X] * A(traits::ind_Y,traits::ind_X);
        r(1,1) = lv[traits::ind_Y] * lv[traits::ind_Y] * A(traits::ind_Y,traits::ind_Y);
        r(1,2) = lv[traits::ind_Y] * lv[traits::ind_Z] * A(traits::ind_Y,traits::ind_Z);

        r(2,0) = lv[traits::ind_Z] * lv[traits::ind_X] * A(traits::ind_Z,traits::ind_X);
        r(2,1) = lv[traits::ind_Z] * lv[traits::ind_Y] * A(traits::ind_Z,traits::ind_Y);
        r(2,2) = lv[traits::ind_Z] * lv[traits::ind_Z] * A(traits::ind_Z,traits::ind_Z);
        return r;
      }

      ltI rotSymmetricMatrix(const ltI &A)
      {
        ltI r;
        static const int lv[3] = { traits::value_X, traits::value_Y, traits::value_Z };
        static const int lind[9] = { 0, 1, 3,
                                     1, 2, 4,
                                     3, 4, 5};

        r.m_ltI(0) = lv[traits::ind_X]*lv[traits::ind_X] * A.m_ltI(lind[traits::ind_X*3+traits::ind_X]);

        r.m_ltI(1) = lv[traits::ind_Y]*lv[traits::ind_X] * A.m_ltI(lind[traits::ind_Y*3+traits::ind_X]);
        r.m_ltI(2) = lv[traits::ind_Y]*lv[traits::ind_Y] * A.m_ltI(lind[traits::ind_Y*3+traits::ind_Y]);

        r.m_ltI(3) = lv[traits::ind_Z]*lv[traits::ind_X] * A.m_ltI(lind[traits::ind_Z*3+traits::ind_X]);
        r.m_ltI(4) = lv[traits::ind_Z]*lv[traits::ind_Y] * A.m_ltI(lind[traits::ind_Z*3+traits::ind_Y]);
        r.m_ltI(5) = lv[traits::ind_Z]*lv[traits::ind_Z] * A.m_ltI(lind[traits::ind_Z*3+traits::ind_Z]);
        return r;
      }

      Vector3d operator*(const Vector3d &ar) const
      {
        return Vector3d(traits::value_X*ar[traits::ind_X],
                        traits::value_Y*ar[traits::ind_Y],
                        traits::value_Z*ar[traits::ind_Z]);
      }

      friend std::ostream & operator<<(std::ostream &os,
                                       const RotationMatrixChangeAxis<traits> & aRMCA)
      {
        static const int lind[3] = { traits::ind_X, traits::ind_Y, traits::ind_Z };
        static const int lvalue[3] = { traits::value_X, traits::ind_Y, traits::ind_Z };

        for(unsigned int li=0;li<3;++li)
        {
          for(unsigned int lj=0;lj<3;++lj)
            if (lj==lind[li])
              cout << " " << lvalue[li] ;
            else
              cout << " 0.0";
          cout << endl;
        }
        return os;
      }
    };

    template < typename RotationMatrixT,
               typename RotMatChgAxis>
    struct RotationBinaryOp
    {
    private:
      RotMatChgAxis m_rmca;
      RotationMatrixT m_RM;
      Matrix3d m_r;
    public:

      RotationBinaryOp(RotationMatrix &lrm, RotMatChgAxis &lrmca):
        m_RM(lrm)
      {
        m_r = lrm.toMatrix() * lrmca.toMatrix();
      }

      RotationBinaryOp transpose() const
      {
        RotationMatrixChangeAxis< rmca_traits_transpose<typename RotMatChgAxis::Traits> > lrmca;
        RotationBinaryOp< rmca_traits_transpose<typename RotMatChgAxis::Traits>,
            RotationMatrixT> r(lrmca,m_RM.toMatrix().transpose());
        return r;
      }

      Matrix3d toMatrix() const
      {
        return m_r;
      }

      Matrix3d operator*(FloatType a) const
      {
        return Matrix3d(a*toMatrix());
      }

      RotationBinaryOp operator-() const
      {
        RotationMatrixChangeAxis< rmca_traits_minus<typename RotMatChgAxis::Traits> > lrmca;
        RotationBinaryOp< RotationMatrixT,
            rmca_traits_minus<typename RotMatChgAxis::Traits> >
            r(lrmca,-m_RM.toMatrix());
        return r;
      }

      Matrix3d operator*(const Matrix3d &A) const
      {
        return Matrix3d(m_r * A);
      }

      RotationMatrix operator*(const RotationMatrix &aRM) const
      {
        return RotationMatrix(m_r *aRM.toMatrix());
      }

      RotationMatrix operator*(const RotationMatrixAboutX &aRM) const
      {
        return RotationMatrix(m_r *aRM.toMatrix());
      }

      Vector3d operator*(const Vector3d &av) const
      {
        return Vector3d (m_RM * ( m_rmca * av));
      }

    };

  } // end Spatial namespace
} // end metapod namespace

#endif // METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_CHGAXIS_HH
