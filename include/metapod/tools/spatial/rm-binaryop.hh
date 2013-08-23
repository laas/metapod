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


#ifndef METAPOD_SPATIAL_ALGEBRA_ROTATION_BINARY_OP_HH
# define METAPOD_SPATIAL_ALGEBRA_ROTATION_BINARY_OP_HH

namespace metapod
{

  
  namespace Spatial
  {
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

#endif // METAPOD_SPATIAL_ALGEBRA_ROTATION_BINARY_OP_HH
