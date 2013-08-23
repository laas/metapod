// Copyright 2012,
//
// Maxime Reis
// Antonio El Khoury
//
// JRL/LAAS, CNRS/AIST
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


#ifndef METAPOD_SPATIAL_ALGEBRA_TRANSFORM_HH
# define METAPOD_SPATIAL_ALGEBRA_TRANSFORM_HH

# include <metapod/tools/spatial/cm-anyaxis.hh>
# include <metapod/tools/spatial/cm-oneaxis.hh>
# include <metapod/tools/spatial/cm-freeflyer.hh>

# include <iostream>
# include "metapod/tools/fwd.hh"
# include <metapod/tools/spatial/rotation-matrix.hh>

namespace metapod
{

  namespace Spatial
  {


    /// Given two frames a and b, one can define the transform bXa, which
    /// changes coordinates between the two. bXa is composed of a rotation
    /// matrix E, which changes vector coordinates from a to b, and a vector
    /// r, which gives the position of the origin of b, expressed in the a
    /// frame.
    /// So if v is a vector and p is a point, we have:
    ///
    ///   vb = bXa.E * va
    ///   pb = bXa.E * (pa - bXa.r)
    template <class RotationClass> 
    class TransformT
    {
      public:
        // Constructors
        TransformT() : m_E(), m_r() {}
        TransformT(const Matrix3d & E, const Vector3d & r) : m_E(E), m_r(r) {}
        TransformT(const RotationClass & E, const Vector3d & r) : m_E(E), m_r(r) {}
        static const TransformT Identity()
        {
          return TransformT (Matrix3d::Identity(), Vector3d::Zero());
        }

        // Getters
        const Vector3d & r() const { return m_r; }
        const RotationClass & E() const { return m_E; }

        // Transformations

        /// Vb = bXa.apply(Va)
        Motion apply(const Motion & mv) const
        {
          return Motion(m_E * mv.w(),
                        m_E * (mv.v() - m_r.cross(mv.w())));
        }

        /// Fb = bXa.apply(Fa)
        Force apply(const Force & fv) const
        {
          return Force(m_E*(fv.n() - m_r.cross(fv.f())), m_E*fv.f());
        }

        /// Ib = bXa.apply(Ia)
        Inertia apply(const Inertia & I) const
        {
          Vector3d tmp = I.h() - I.m()*m_r;

          return Inertia(I.m(),
                         m_E*tmp,
                         m_E*(I.I()
                         + skew(m_r)*skew(I.h())
                         + skew(tmp)*skew(m_r))*m_E.transpose());
        }

        /// Pb = bXa.apply(Pa)
        Vector3d apply(const Vector3d& p) const
        {
          return m_E*static_cast<Vector3d>((p - m_r));
        }

        /// Sb = bXa.apply(Sa)
        ///
        /// Specialization for JOINT_REVOLUTE_AXIS_ANY
        Vector6d apply(const ConstraintMotionAnyAxis& S) const
        {
          Vector6d tmp;
          tmp.head<3>() = m_E*static_cast<Vector3d>(S.S().head<3>());
          tmp.tail<3>() = -(m_E*m_r.cross(S.S().head<3>()));
          return tmp;
        }

        /// Sb = bXa.apply(Sa)
        ///
        /// Specialization for JOINT_REVOLUTE_AXIS_X
        Vector6d apply(const ConstraintMotionAxisX& ) const
        {
          Vector6d tmp;
          tmp.head<3>() = m_E.col(0);
          tmp.tail<3>() = m_E*Vector3d(0,-m_r[2],m_r[1]);
          return tmp;
        }

        /// Sb = bXa.apply(Sa)
        ///
        /// Specialization for JOINT_REVOLUTE_AXIS_Y
        Vector6d apply(const ConstraintMotionAxisY& ) const
        {
          Vector6d tmp;
          tmp.head<3>() = m_E.col(1);
          tmp.tail<3>() = m_E*Vector3d(m_r[2], 0, -m_r[0]);
          return tmp;
        }

        /// Sb = bXa.apply(Sa)
        ///
        /// Specialization for JOINT_REVOLUTE_AXIS_Z
        Vector6d apply(const ConstraintMotionAxisZ& ) const
        {
          Vector6d tmp;
          tmp.head<3>() = m_E.col(2);
          tmp.tail<3>() = m_E*Vector3d(-m_r[1], m_r[0], 0);
          return tmp;
        }

        /// Sb = bXa.apply(Sa)
        ///
        /// Specialization for JOINT_FREE_FLYER
        Matrix6d apply(const ConstraintMotionFreeFlyer& S) const
        {
          Matrix6d tmp = Matrix6d::Zero();

          tmp.topRightCorner<3,3>() = m_E * static_cast<Matrix3d>(S.S().topRightCorner<3,3>());
          tmp.bottomLeftCorner<3,3>() = m_E * static_cast<Matrix3d>(S.S().bottomLeftCorner<3,3>());
          tmp.bottomRightCorner<3,3>()
            = -(m_E * Spatial::skew(m_r) * S.S().topRightCorner<3,3>());
          return tmp;
        }

        /// Vb.toVector() = bXa.toMatrix() * Va.toVector()
        Matrix6d toMatrix() const
        {
          Matrix6d M;
          M.block<3,3>(0,0) = M.block<3,3>(3,3) = m_E.toMatrix();
          M.block<3,3>(0,3).setZero();
          M.block<3,3>(3,0) = -M.block<3,3>(0,0) * skew(m_r);

          return M;
        }

        /// Fa.toVector() = bXa.toMatrixTranspose() * Fb.toVector()
        Matrix6d toMatrixTranspose() const
        {
          Matrix6d M;
          M.block<3,3>(0,0) = m_E.transpose();
          M.block<3,3>(3,0).setZero();
          M.block<3,3>(0,3) = (-m_E * skew(m_r)).transpose();
          M.block<3,3>(3,3) = m_E.transpose();
          return M;
        }


        /// Fb.toVector() = bXa.toMatrixTransposeInverse() * Fa.toVector()
        Matrix6d toMatrixTransposeInverse() const
        {
          Matrix6d M;
          M.block<3,3>(0,0) = m_E;
          M.block<3,3>(0,3) = -m_E * skew(m_r);
          M.block<3,3>(3,0) = Matrix3d::Zero();
          M.block<3,3>(3,3) = m_E;
          return M;
        }

        /// Va = bXa.applyInv(Vb)
        Motion applyInv(const Motion & mv) const
        {
          Vector3d ET_w = static_cast<Vector3d>(m_E.transpose()*mv.w());
          return Motion(ET_w, m_E.transpose()*mv.v() + m_r.cross(ET_w));
        }

        /// Fa = bXa.applyInv(Fb)
        Force applyInv(const Force & fv) const
        {
          Vector3d ET_f = static_cast<Vector3d>(m_E.transpose()*fv.f());
          return Force(m_E.transpose()*fv.n() + m_r.cross(ET_f), ET_f);
        }

        /// Ia = bXa.applyInv(Ib)
        Inertia applyInv(const Inertia & I) const
        {
          Vector3d tmp1 = static_cast<Vector3d>(m_E.transpose()*I.h());
          Vector3d tmp2 = static_cast<Vector3d>(tmp1 + I.m()*m_r);
          lowerTriangularMatrix aEIEt = m_E.rotTSymmetricMatrix(I.I());

          return Inertia(I.m(),
                         tmp2,
                         aEIEt
                         - skew(m_r)*skew(tmp1)
                         - skew(tmp2)*skew(m_r));
        }


        /// Pa = bXa.applyInv(Pb)
        Vector3d applyInv(const Vector3d& p) const
        {
          return m_E.transpose()*p + m_r;
        }
      
        Vector6d mulMatrixTransposeBy(Vector6d &aF) const;
      
        /// aXb = bXa.inverse()
        TransformT inverse() const
        {
          return TransformT(m_E.transpose(), -(m_E*m_r));
        }

        // Arithmetic operators
        TransformT operator*(FloatType a) const
        {
          return TransformT(a*m_E, a*m_r);
        }

        // bXz = bXa * aXz
        TransformT operator*(const TransformT & X) const
        {
          return TransformT(m_E*X.E(), 
                            (Vector3d)(X.r() + X.E().transpose()*m_r));
        }

        /// Specialization of transform multiplication.
        /// Compute transform cXa from bXa, with cXb a translation defined by
        /// the vector Pb (expressed in frame b).
        ///
        /// In a nutshell:
        ///
        /// cXb == TransformT(Eye, Pb)
        /// cXa == cXb * bXa == bXa.toPointFrame(Pb)
        TransformT toPointFrame(const Vector3d& p) const
        {
          return TransformT (m_E, m_r + m_E.transpose()*p);
        }

        /// Vb = bXa * Va
        Motion operator*(const Motion & mv) const
        {
          return Motion(m_E * mv.w(), m_E * static_cast<Vector3d>(mv.v() - m_r.cross(mv.w())));
        }

        /// Fb = bXa * Fa
        Force operator*(const Force & fv) const
        {
          Vector3d lf = fv.f();
          return Force(m_E*static_cast<Vector3d>(fv.n() - m_r.cross(lf)), m_E*lf);
        }

        /// Ib = bXa * Ia
        Inertia operator*(const Inertia & I) const
        {
          Vector3d tmp = I.h() - I.m()*m_r;
          return Inertia(I.m(),
                         m_E*tmp,
                         m_E*(I.I()
                              + skew(m_r)*skew(I.h())
                              + skew(tmp)*skew(m_r))*m_E.transpose());
        }

        template<class S>
        TransformT< typename rm_mul_op<RotationClass,S>::rm >
        operator*(const TransformT<S> &aTsf) const
        {
          typedef typename rm_mul_op<RotationClass,S>::rm RMResult;
          RMResult  aR=E()*aTsf.E(); 
          Vector3d aV= (Vector3d)(aTsf.r() +  aTsf.E().transpose()*r());
          return TransformT<RMResult>(aR,aV);
        }

        // Print operator
        friend std::ostream & operator << (std::ostream & os,
                                           const TransformT & X)
        {
          os
            << "  E =\n" << X.E() << std::endl
            << "  r =\n" << X.r().transpose() << std::endl;
          return os;
        }


      private:
        // Private members
        // Matrix3d m_E;
        RotationClass m_E;
        Vector3d m_r;
    };

    typedef TransformT<RotationMatrix> Transform;
    typedef TransformT<RotationMatrixAboutX> TransformX;
    typedef TransformT<RotationMatrixAboutY> TransformY;
    typedef TransformT<RotationMatrixAboutZ> TransformZ;
    typedef TransformT<RotationMatrixIdentity> TransformId;

    template <> inline
    Vector6d TransformT<RotationMatrixAboutX>::mulMatrixTransposeBy(Vector6d &aF) const
    {
      Vector6d M;
      M[0] = aF(0);
      M[1] = m_E.m_c*aF(1) - m_E.m_s*aF(2);
      M[2] = m_E.m_s*aF(1) + m_E.m_c*aF(2);
          
      M[0] += (-m_E.m_c*m_r(2) + m_E.m_s*m_r(1)) * aF(4) + 
        (m_E.m_s*m_r(2) + m_E.m_c*m_r(1)) * aF(5) ;

      M[1] += m_r(2) * aF(3) 
        - m_E.m_s*m_r(0) * aF(4) 
        - m_E.m_c*m_r(0) * aF(5) ;

      M[2] += -m_r(1)* aF(3) + 
        m_E.m_c*m_r(0) * aF(4) 
        -m_E.m_s*m_r(0) * aF(5);
          
      M[3] = aF(3) ;
      M[4] = m_E.m_c*aF(4) - m_E.m_s*aF(5);
      M[5] = m_E.m_s*aF(4) + m_E.m_c*aF(5);
          
      return M;
    }

    template <> inline
    Vector6d TransformT<RotationMatrixAboutY>::mulMatrixTransposeBy(Vector6d &aF) const
    {
      Vector6d M;
      M[0] = m_E.m_c*aF(0) + m_E.m_s*aF(2);
      M[1] = aF(1);
      M[2] = -m_E.m_s*aF(0) + m_E.m_c*aF(2);
          
      M[0] += -m_E.m_s*m_r(1) * aF(3)
              - m_r(2) * aF(4)
              + m_E.m_c*m_r(1) * aF(5);

      M[1] += (m_E.m_c*m_r(2) + m_E.m_s*m_r(0)) * aF(3) + 
              (m_E.m_s*m_r(2) - m_E.m_c*m_r(0)) * aF(5);

      M[2] += -m_E.m_c*m_r(1) * aF(3) +
              m_r(0)* aF(4) +
              -m_E.m_s*m_r(1) * aF(5);

      M[3] = m_E.m_c*aF(3) + m_E.m_s*aF(5) ;
      M[4] = aF(4);
      M[5] =-m_E.m_s*aF(3) + m_E.m_c*aF(5);
          
      return M;
    }

    template <> inline
    Vector6d TransformT<RotationMatrixAboutZ>::mulMatrixTransposeBy(Vector6d &aF) const
    {
      Vector6d M;
      M[0] = m_E.m_c*aF(0) - m_E.m_s*aF(1);
      M[1] = m_E.m_s*aF(0) + m_E.m_c*aF(1);
      M[2] = aF(2);
          
      M[0] += -m_E.m_s * m_r(2) * aF(3)
              - m_E.m_c * m_r(2) * aF(4)
              + m_r(1) * aF(5);

      M[1] += m_E.m_c * m_r(2) * aF(3)
              - m_E.m_s * m_r(2) * aF(4)
              - m_r(0) * aF(5);

      M[2] += (-m_E.m_c*m_r(1) + m_E.m_s*m_r(0)) * aF(3) +
        (m_E.m_s*m_r(1) + m_E.m_c*m_r(0)) * aF(4);
          
      M[3] = m_E.m_c*aF(3) - m_E.m_s*aF(4);
      M[4] = m_E.m_s*aF(3) + m_E.m_c*aF(4);
      M[5] = aF(5);
          
      return M;
    }

    template <> inline
    Vector6d TransformT<RotationMatrix>::mulMatrixTransposeBy(Vector6d &aF) const 
    {
      Vector6d M;
          
      M[0] = m_E(0,0)*aF(0) + m_E(1,0)*aF(1) + m_E(2,0)*aF(2);
      M[1] = m_E(0,1)*aF(0) + m_E(1,1)*aF(1) + m_E(2,1)*aF(2);
      M[2] = m_E(0,2)*aF(0) + m_E(1,2)*aF(1) + m_E(2,2)*aF(2);
          
      M[0] += (-m_E(0,1)*m_r(2) + m_E(0,2)*m_r(1))* aF(3) + 
        (-m_E(1,1)*m_r(2) + m_E(1,2)*m_r(1)) * aF(4) + 
        (-m_E(2,1)*m_r(2) + m_E(2,2)*m_r(1)) * aF(5) ;

      M[1] += ( m_E(0,0)*m_r(2) - m_E(0,2)*m_r(0))* aF(3) + 
        ( m_E(1,0)*m_r(2) - m_E(1,2)*m_r(0)) * aF(4) + 
        ( m_E(2,0)*m_r(2) - m_E(2,2)*m_r(0)) * aF(5) ;

      M[2] += (-m_E(0,0)*m_r(1) + m_E(0,1)*m_r(0))* aF(3) + 
        (-m_E(1,0)*m_r(1) + m_E(1,1)*m_r(0)) * aF(4) + 
        (-m_E(2,0)*m_r(1) + m_E(2,1)*m_r(0)) * aF(5);
          
      M[3] = m_E(0,0)*aF(3) + m_E(1,0)*aF(4) + m_E(2,0)*aF(5);
      M[4] = m_E(0,1)*aF(3) + m_E(1,1)*aF(4) + m_E(2,1)*aF(5);
      M[5] = m_E(0,2)*aF(3) + m_E(1,2)*aF(4) + m_E(2,2)*aF(5);
          
      return M;
    }
  
  } // end of namespace Spatial


} // end of namespace metapod

# endif
