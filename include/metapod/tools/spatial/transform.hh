// Copyright 2012, 2014
//
// Maxime Reis
// Antonio El Khoury
// Olivier Stasse
// Nuno Guedelha
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


# include <iostream>
# include "metapod/tools/spatial/constraintmotion.hh"
# include <metapod/tools/spatial/rotation-matrix.hh>

namespace metapod
{

namespace Spatial
{
    
namespace internal {
/// Given two frames a and b, one can define the transform bXa, which
/// changes coordinates between the two. bXa is composed of a rotation
/// matrix E, which changes vector coordinates from a to b, and a vector
/// r, which gives the position of the origin of b, expressed in the a
/// frame.
/// So if v is a vector and p is a point, we have:
///
///   vb = bXa.E * va
///   pb = bXa.E * (pa - bXa.r)
template <typename FloatType,
          typename RotationClass > 
class TransformT_helper
{
  EIGEN_METAPOD_TYPEDEFS;
  EIGEN_METAPOD_CM_TYPEDEFS;
  EIGEN_METAPOD_SPATIAL_MOTION_TYPEDEF;
  EIGEN_METAPOD_SPATIAL_INERTIA_TYPEDEF;
  EIGEN_METAPOD_SPATIAL_FORCE_TYPEDEF;
 public:
  // Constructors
  TransformT_helper() : m_E(), m_r() {}
  TransformT_helper(const Matrix3d & E, const typename Vector3dTpl<FloatType>::Type & r) : m_E(E), m_r(r) {}
  TransformT_helper(const RotationClass & E, const typename Vector3dTpl<FloatType>::Type & r) : m_E(E), m_r(r) {}
  static const TransformT_helper Identity()
  {
    return TransformT_helper (Matrix3d::Identity(), Vector3d::Zero());
  }

  // Getters
  const typename Vector3dTpl<FloatType>::Type & r() const { return m_r; }
  const RotationClass & E() const { return m_E; }

  // Transformations

  /// Vb = bXa.apply(Va)
  Motion apply(const Motion & mv) const
  {
    return Motion(m_E * mv.w(),
                  m_E * (mv.v() - m_r.cross(mv.w())).eval());
  }

  /// Fb = bXa.apply(Fa)
  Force apply(const Force & fv) const
  {
    return Force(m_E*(fv.n() - m_r.cross(fv.f())).eval(), m_E*fv.f());
  }

  /// Ib = bXa.apply(Ia)
  Inertia apply(const Inertia & I) const
  {
    typename Vector3dTpl<FloatType>::Type tmp = I.h() - I.m()*m_r;
    const Matrix3d E3(I.I()
                      +skew<FloatType>(r())*skew<FloatType>(I.h())
                      +skew<FloatType>(tmp)*skew<FloatType>(m_r));
    const Matrix3d E4(m_E*E3);

    return Inertia(I.m(), m_E*tmp,E4*m_E.toMatrix().transpose());
  }

  /// Pb = bXa.apply(Pa)
  typename Vector3dTpl<FloatType>::Type
  apply(const typename Vector3dTpl<FloatType>::Type & p) const
  {
    return m_E*static_cast<typename Vector3dTpl<FloatType>::Type >((p - m_r));
  }


  /// Sb = bXa.apply(Sa)
  ///
  /// Specialization for JOINT_REVOLUTE_AXIS_ANY
  Vector6d apply(const ConstraintMotionAnyAxis<FloatType>& S) const
  {
    Vector6d tmp;
    tmp.template head<3>() = 
      m_E*static_cast<typename Vector3dTpl<FloatType>::Type >(S.S().template head<3> () );
    tmp.template tail<3>() = -(m_E*m_r.cross(S.S().template head<3>() ) );
    return tmp;
  }

  /// Sb = bXa.apply(Sa)
  ///
  /// Specialization for JOINT_REVOLUTE_AXIS_X
  Vector6d apply(const ConstraintMotionAxisX & ) const
  {
    Vector6d tmp;
    tmp.template head<3>() = m_E.col(0);
    tmp.template tail<3>() = m_E*typename Vector3dTpl<FloatType>::Type(0,-m_r[2],m_r[1]);
    return tmp;
  }

  /// Sb = bXa.apply(Sa)
  ///
  /// Specialization for JOINT_REVOLUTE_AXIS_Y
  Vector6d apply(const ConstraintMotionAxisY& ) const
  {
    Vector6d tmp;
    tmp.template head<3>() = m_E.col(1);
    tmp.template tail<3>() = m_E*typename Vector3dTpl<FloatType>::Type(m_r[2], 0, -m_r[0]);
    return tmp;
  }

  /// Sb = bXa.apply(Sa)
  ///
  /// Specialization for JOINT_REVOLUTE_AXIS_Z
  Vector6d apply(const ConstraintMotionAxisZ & ) const
  {
    Vector6d tmp;
    tmp.template head<3>() = m_E.col(2);
    tmp.template tail<3>() = m_E*typename Vector3dTpl<FloatType>::Type(-m_r[1], m_r[0], 0);
    return tmp;
  }

  /// Sb = bXa.apply(Sa)
  ///
  /// Specialization for JOINT_FREE_FLYER
  Matrix6d apply(const ConstraintMotionFreeFlyer & S) const
  {
    Matrix6d tmp = Matrix6d::Zero();

    tmp.template topRightCorner<3,3>() = 
        m_E * static_cast<Matrix3d>(S.S().template topRightCorner<3,3>());
    tmp.template bottomLeftCorner<3,3>() = 
        m_E * static_cast<Matrix3d>(S.S().template bottomLeftCorner<3,3>());
    tmp.template bottomRightCorner<3,3>()
        = -(m_E * Spatial::skew<FloatType>(m_r) * S.S().template topRightCorner<3,3>());
    return tmp;
  }

  /// Vb.toVector() = bXa.toMatrix() * Va.toVector()
  Matrix6d toMatrix() const
  {
    Matrix6d M;
    M.template block<3,3>(0,0) = M.template block<3,3>(3,3) = m_E.toMatrix();
    M.template block<3,3>(0,3).setZero();
    M.template block<3,3>(3,0) = -M.template block<3,3>(0,0) * skew<FloatType>(m_r);

    return M;
  }

  /// Fa.toVector() = bXa.toMatrixTranspose() * Fb.toVector()
  Matrix6d toMatrixTranspose() const
  {
    Matrix6d M;
    M.template block<3,3>(0,0) = m_E.transpose();
    M.template block<3,3>(3,0).setZero();
    M.template block<3,3>(0,3) = (-m_E * skew(m_r)).transpose();
    M.template block<3,3>(3,3) = m_E.transpose();
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
    typename Vector3dTpl<FloatType>::Type ET_w = static_cast<typename Vector3dTpl<FloatType>::Type >(m_E.transpose()*mv.w());
    return Motion(ET_w, m_E.transpose()*mv.v() + m_r.cross(ET_w));
  }

  /// Fa = bXa.applyInv(Fb)
  Force applyInv(const Force & fv) const
  {
    typename Vector3dTpl<FloatType>::Type ET_f = static_cast<typename Vector3dTpl<FloatType>::Type >(m_E.transpose()*fv.f());
    return Force(m_E.transpose()*fv.n() + m_r.cross(ET_f), ET_f);
  }

  /// Ia = bXa.applyInv(Ib)
  Inertia applyInv(const Inertia & I) const
  {
    typename Vector3dTpl<FloatType>::Type tmp1 = static_cast<typename Vector3dTpl<FloatType>::Type >(m_E.transpose()*I.h());
    typename Vector3dTpl<FloatType>::Type tmp2 = static_cast<typename Vector3dTpl<FloatType>::Type >(tmp1 + I.m()*m_r);
    ltI<FloatType> aEtIE = m_E.rotTSymmetricMatrix(I.I());
    return Inertia(I.m(),
                   tmp2,
                   aEtIE
                   - skew<FloatType>(m_r)*skew<FloatType>(tmp1)
                   - skew<FloatType>(tmp2)*skew<FloatType>(m_r));
  }


  /// Pa = bXa.applyInv(Pb)
  typename Vector3dTpl<FloatType>::Type 
  applyInv(const typename Vector3dTpl<FloatType>::Type & p) const
  {
    return m_E.transpose()*p + m_r;
  }
      
       
  /// aXb = bXa.inverse()
  template <template <typename LFloatType, typename LRotationClass> class T >
  T<FloatType, RotationClass> inverse() const
  {
    return T<FloatType, RotationClass>(m_E.transpose(), -(m_E*m_r));
  }

  /// Specialization of transform multiplication.
  /// Compute transform cXa from bXa, with cXb a translation defined by
  /// the vector Pb (expressed in frame b).
  ///
  /// In a nutshell:
  ///
  /// cXb == TransformT(Eye, Pb)
  /// cXa == cXb * bXa == bXa.toPointFrame(Pb)
  template <template <typename U, typename C> class T>
  T<FloatType, RotationClass> toPointFrame(const Vector3dTpl<FloatType>& p) const
  {
    return T<FloatType,RotationClass> (m_E, m_r + m_E.transpose()*p);
  }

  /// Vb = bXa * Va
  Motion operator*(const Motion & mv) const
  {
    return Motion(m_E * mv.w(), m_E * static_cast<typename Vector3dTpl<FloatType>::Type >(mv.v() - m_r.cross(mv.w())));
  }

  /// Fb = bXa * Fa
  Force operator*(const Force & fv) const
  {
    typename Vector3dTpl<FloatType>::Type lf = fv.f();
    return Force(m_E*static_cast<typename Vector3dTpl<FloatType>::Type >(fv.n() - m_r.cross(lf)), m_E*lf);
  }

  /// Ib = bXa * Ia
  Inertia operator*(const Inertia & I) const
  {
    typename Vector3dTpl<FloatType>::Type tmp = I.h() - I.m()*m_r;
    return Inertia(I.m(),
                   m_E*tmp,
                   m_E*(I.I()
                        + skew(m_r)*skew(I.h())
                        + skew(tmp)*skew(m_r))*m_E.transpose());
  }

  template<template <typename TFloatType, typename TRotationMatrix> class TransformT,
           typename S>
  TransformT< FloatType, typename rm_mul_op<FloatType, RotationClass, S >::rm >
  operator*(const TransformT<FloatType, S> &aTsf) const
  {
    typedef typename rm_mul_op<FloatType,RotationClass,S>::rm RMResult;
    RMResult aR= m_E*aTsf.E(); 
    typename Vector3dTpl<FloatType>::Type 
      aV= (typename Vector3dTpl<FloatType>::Type)(aTsf.r() +  aTsf.E().transpose()*m_r);
    return TransformT<FloatType, RMResult>(aR,aV);
  }

  // Print operator
  friend std::ostream & operator << (std::ostream & os,
                                     const TransformT_helper & X)
  {  os << "  E =\n" << X.E() << std::endl
    << "  r =\n" << X.r().transpose() << std::endl;
    return os;
  }

  // Private members
  // Matrix3d m_E;
  RotationClass m_E;
  typename Vector3dTpl<FloatType>::Type m_r;
};

} // namespace internal

template <typename FloatType,
          typename RotationClass>
class TransformT: public internal::TransformT_helper<FloatType, RotationClass >
{
  EIGEN_METAPOD_TYPEDEFS;
  EIGEN_METAPOD_CM_TYPEDEFS;
  EIGEN_METAPOD_SPATIAL_MOTION_TYPEDEF;
  EIGEN_METAPOD_SPATIAL_INERTIA_TYPEDEF;
  EIGEN_METAPOD_SPATIAL_FORCE_TYPEDEF;

 public:
  TransformT(): internal::TransformT_helper<FloatType,RotationClass >() {}
  TransformT(const Matrix3d & E, const typename Vector3dTpl<FloatType>::Type & r) : 
      internal::TransformT_helper<FloatType,RotationClass >(E,r) {}
  TransformT(const RotationClass & E, const typename Vector3dTpl<FloatType>::Type & r) : 
      internal::TransformT_helper<FloatType,RotationClass >(E,r) {}

  typename Vector6dTpl<FloatType>::Type mulMatrixTransposeBy(typename Vector6dTpl<FloatType>::Type &aF) const
  {};


  // Arithmetic operators
  TransformT operator*(FloatType a) const
  {
    RotationClass &LM_E=internal::TransformT_helper<FloatType,RotationClass>::m_E;
    Vector3dTpl<FloatType> &LM_R=internal::TransformT_helper<FloatType,RotationClass>::m_r;

    return TransformT(a*LM_E, a*LM_R);
  }

  // bXz = bXa * aXz
  /*
  TransformT operator*(const TransformT & X) const
  {
    return TransformT(LM_E*X.E(), 
                      (Vector3dTpl<FloatType>)(X.r() + X.E().transpose()*LM_R));
                      }*/


  template<class S>
  TransformT< FloatType, typename rm_mul_op<FloatType, RotationClass, S >::rm >
  operator*(const TransformT<FloatType, S> &aTsf) const
  {
    typedef typename rm_mul_op<FloatType,RotationClass,S>::rm RMResult;
    RotationClass &LM_E=internal::TransformT_helper<FloatType,RotationClass>::m_E;
    Vector3dTpl<FloatType> &LM_R=internal::TransformT_helper<FloatType,RotationClass>::m_r;

    RMResult  aR= LM_E*aTsf.E(); 
    Vector3dTpl<FloatType> aV= (Vector3dTpl<FloatType>)(aTsf.r() +  aTsf.E().transpose()*LM_R);
    return TransformT<FloatType, RMResult>(aR,aV);
  }

};



template <typename FloatType> 
class TransformT<FloatType, RotationMatrixAboutXTpl<FloatType> >: 
public internal::TransformT_helper<FloatType, RotationMatrixAboutXTpl<FloatType> >
{
  EIGEN_METAPOD_TYPEDEFS;
  EIGEN_METAPOD_CM_TYPEDEFS;
  METAPOD_SPATIAL_ROTATION_TYPEDEFS;
 public:
  TransformT(): internal::TransformT_helper<FloatType,RotationMatrixAboutX >() {}
  TransformT(const Matrix3d & E, const typename Vector3dTpl<FloatType>::Type & r) : 
      internal::TransformT_helper<FloatType,RotationMatrixAboutX >(E,r) {}
  TransformT(const RotationMatrixAboutX & E, const typename Vector3dTpl<FloatType>::Type & r) : 
      internal::TransformT_helper<FloatType,RotationMatrixAboutX >(E,r) {}

  typename Vector6dTpl<FloatType>::Type mulMatrixTransposeBy(typename Vector6dTpl<FloatType>::Type &aF) const
  {
    EIGEN_METAPOD_TYPEDEFS;
    Vector6d M;
    const RotationMatrixAboutX &lE = internal::TransformT_helper<FloatType,RotationMatrixAboutX >::m_E;
    const typename Vector3dTpl<FloatType>::Type &lr = internal::TransformT_helper<FloatType,RotationMatrixAboutX >::m_r;

    M[0] = aF(0);
    M[1] = lE.m_c*aF(1) - lE.m_s*aF(2);
    M[2] = lE.m_s*aF(1) + lE.m_c*aF(2);
        
    M[0] += (-lE.m_c*lr(2) + lE.m_s*lr(1)) * aF(4) + 
        (lE.m_s*lr(2) + lE.m_c*lr(1)) * aF(5) ;
        
    M[1] += lr(2) * aF(3) 
        - lE.m_s*lr(0) * aF(4) 
        - lE.m_c*lr(0) * aF(5) ;
        
    M[2] += -lr(1)* aF(3) + 
        lE.m_c*lr(0) * aF(4) 
        -lE.m_s*lr(0) * aF(5);
        
    M[3] = aF(3) ;
    M[4] = lE.m_c*aF(4) - lE.m_s*aF(5);
    M[5] = lE.m_s*aF(4) + lE.m_c*aF(5);
        
    return M;
  }



};

template <typename FloatType> 
class TransformT<FloatType, RotationMatrixAboutYTpl<FloatType> >: 
      public internal::TransformT_helper<FloatType, RotationMatrixAboutYTpl<FloatType> >
{
  EIGEN_METAPOD_TYPEDEFS;
  EIGEN_METAPOD_CM_TYPEDEFS;
  METAPOD_SPATIAL_ROTATION_TYPEDEFS;
 public:
  TransformT(): internal::TransformT_helper<FloatType,RotationMatrixAboutY >() {}
  TransformT(const Matrix3d & E, const typename Vector3dTpl<FloatType>::Type & r) : 
      internal::TransformT_helper<FloatType,RotationMatrixAboutY >(E,r) {}
  TransformT(const RotationMatrixAboutY & E, const typename Vector3dTpl<FloatType>::Type & r) : 
      internal::TransformT_helper<FloatType,RotationMatrixAboutY >(E,r) {}

  typename Vector6dTpl<FloatType>::Type mulMatrixTransposeBy(typename Vector6dTpl<FloatType>::Type &aF) const
  {
    Vector6d M;
    const RotationMatrixAboutY &lE = internal::TransformT_helper<FloatType,RotationMatrixAboutY >::m_E;
    const typename Vector3dTpl<FloatType>::Type &lr = internal::TransformT_helper<FloatType,RotationMatrixAboutY >::m_r;

    M[0] = lE.m_c*aF(0) + lE.m_s*aF(2);
    M[1] = aF(1);
    M[2] = -lE.m_s*aF(0) + lE.m_c*aF(2);
    
    M[0] += -lr(2) * aF(4) 
        - lE.m_s*lr(1) * aF(3) 
        + lE.m_c*lr(1) * aF(5) ;

    M[1] += (lE.m_c*lr(2) + lE.m_s*lr(0)) * aF(3) + 
        (lE.m_s*lr(2) - lE.m_c*lr(0)) * aF(5) ;

    M[2] += lr(0)* aF(4) + 
        -lE.m_c*lr(1) * aF(3) 
        -lE.m_s*lr(1) * aF(5);
          
    M[3] = lE.m_c*aF(3) + lE.m_s*aF(5) ;
    M[4] = aF(4);
    M[5] =-lE.m_s*aF(3) + lE.m_c*aF(5);
          
    return M;
  }

};


template <typename FloatType> 
class TransformT<FloatType, RotationMatrixAboutZTpl<FloatType> >: 
      public internal::TransformT_helper<FloatType, RotationMatrixAboutZTpl<FloatType> >
{
  EIGEN_METAPOD_TYPEDEFS;
  EIGEN_METAPOD_CM_TYPEDEFS;
  METAPOD_SPATIAL_ROTATION_TYPEDEFS;
 public:
  TransformT(): internal::TransformT_helper<FloatType,RotationMatrixAboutZ >() {}
  TransformT(const Matrix3d & E, const typename Vector3dTpl<FloatType>::Type & r) : 
      internal::TransformT_helper<FloatType,RotationMatrixAboutZ >(E,r) {}
  TransformT(const RotationMatrixAboutZ & E, const typename Vector3dTpl<FloatType>::Type & r) : 
      internal::TransformT_helper<FloatType,RotationMatrixAboutZ >(E,r) {}

  Vector6d  mulMatrixTransposeBy(Vector6d &aF) const
  {
    Vector6d M;
    const RotationMatrixAboutZ &lE = internal::TransformT_helper<FloatType,RotationMatrixAboutZ >::m_E;
    const typename Vector3dTpl<FloatType>::Type &lr = internal::TransformT_helper<FloatType,RotationMatrixAboutZ >::m_r;

    M[0] = lE.m_c*aF(0) - lE.m_s*aF(1);
    M[1] = lE.m_s*aF(0) + lE.m_c*aF(1);
    M[2] = aF(2);
    
    M[0] += - lr(2) * lE.m_s * aF(3)
        - lr(2) * lE.m_c*aF(4) 
        + lr(1) * aF(5) ;
    
    M[1] += lE.m_c*lr(2) * aF(3) 
        - lE.m_s*lr(2) * aF(4)
        - lr(0)* aF(5) ;
    
    M[2] += (-lE.m_c*lr(1) + lE.m_s*lr(0)) * aF(3) +
      (lE.m_s*lr(1) + lE.m_c*lr(0)) * aF(4) ;
    
    M[3] = lE.m_c*aF(3) - lE.m_s*aF(4);
    M[4] = lE.m_s*aF(3) + lE.m_c*aF(4);
    M[5] = aF(5);
          
    return M;
  }


};

template <typename FloatType> 
class TransformT<FloatType, RotationMatrixTpl<FloatType> >: 
      public internal::TransformT_helper<FloatType, RotationMatrixTpl<FloatType> >
{
  EIGEN_METAPOD_TYPEDEFS;
  EIGEN_METAPOD_CM_TYPEDEFS;
  METAPOD_SPATIAL_ROTATION_TYPEDEFS;
 public:
  TransformT(): internal::TransformT_helper<FloatType,RotationMatrix >() {}
  TransformT(const Matrix3d & E, const typename Vector3dTpl<FloatType>::Type & r) : 
      internal::TransformT_helper<FloatType,RotationMatrix >(E,r) {}
  TransformT(const RotationMatrix & E, const typename Vector3dTpl<FloatType>::Type & r) : 
      internal::TransformT_helper<FloatType,RotationMatrix >(E,r) {}

  Vector6d  mulMatrixTransposeBy(Vector6d &aF) const
  {
    Vector6d M;
    const RotationMatrix &lE = internal::TransformT_helper<FloatType,RotationMatrix >::m_E;
    const typename Vector3dTpl<FloatType>::Type &lr = internal::TransformT_helper<FloatType,RotationMatrix >::m_r;
    
    M[0] = lE(0,0)*aF(0) + lE(1,0)*aF(1) + lE(2,0)*aF(2);
    M[1] = lE(0,1)*aF(0) + lE(1,1)*aF(1) + lE(2,1)*aF(2);
    M[2] = lE(0,2)*aF(0) + lE(1,2)*aF(1) + lE(2,2)*aF(2);
    
    M[0] += (-lE(0,1)*lr(2) + lE(0,2)*lr(1))* aF(3) + 
        (-lE(1,1)*lr(2) + lE(1,2)*lr(1)) * aF(4) + 
        (-lE(2,1)*lr(2) + lE(2,2)*lr(1)) * aF(5) ;
    
    M[1] += ( lE(0,0)*lr(2) - lE(0,2)*lr(0))* aF(3) + 
        ( lE(1,0)*lr(2) - lE(1,2)*lr(0)) * aF(4) + 
        ( lE(2,0)*lr(2) - lE(2,2)*lr(0)) * aF(5) ;
    
    M[2] += (-lE(0,0)*lr(1) + lE(0,1)*lr(0))* aF(3) + 
        (-lE(1,0)*lr(1) + lE(1,1)*lr(0)) * aF(4) + 
        (-lE(2,0)*lr(1) + lE(2,1)*lr(0)) * aF(5);
      
    M[3] = lE(0,0)*aF(3) + lE(1,0)*aF(4) + lE(2,0)*aF(5);
    M[4] = lE(0,1)*aF(3) + lE(1,1)*aF(4) + lE(2,1)*aF(5);
    M[5] = lE(0,2)*aF(3) + lE(1,2)*aF(4) + lE(2,2)*aF(5);
    
    return M;
  }

  /// Why are those methods necessary ?
  /// TODO: Remove them, they should be handle by TransformT_helper.
  TransformT<FloatType, RotationMatrix> inverse() const
  {
    const RotationMatrix &lE = internal::TransformT_helper<FloatType,RotationMatrix >::m_E;
    const typename Vector3dTpl<FloatType>::Type &lr = internal::TransformT_helper<FloatType,RotationMatrix >::m_r;

    return TransformT<FloatType, RotationMatrix>(lE.transpose(), -(lE*lr));
  }

  
  TransformT<FloatType, RotationMatrix> toPointFrame(const typename Vector3dTpl<FloatType>::Type& p) const
  {
    const RotationMatrix &lE = internal::TransformT_helper<FloatType,RotationMatrix >::m_E;
    const typename Vector3dTpl<FloatType>::Type &lr = internal::TransformT_helper<FloatType,RotationMatrix >::m_r;

    return TransformT<FloatType,RotationMatrix> (lE, lr + lE.transpose()*p);
  }

};


#define EIGEN_METAPOD_TRANSFORM_TYPEDEFS                                \
  typedef class Spatial::TransformT<FloatType, Spatial::RotationMatrixTpl<FloatType> > Transform; \
  typedef class Spatial::TransformT<FloatType, Spatial::RotationMatrixAboutXTpl<FloatType> > TransformX; \
  typedef class Spatial::TransformT<FloatType, Spatial::RotationMatrixAboutYTpl<FloatType> > TransformY; \
  typedef class Spatial::TransformT<FloatType, Spatial::RotationMatrixAboutZTpl<FloatType> > TransformZ; \
  typedef class Spatial::TransformT<FloatType, Spatial::RotationMatrixIdentityTpl<FloatType> > TransformId
  
} // end of namespace Spatial


} // end of namespace metapod

# endif
