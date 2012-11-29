// Copyright 2011, 2012,
//
// Maxime Reis
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
// GNU Lesser General Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod.  If not, see <http://www.gnu.org/licenses/>.

/*
 * This files include the eigen library and makes the typedefs used throughout
 * the project.
 */

#ifndef METAPOD_FORWARD_HH
# define METAPOD_FORWARD_HH

# include "Eigen/Dense"

namespace metapod
{
  typedef double FloatType;

  typedef Eigen::Matrix< FloatType, 1, 1 > vector1d;
  typedef Eigen::Matrix< FloatType, 3, 1 > vector3d;
  typedef Eigen::Matrix< FloatType, 6, 1 > vector6d;

  typedef Eigen::Matrix< FloatType, 1, 6 > vector6dt;

  typedef Eigen::Matrix< FloatType, 3, 3 > matrix3d;
  typedef Eigen::Matrix< FloatType, 6, 6 > matrix6d;

  typedef Eigen::Matrix< FloatType, Eigen::Dynamic, Eigen::Dynamic > matrixN;
  typedef Eigen::Matrix< FloatType, Eigen::Dynamic, 1 > vectorN;
  typedef Eigen::AngleAxis< FloatType > AngleAxisd;

  class NC;

  namespace Spatial
  {
    /// Implementation of a spatial algebra.
    /// It follows R. Featherstone guidelines, and implements :
    /// - Spatial Motion vectors (a.k.a. twists)
    /// - Spatial Force vectors (a.k.a. wrenches)
    /// - Spatial Rigid Body Inertia matrices
    /// - Spatial Transforms (a.k.a. homogeneous matrices, elements of SE(3))

    // Tool methods
    inline matrix3d skew(const vector3d & v)
    {
      matrix3d m;
      m(0,0) = 0;    m(0,1) = -v(2); m(0,2) = v(1);
      m(1,0) = v(2); m(1,1) = 0    ; m(1,2) = -v(0);
      m(2,0) = -v(1);m(2,1)=  v(0) ; m(2,2) =  0 ;
      return m;
    }

    // Template for operator * with inertia matrix I
    // T = I * S
    template <class T, class U, class S>
    struct OperatorMul
    {
      T mul(const U &u, const S &s) const;
    };

    template <class V, 
	      class U, 
	      template <class > class W,
	      class S>
    struct OperatorMult
    {
      V mul(const U &u, const W<S> &s) const;
    };


  } // end of namespace Spatial

}



#endif

