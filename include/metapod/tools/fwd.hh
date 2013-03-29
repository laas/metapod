// Copyright 2011, 2012, 2013
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
// Sébastien Barthélémy (Aldebaran Robotics)
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

  typedef Eigen::Matrix< FloatType, 1, 1 > Vector1d;
  typedef Eigen::Matrix< FloatType, 3, 1 > Vector3d;
  typedef Eigen::Matrix< FloatType, 6, 1 > Vector6d;

  typedef Eigen::Matrix< FloatType, 1, 6 > Vector6dt;

  typedef Eigen::Matrix< FloatType, 2, 2 > Matrix2d;
  typedef Eigen::Matrix< FloatType, 3, 2 > Matrix3_2d;
  typedef Eigen::Matrix< FloatType, 3, 3 > Matrix3d;
  typedef Eigen::Matrix< FloatType, 6, 6 > Matrix6d;

  typedef Eigen::Matrix< FloatType, Eigen::Dynamic, Eigen::Dynamic > MatrixN;
  typedef Eigen::Matrix< FloatType, Eigen::Dynamic, 1 > VectorN;
  typedef Eigen::AngleAxis< FloatType > AngleAxisd;

  namespace Spatial
  {
    /// Implementation of a spatial algebra.
    /// It follows R. Featherstone guidelines, and implements :
    /// - Spatial Motion vectors (a.k.a. twists)
    /// - Spatial Force vectors (a.k.a. wrenches)
    /// - Spatial Rigid Body Inertia matrices
    /// - Spatial Transforms (a.k.a. homogeneous matrices, elements of SE(3))

    // Tool methods
    inline Matrix3d skew(const Vector3d & v)
    {
      Matrix3d m;
      m(0,0) = 0;    m(0,1) = -v(2); m(0,2) = v(1);
      m(1,0) = v(2); m(1,1) = 0    ; m(1,2) = -v(0);
      m(2,0) = -v(1);m(2,1)=  v(0) ; m(2,2) =  0 ;
      return m;
    }
  } // end of namespace Spatial

}



#endif

