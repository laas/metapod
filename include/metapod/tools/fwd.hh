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

  typedef Eigen::Matrix< FloatType, 3, 3 > matrix3d;
  typedef Eigen::Matrix< FloatType, 6, 6 > matrix6d;

  typedef Eigen::Matrix< FloatType, Eigen::Dynamic, Eigen::Dynamic > matrixN;
  typedef Eigen::Matrix< FloatType, Eigen::Dynamic, 1 > vectorN;
  typedef Eigen::AngleAxis< FloatType > AngleAxisd;

  class NC;
}

#endif
