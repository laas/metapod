// Copyright 2012,
//
// Olivier STASSE
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


#ifndef METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_HH
# define METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_HH

# include "metapod/tools/fwd.hh"

#include "metapod/tools/spatial/cm-oneaxis.hh"
#include "metapod/tools/spatial/cm-freeflyer.hh"
#include "metapod/tools/spatial/cm-anyaxis.hh"
#define EIGEN_METAPOD_CM_TYPEDEFS  \
  typedef class Spatial::ConstraintMotionAxisXTpl<FloatType>::Type ConstraintMotionAxisX; \
  typedef class Spatial::ConstraintMotionAxisYTpl<FloatType>::Type ConstraintMotionAxisY; \
  typedef class Spatial::ConstraintMotionAxisZTpl<FloatType>::Type ConstraintMotionAxisZ; \
  typedef class Spatial::ConstraintMotionFreeFlyerTpl<FloatType> ConstraintMotionFreeFlyer

#endif /* METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_HH */
