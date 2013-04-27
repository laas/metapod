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


#ifndef METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_HH
# define METAPOD_SPATIAL_ALGEBRA_ROTATION_MATRIX_HH

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/linear_congruential.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/generator_iterator.hpp>


# include "metapod/tools/fwd.hh"
# include "metapod/tools/spatial/lti.hh"

# include "metapod/tools/spatial/rm-general.hh"
# include "metapod/tools/spatial/rm-aboutxaxis.hh"
# include "metapod/tools/spatial/rm-aboutyaxis.hh"
# include "metapod/tools/spatial/rm-aboutzaxis.hh"
# include "metapod/tools/spatial/rm-identity.hh"
# include "metapod/tools/spatial/rm-chgaxis.hh"
# include "metapod/tools/spatial/rm-binaryop.hh"
# include "metapod/tools/spatial/rm-mulop.hh"

# define METAPOD_SPATIAL_ROTATION_TYPEDEFS \
  METAPOD_SPATIAL_ROTATION_MATRIX_TYPEDEF; \
  METAPOD_SPATIAL_ROTATION_MATRIX_X_TYPEDEF; \
  METAPOD_SPATIAL_ROTATION_MATRIX_Y_TYPEDEF; \
  METAPOD_SPATIAL_ROTATION_MATRIX_Z_TYPEDEF; \
  METAPOD_SPATIAL_ROTATION_MATRIX_I_TYPEDEF

#endif 
