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
#ifndef METAPOD_SPATIAL_ALGEBRA_ROTATION_MUL_OP_HH
# define METAPOD_SPATIAL_ALGEBRA_ROTATION_MUL_OP_HH

# include "metapod/tools/spatial/rm-general.hh"
# include "metapod/tools/spatial/rm-aboutxaxis.hh"
# include "metapod/tools/spatial/rm-aboutyaxis.hh"
# include "metapod/tools/spatial/rm-aboutzaxis.hh"
# include "metapod/tools/spatial/rm-identity.hh"
# include "metapod/tools/spatial/rm-chgaxis.hh"

namespace metapod
{

  namespace Spatial
  {

    /** This template implements the type conversion for the multiplication
        between rotation matrix. This ensure to have the more compact
        representation for a rotation matrix.
    */
    template<typename FloatType, class op1, class op2>
    struct rm_mul_op
    { METAPOD_SPATIAL_ROTATION_MATRIX_TYPEDEF;
      typedef RotationMatrix rm; };

    /** Specializations */

    /** All multiplication with the identity gives back
        the same identity.
     */
    template <typename FloatType, class op1>
    struct rm_mul_op<FloatType, op1,RotationMatrixIdentityTpl<FloatType > >
    { typedef op1 rm; };

    template <typename FloatType,class op2>
    struct rm_mul_op<FloatType,RotationMatrixIdentityTpl<FloatType>,op2>
    { typedef op2 rm; };

    template <typename FloatType>
    struct rm_mul_op<FloatType,RotationMatrixAboutXTpl<FloatType>,RotationMatrixAboutXTpl<FloatType> >
    { typedef RotationMatrixAboutXTpl<FloatType>  rm;};

    template <typename FloatType>
    struct rm_mul_op<FloatType,RotationMatrixAboutYTpl<FloatType> ,RotationMatrixAboutYTpl<FloatType> >
    { typedef RotationMatrixAboutYTpl<FloatType>  rm;};

    template <typename FloatType>
    struct rm_mul_op<FloatType, RotationMatrixAboutZTpl<FloatType> ,RotationMatrixAboutZTpl<FloatType> >
    { typedef RotationMatrixAboutZTpl<FloatType>  rm;};
      
  }
}
#endif /* METAPOD_SPATIAL_ALGEBRA_ROTATION_MUL_OP_HH */
