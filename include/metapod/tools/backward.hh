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

#ifndef METAPOD_BACKWARD_HH
# define METAPOD_BACKWARD_HH

namespace metapod
{
  namespace Spatial
  {

    template<typename FloatType>
    inline MotionTpl<FloatType> sum(const MotionTpl<FloatType> & mv1,
                                    const MotionTpl<FloatType> & mv2,
                                    const MotionTpl<FloatType> & mv3,
                                    const MotionTpl<FloatType> & mv4)
    {
      return MotionTpl<FloatType>(mv1.w() + mv2.w() + mv3.w() + mv4.w(),
                    mv1.v() + mv2.v() + mv3.v() + mv4.v());
    }

    template <typename FloatType>
    inline MotionTpl<FloatType> sum(const MotionTpl<FloatType> & mv1,
                                    const MotionTpl<FloatType> & mv2,
                                    const MotionTpl<FloatType> & mv3)
    {
      return MotionTpl<FloatType>(mv1.w() + mv2.w() + mv3.w(),
                                  mv1.v() + mv2.v() + mv3.v());
    }

    template <typename FloatType>
    inline ForceTpl<FloatType> sum(const ForceTpl<FloatType> & fv1,
                                   const ForceTpl<FloatType> & fv2,
                                   const ForceTpl<FloatType> & fv3)
    {
      return ForceTpl<FloatType>(fv1.n() + fv2.n() + fv3.n(),
                              fv1.f() + fv2.f() + fv3.f());
    }
  } // end of namespace Spatial

}

#endif
