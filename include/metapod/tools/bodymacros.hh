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
 * This file defines the macro that creates the bodies classes
 */

#ifndef METAPOD_BODY_MACROS_HH
# define METAPOD_BODY_MACROS_HH

namespace metapod
{

  // Create a body
  #define CREATE_BODY(libname,                             \
                      classname,                           \
                      parentname,                          \
                      jointname)                           \
    class libname ## _DLLAPI classname                     \
    {                                                      \
      public:                                              \
        static const std::string name;                     \
        static const int label;                            \
        static const FloatType mass;                       \
        static const Vector3d CoM;                         \
        static const Matrix3d inertie;                     \
        static Spatial::Transform iX0;                     \
        static Spatial::Motion vi;                         \
        static Spatial::Motion ai;                         \
        static Spatial::Force Fext;                        \
        static Spatial::Inertia I;                         \
        static Spatial::Inertia Iic;                       \
        typedef parentname Parent;                         \
        typedef jointname Joint;                           \
    }                                                      \

  # define INITIALIZE_BODY(classname)                      \
    Spatial::Transform classname::iX0;                     \
    Spatial::Motion classname::vi;                         \
    Spatial::Motion classname::ai;                         \
    Spatial::Inertia classname::Iic;                       \
    Spatial::Force classname::Fext

} // end of namespace metapod

#endif
