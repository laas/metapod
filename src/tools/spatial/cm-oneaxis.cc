// Copyright 2013,
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
#include <metapod/config.hh>
#include <metapod/tools/spatial.hh>

namespace metapod
{
  namespace Spatial
  {
    
    // Operator Inertia = Inertia * float
    METAPOD_DLLEXPORT Vector6d operator*(const Inertia & m,
                       const ConstraintMotionAxisX &)
    {
      Vector6d r;
      r[0] = m.I()(0); r[1] = m.I()(1);r[2] = m.I()(3);
      r[3] = 0.0; r[4] = -m.h()(2); r[5] = m.h()(1);
      return r;
    }

    METAPOD_DLLEXPORT Vector6d operator*(const Inertia & m,
                       const ConstraintMotionAxisY &)
    {
      Vector6d r;
      r[0] = m.I()(1); r[1] = m.I()(2);r[2] = m.I()(4);
      r[3] = m.h()(2); r[4] = 0.0; r[5] = -m.h()(0);
      return r;
    }

    METAPOD_DLLEXPORT Vector6d operator*(const Inertia & m,
                       const ConstraintMotionAxisZ &)
    {
      Vector6d r;
      r[0] = m.I()(3); r[1] = m.I()(4);r[2] = m.I()(5);
      r[3] = -m.h()(1); r[4] = m.h()(0); r[5] = 0.0;
      return r;
    }

    template <> 
    METAPOD_DLLEXPORT const Vector6d ConstraintMotionOneAxis<AxisX>::m_S =
      lvector6dMaker( 1.0, 0.0, 0.0, 0.0 , 0.0, 0.0);

    template <>
    METAPOD_DLLEXPORT const Vector6d ConstraintMotionOneAxis<AxisY>::m_S =
      lvector6dMaker( 0.0, 1.0, 0.0, 0.0 , 0.0, 0.0);
    
    template <> 
    METAPOD_DLLEXPORT const Vector6d ConstraintMotionOneAxis<AxisZ>::m_S =
      lvector6dMaker( 0.0, 0.0, 1.0, 0.0 , 0.0, 0.0);

  }
}
