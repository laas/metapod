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


#ifndef METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_ONEAXIS_HH
# define METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_ONEAXIS_HH

namespace metapod
{

  namespace Spatial
  {
    
    /* Operator Inertia = Inertia * float */
    /* */
    vector6d operator*(const Inertia & m,
		       const ConstraintMotionOneAxisAxisX &) 
    {
     vector6d r;
      r[0] = m.I()(0,0); r[1] = m.I()(1,0);r[2] = m.I()(2,0);      
      r[3] = 0.0; r[4] = -m.h()(2); r[5] = m.h()(1);
      return r;
    }

    vector6d operator*(const Inertia & m,
		       const ConstraintMotionOneAxisAxisY &) 
    {
     vector6d r;
      r[0] = m.I()(0,1); r[1] = m.I()(1,1);r[2] = m.I()(2,1);      
      r[3] = m.h()(2); r[4] = 0.0; r[5] = -m.h()(0);
      return r;
    }

    vector6d operator*(const Inertia & m,
		       const ConstraintMotionOneAxisAxisZ &) 
    {
     vector6d r;
      r[0] = m.I()(0,2); r[1] = m.I()(1,2);r[2] = m.I()(2,2);      
      r[3] = -m.h()(1); r[4] = m.h()(0); r[5] = 0.0;
      return r;
    }
    

  }
}
#endif 
