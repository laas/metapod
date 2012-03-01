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
 * This file contains the includes and class definitions necessary for the whole project.
 */

#ifndef metapod_COMMON_HH
# define metapod_COMMON_HH

# include "metapod/tools/fwd.hh"
# include "metapod/tools/jointmacros.hh"
# include "metapod/tools/smallmatrixmacros.hh"
# include "metapod/tools/spatial.hh"
# include <fstream>
# include <iostream>
# include <vector>

namespace metapod
{
  using namespace Spatial;

  #define GRAVITY_CST 9.81
  
  // Class No-Joint. Necessary to set the "joint" of the NP (No Parent) class, which is used to set the Parent of the base link.
  NO_JOINT(NJ);
  Force NJ::f;

  // Class No-Child. Necessary to end the recursion on the multibody tree.
  class NC
  {
    public:
      enum { isNode = 0 };
  };

  // Class No-Parent. Used to set the parent body of the freeflyer.
  class NP
  {
    public:
      enum { HAS_PARENT = 0 };
      static const std::string label;
      static PluckerTransform iX0;
      static Velocity vi; 
      static Acceleration ai; 
      static Force Fext;
      static Inertia I;
      typedef NJ Joint;
  };

  // Initialization of the NP class
  Acceleration NP::ai;
  PluckerTransform NP::iX0;

  // Base class for Nodes. Provides the boolean "isNode", used to end recursions on the tree structure of the robot.
  class NodeBase
  {
    public:
      enum { isNode = 1 };
  };

  // Class Node. Contains a Body, a Joint, and up to 3 Node children. Non-existant children make use of the NC class (No-Child).
  template< typename B,        // Body
            typename J,        // Joint
            typename C1 = NC,  // Children nodes
            typename C2 = NC,
            typename C3 = NC >
  class Node : public NodeBase
  {
    public:
      typedef B Body;
      typedef J Joint;
      typedef C1 Child1;
      typedef C2 Child2;
      typedef C3 Child3;
  };

  // Constant 3x3 matrix initialization method.
  const matrix3d matrix3dMaker(double v1, double v2, double v3, 
                               double v4, double v5, double v6, 
                               double v7, double v8, double v9) 
  {
    matrix3d m;
    m << v1, v2, v3, 
         v4, v5, v6, 
         v7, v8, v9; 
    return m;
  }
  
  // Constant size 3 vector initialization method.
  const vector6d vector6dMaker(double v1,
                               double v2,
                               double v3,
                               double v4,
                               double v5,
                               double v6)
  {
    vector6d v;
    v << v1, v2, v3, v4, v5, v6;
    return v;
  }

  // Constant Spatial::Inertia initialization method.
  Spatial::Inertia spatialInertiaMaker(const matrix3d & inertie, const vector3d & CoM, const double mass)
  {
    matrix3d I = inertie;
    matrix3d tmp; SKEW(CoM,tmp);
    tmp = tmp*tmp.transpose();
    I = I + tmp*mass;
    vector3d h = CoM*mass;
    return Inertia(I,h,mass);
  }

} // end of namespace metapod.

#endif
