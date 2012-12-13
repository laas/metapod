// Copyright 2012,
//
// Sébastien Barthélémy
//
// Aldebaran Robotics
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
 * This file is part of a simple arm robot model, used for test purpose.
 * It defines the tree structure of the robot.
 */

#ifndef METAPOD_SIMPLE_ARM_ROBOT_HH
# define METAPOD_SIMPLE_ARM_ROBOT_HH

# include <metapod/models/simple_arm/config.hh>
# include <metapod/tools/common.hh>
# include <metapod/models/simple_arm/joint.hh>
# include <metapod/models/simple_arm/body.hh>

namespace metapod
{
  namespace simple_arm
  {
    // Model of the robot. Contains data at the global robot level and the tree
    // of Body/Joint
    class METAPOD_SIMPLE_ARM_DLLAPI Robot
    {
      public:
        // Global constants or variable of the robot
        enum { NBDOF = 3 };
        enum { NBBODIES = 3 };
        static Eigen::Matrix< FloatType, NBDOF, NBDOF > H;
        typedef Eigen::Matrix< FloatType, NBDOF, 1 > confVector;

        // Definition of the multibody tree as a type.
        typedef Node< ARM,
                      SHOULDER,
                      Node< FOREARM,
                            ELBOW,
                            Node< HAND,
                                  WRIST >
                            >
                      > Tree;
    };
  } // end of namespace simple_arm
} // end of namespace metapod

#endif
