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
 * This file is part of a simple humanoid robot model, used for test purpose.
 * It defines the tree structure of the robot.
 */

#ifndef METAPOD_SIMPLE_HUMANOID_ROBOT_HH
# define METAPOD_SIMPLE_HUMANOID_ROBOT_HH

# include <metapod/models/simple_humanoid/config.hh>
# include <metapod/tools/common.hh>
# include <metapod/models/simple_humanoid/joint.hh>
# include <metapod/models/simple_humanoid/body.hh>

namespace metapod
{
  namespace simple_humanoid
  {
    // Model of the robot. Contains data at the global robot level and the tree
    // of Body/Joint
    class METAPOD_SIMPLE_HUMANOID_DLLAPI Robot
    {
      public:
        // Global constants or variable of the robot
        enum { NBDOF = 35 };
        enum { NBBODIES = 30 };
        static Eigen::Matrix< FloatType, NBDOF, NBDOF > H;
        typedef Eigen::Matrix< FloatType, NBDOF, 1 > confVector;

        // Definition of the multibody tree as a type.
        typedef Node< WAIST_LINK0,
                      WAIST,
                      Node< WAIST_LINK1,
                            WAIST_P,
                            Node< WAIST_LINK2,
                                  WAIST_R,
                                  Node< WAIST_LINK3,
                                        CHEST,
                                        Node< LARM_LINK1,
                                              LARM_SHOULDER_P,
                                              Node< LARM_LINK2,
                                                    LARM_SHOULDER_R,
                                                    Node< LARM_LINK3,
                                                          LARM_SHOULDER_Y,
                                                          Node< LARM_LINK4,
                                                                LARM_ELBOW,
                                                                Node< LARM_LINK5,
                                                                      LARM_WRIST_Y,
                                                                      Node< LARM_LINK6,
                                                                            LARM_WRIST_P,
                                                                            Node< LARM_LINK7,LARM_WRIST_R >

                                                                      >
                                                                >
                                                          >
                                                    >
                                              >
                                        >,
                                        Node< RARM_LINK1,
                                              RARM_SHOULDER_P,
                                              Node< RARM_LINK2,
                                                    RARM_SHOULDER_R,
                                                    Node< RARM_LINK3,
                                                          RARM_SHOULDER_Y,
                                                          Node< RARM_LINK4,
                                                                RARM_ELBOW,
                                                                Node< RARM_LINK5,
                                                                      RARM_WRIST_Y,
                                                                      Node< RARM_LINK6,
                                                                            RARM_WRIST_P,
                                                                            Node< RARM_LINK7,RARM_WRIST_R >

                                                                      >
                                                                >
                                                          >
                                                    >
                                              >
                                        >
                                  >
                            >
                      >,
                      Node< LLEG_LINK1,
                            LLEG_HIP_R,
                            Node< LLEG_LINK2,
                                  LLEG_HIP_P,
                                  Node< LLEG_LINK3,
                                        LLEG_HIP_Y,
                                        Node< LLEG_LINK4,
                                              LLEG_KNEE,
                                              Node< LLEG_LINK5,
                                                    LLEG_ANKLE_P,
                                                    Node< LLEG_LINK6,LLEG_ANKLE_R >

                                              >
                                        >
                                  >
                            >
                      >,
                      Node< RLEG_LINK1,
                            RLEG_HIP_R,
                            Node< RLEG_LINK2,
                                  RLEG_HIP_P,
                                  Node< RLEG_LINK3,
                                        RLEG_HIP_Y,
                                        Node< RLEG_LINK4,
                                              RLEG_KNEE,
                                              Node< RLEG_LINK5,
                                                    RLEG_ANKLE_P,
                                                    Node< RLEG_LINK6,RLEG_ANKLE_R >

                                              >
                                        >
                                  >
                            >
                      >
        > Tree;
    };
  } // end of namespace simple_humanoid
} // end of namespace metapod

#endif
