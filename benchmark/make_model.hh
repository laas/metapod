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
 * Build the models used in the benchmark.
 */

#ifndef METAPOD_MAKE_MODEL_HH
# define METAPOD_MAKE_MODEL_HH

# include <iostream>
# include "metapod/tools/buildrobot.hh"

namespace metapod
{
  namespace benchmark
  {
    void addNode(std::ofstream & robot_hh,
                 std::ofstream & joint_hh,
                 std::ofstream & body_hh,
                 int* label,
                 int max_depth,
                 int depth = 1);
    void addRandomNode(std::ofstream & body_hh, std::ofstream & joint_hh,
                       int label, int parent_label);

    void generate_model(const std::string & name,
                        int depth)
    {
      std::stringstream ss_body, ss_joint, ss_robot;
      ss_body << "models/" << name << "_body.hh";
      ss_joint << "models/" << name << "_joint.hh";
      ss_robot << "models/" << name << "_robot.hh";
      std::string body = ss_body.str(),
                  joint = ss_joint.str(),
                  robot = ss_robot.str();
      std::ofstream body_hh(body.c_str()),
                    joint_hh(joint.c_str()),
                    robot_hh(robot.c_str());

      int NBDOF = pow(2,depth+1) - 1;

      joint_hh << "# include \"metapod/tools/jointmacros.hh\"\n\n"
               << "namespace " << name << "\n{\n"
               << "using namespace metapod;\n"
               << "using namespace metapod::Spatial;\n";
      body_hh << "# include \"metapod/tools/bodymacros.hh\"\n\n"
              << "namespace " << name << "\n{\n"
              << "  using namespace metapod;\n"
              << "  using namespace metapod::Spatial;\n";
      robot_hh << "# include \"metapod/tools/common.hh\"\n"
               << "# include \"" << name << "_joint.hh\"\n"
               << "# include \"" << name << "_body.hh\"\n\n"
               << "namespace " << name << "\n"
               << "{\n"
               << "  class Robot\n"
               << "  {\n"
               << "    public:\n"
               << "      enum { NBDOF = " << NBDOF << " };\n"
               << "      static Eigen::Matrix< FloatType, NBDOF, NBDOF > H;\n"
               << "      typedef Eigen::Matrix< FloatType, NBDOF, 1 > confVector;\n";
          
      createJoint(joint_hh, REVOLUTE, "J0", 0, 0,
                  matrix3d::Random(), 
                  vector3d::Random(), "  ");

      createBody(body_hh, "B0", "NP", "J0", 0, 1.,
                 vector3d::Random(),
                 matrix3d::Random(),
                 "  ",
                 false);

      robot_hh << "      typedef Node< B0,\n"
               << "                    J0,\n";
      int label = 0;

      addNode(robot_hh, joint_hh, body_hh, &label, depth);

      joint_hh << "} // end of namespace " << name;
      body_hh << "} // end of namespace " << name;
      robot_hh << "                  > Tree;\n"
               << "  };\n"
               << "  Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF >"
                 << " Robot::H;\n"
               << "} // end of namespace " << name;
    }

    void addNode(std::ofstream & robot_hh,
                 std::ofstream & joint_hh,
                 std::ofstream & body_hh,
                 int* label,
                 int max_depth,
                 int depth)
    {
      int parent_label = *label;
      std::stringstream tab;
      tab << "              ";
      for(int i=0; i<depth; i++)
        tab << "      ";
      if(depth < max_depth)
      {
        for(int i=0; i<2; i++)
        {
          *label = *label+1;
          addRandomNode(body_hh, joint_hh, *label, parent_label);
          robot_hh << tab.str() << "Node< B" << *label << ",\n"
                   << tab.str() << "      J" << *label << ",\n";
          addNode(robot_hh, joint_hh, body_hh, label, max_depth, depth+1);
          robot_hh << tab.str() << "    >" << (i==0?",":"") << "\n";
        }
      }
      else
      {
        *label = *label+1;
        addRandomNode(body_hh, joint_hh, *label, parent_label);
        robot_hh << tab.str() << "Node< B" << *label << ", J" << *label << ">,\n";
        *label = *label+1;
        addRandomNode(body_hh, joint_hh, *label, parent_label);
        robot_hh << tab.str() << "Node< B" << *label << ", J" << *label << ">\n";
      }  
    }

    void addRandomNode(std::ofstream & body_hh, std::ofstream & joint_hh,
                       int label, int parent_label)
    {
      std::stringstream name, parent_name, joint_name;
      name << "B" << label;
      parent_name << "B" << parent_label;
      joint_name << "J" << label;
      createBody(body_hh, name.str(), parent_name.str(), joint_name.str(),
                 label, 1., vector3d::Random(), matrix3d::Random(), "  ", true);
      createJoint(joint_hh, REVOLUTE, joint_name.str(), label, label,
                  matrix3d::Random(), vector3d::Random(), "  ");
    }
  } // end of namespace benchmark
} // end of namespace metapod

#endif
