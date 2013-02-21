// Copyright 2011, 2012, 2013
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
// Sébastien Barthélémy (Aldebaran Robotics)
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

#ifndef METAPOD_ROBOT_BUILDER_P_HH
# define METAPOD_ROBOT_BUILDER_P_HH

# include <metapod/robotbuilder/robotbuilder.hh>

# include <string>
# include <utility>
# include <fstream>
# include <sstream>
# include <vector>
# include <map>
# include "robotmodel.hh"

namespace metapod {

class RobotBuilderP
{
public:
  RobotBuilderP();
  ~RobotBuilderP();
  RobotBuilder::Status set_name(const std::string& name);
  RobotBuilder::Status set_libname(const std::string& libname);
  RobotBuilder::Status set_directory(const std::string& directory);
  RobotBuilder::Status set_use_dof_index(bool);
  RobotBuilder::Status set_license(const std::string& text);
  RobotBuilder::Status init();

  // R_joint_parent is the rotation matrix which converts vector from the
  // parent body frame to the joint frame coordinate systems.
  // r_parent_joint is a 3D vector giving the position of the joint frame
  // origin in the parent frame coordinate system.
  //
  // Thus if p_parent is the coordinates vector of the point p in the parent
  // frame, then p_joint is given by
  //
  //   p_joint = R_joint_parent * (p_parent - r_parent_joint)
  //
  // dof_index will only be taken into account if set_use_dof_index(true)
  // has been called. In such a case, consistent dof indexes should be provided
  // for each link.
  RobotBuilder::Status addLink(
      const std::string& parent_body_name,
      const std::string& joint_name,
      unsigned int joint_type,
      const Eigen::Matrix3d & R_joint_parent,
      const Eigen::Vector3d & r_parent_joint,
      const std::string& body_name,
      double body_mass,
      const Eigen::Vector3d & body_center_of_mass,
      const Eigen::Matrix3d & body_rotational_inertia,
      const Eigen::Vector3d & joint_axis,
      int dof_index=-1);
  RobotBuilder::Status write() const;
private:
  typedef std::map<std::string, std::string> ReplMap;
  // Tuple-like to hold the stream in which the link are written
  struct TmpStreams {
    // for content in robot.hh
    std::ostringstream node_type_definitions;
    std::ostringstream nodeid_enum_definition;
    std::ostringstream nodes_type_list;
    std::ostringstream map_node_id_to_type;
    // for content in init.cc
    std::ostringstream init_nodes;
  };
  void writeLink(int link_id, const ReplMap& replacements,
                 TmpStreams& out) const;
  void writeTemplate(const std::string& output_filename,
                     const std::string &input_template,
                     const ReplMap& replacements) const;
  RobotBuilderP(const RobotBuilderP&); // forbid copy-constuction
  int nb_dof_;
  bool is_initialized_;
  bool use_dof_index_;
  RobotModel model_;
  std::string name_;
  std::string libname_;
  std::string directory_;
  std::string license_;
};
}
#endif
