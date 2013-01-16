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

#include "robotbuilder_p.hh"

namespace metapod {

RobotBuilder::RobotBuilder()
  : pimpl_(new RobotBuilderP())
{}

// For pimpl, we need the destructor to be defined (even empty) here (ie.after
// the declaration of the private implementation).
// Note that a compiler-generated destructor would not be.
RobotBuilder::~RobotBuilder()
{}

RobotBuilder::Status RobotBuilder::set_directory(const std::string & directory)
{
  return pimpl_->set_directory(directory);
}

RobotBuilder::Status RobotBuilder::set_name(const std::string & name)
{
  return pimpl_->set_name(name);
}

RobotBuilder::Status RobotBuilder::set_libname(const std::string & libname)
{
  return pimpl_->set_libname(libname);
}

RobotBuilder::Status RobotBuilder::set_use_dof_index(bool flag)
{
  return pimpl_->set_use_dof_index(flag);
}

RobotBuilder::Status RobotBuilder::set_license(const std::string& text)
{
  return pimpl_->set_license(text);
}

RobotBuilder::Status RobotBuilder::addLink(
    const std::string& parent_body_name,
    const std::string& joint_name,
    unsigned int joint_type,
    const Eigen::Matrix3d & joint_Xt_E,
    const Eigen::Vector3d & joint_Xt_r,
    const std::string& body_name,
    double body_mass,
    const Eigen::Vector3d & body_center_of_mass,
    const Eigen::Matrix3d & body_rotational_inertia,
    const Eigen::Vector3d & joint_axis,
    int dof_index)
{
  return pimpl_->addLink(
    parent_body_name,
    joint_name,
    joint_type,
    joint_Xt_E,
    joint_Xt_r,
    body_name,
    body_mass,
    body_center_of_mass,
    body_rotational_inertia,
    joint_axis,
    dof_index);
}

RobotBuilder::Status RobotBuilder::write()
{
  return pimpl_->write();
}

Eigen::Vector3d RobotBuilder::axisX()
{
  Eigen::Vector3d v = Eigen::Vector3d::Zero();
  v[0] = 1.;
  return v;
}
}
