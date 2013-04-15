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

/*
 * Build the models used in the benchmark.
 */

#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <cstdlib>
#include <metapod/robotbuilder/robotbuilder.hh>

Eigen::Matrix3d randomRotation3d()
{
  Eigen::Vector3d v=Eigen::Vector3d::Random();
  double ct,st,cphi,sphi,cpsi,spsi;

  ct = cos(v[0]); st = sin(v[0]);
  cphi = cos(v[1]); sphi = sin(v[1]);
  cpsi = cos(v[2]); spsi = sin(v[2]);

  Eigen::Matrix3d r;
  r(0,0)=ct*cpsi; r(0,1) = -cphi*spsi+sphi*st*cpsi; r(0,2) = sphi*spsi+cphi*st*cpsi;
  r(1,0)=ct*spsi; r(1,1) = cphi*cpsi+sphi*st*spsi;  r(1,2) = -sphi*cpsi+cphi*st*spsi;
  r(2,0)=-st;     r(2,1) = sphi*ct;                 r(2,2) = cphi*ct;
  return r;
}

void addNode(metapod::RobotBuilder & builder, int label, int parent_label)
{
  std::stringstream body_name, joint_name, parent_body_name;
  // special case for the root link
  if (label == 0)
    parent_body_name << "NP";
  else
    parent_body_name << "B" << parent_label;
  body_name << "B" << label;
  joint_name << "J" << label;
  builder.addLink(parent_body_name.str(),
                  joint_name.str(), metapod::RobotBuilder::REVOLUTE_AXIS_X,
                  randomRotation3d(), Eigen::Vector3d::Random(),
                  body_name.str(), 1.,
                  Eigen::Vector3d::Random(), Eigen::Matrix3d::Random());
}

void buildTree(metapod::RobotBuilder & builder, int* label, int max_depth,
               int depth)
{
  int parent_label = *label;
  for(int i=0; i<2; ++i)
  {
    ++(*label);
    addNode(builder, *label, parent_label);
    if(depth < max_depth)
    {
      buildTree(builder, label, max_depth, depth+1);
    }
  }
}

void generate_model(const std::string & name, int depth)
{
  metapod::RobotBuilder builder;
  builder.set_name(name);
  builder.set_libname(name);
  std::stringstream ss_path, ss_namespace, ss_guard;
  ss_path << "models/" << name;
  builder.set_directory(ss_path.str());
  ss_namespace << "metapod::" << name;
  // add root joint and body
  int label = 0;
  addNode(builder, label, 0);
  // add children
  buildTree(builder, &label, depth, 1);
  builder.write();
}

int main(int argc, char** argv)
{
  const char usage[] = "Usage: make_model depth\n"
      "generate source code for a model with an equilibrated binary "
      "kinematic tree of depth `depth` in subdirectory "
       "models/sample_${depth}\n";

  if (argc < 2)
  {
    std::cerr << usage;
    return EXIT_FAILURE;
  }
  int depth = atoi(argv[1]);
  if (depth <= 0)
  {
    std::cerr << usage;
    return EXIT_FAILURE;
  }
  std::stringstream ss_name;
  ss_name << "sample_" << depth;
  generate_model(ss_name.str(), depth);
  return EXIT_SUCCESS;
}
