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
# include <fstream>

namespace metapod
{
  namespace benchmark
  {
    // Generate the sample robot library header.
    void makeLibraryHeader(std::ofstream & lib_hh, int nbdof);

    // Add information about each file in its header.
    void addHeader(std::ofstream & body_hh,
                   std::ofstream & joint_hh,
                   std::ofstream & robot_hh,
                   std::ofstream & lib_cc,
                   int nbdof);

    // Add the License header.
    void addLicense(std::ofstream & of);

    // Build the robot structure bu adding Nodes to form a binary tree.
    void buildTree(std::ofstream & robot_hh,
                   std::ofstream & joint_hh,
                   std::ofstream & body_hh,
                   std::ofstream & lib_cc,
                   const std::string & libname,
                   int* label,
                   int max_depth,
                   int depth = 1);

    // Add a Node (Body, Joint) with random parameters.
    void addNode(std::ofstream & body_hh,
                 std::ofstream & joint_hh,
                 std::ofstream & lib_cc,
                 const std::string & libname,
                 int label, int parent_label);

    // Generate a model of the desired depth, structured as a binary tree.
    // The values of the parameters (mass, joint axis...) are random.
    // A Robot model is made of 5 files :
    // - robot.hh declares the Robot class and defines the tree structure of
    //   the robot. It contains the global parameters of the robot.
    // - joint.hh contains the declaration of the joints od the robot.
    // - body.hh contains the declaration of the bodies od the robot.
    // - a library header file (sample_<nbdof>_dof.hh)
    // - a cc file (sample_<nbdof>_dof.cc) to be compiled and linked as an
    //   external library to provide the algorithms specialized for the model.
    void generate_model(const std::string & name, int depth);
  } // end of namespace benchmark
} // end of namespace metapod

#endif
