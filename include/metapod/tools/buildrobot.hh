// Copyright 2011, 2012,
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
 * This file provides tools to generate metapod robot models.
 */

#ifndef METAPOD_BUILD_ROBOT_HH
# define METAPOD_BUILD_ROBOT_HH

# include "fwd.hh"
# include <fstream>
# include <iostream>
# include <algorithm>
# include <string>

namespace metapod
{
  enum { FREE_FLYER, REVOLUTE_AXIS_X, REVOLUTE_AXIS_ANY };

  void createBody(std::ofstream & body_hh,
                  std::ofstream & init_cc,
                  const std::string & libname,
                  const std::string & name,
                  const std::string parent_name,
                  const std::string joint_name,
                  int label,
                  FloatType mass,
                  const Vector3d & CoM,
                  const Matrix3d & inertie,
                  const std::string & tab)
  {
    std::string libname_uc(libname);
    std::transform(libname.begin(), libname.end(), libname_uc.begin(), ::toupper);
    body_hh
      << tab << "CREATE_BODY("
        << libname_uc << ", "
        << name << ", "
        << parent_name << ", "
        << joint_name << ");\n";
    init_cc
      << tab << "INITIALIZE_BODY(" << name << ");\n"
      << tab << "const std::string " << name << "::name = \"" << name << "\";\n"
      << tab << "const int " << name << "::label = " << label << ";\n"
      << tab << "const FloatType " << name << "::mass = " << mass << ";\n"
      << tab << "const Vector3d " << name << "::CoM = Vector3d("
        << CoM[0] << ", "
        << CoM[1] << ", "
        << CoM[2] << ");\n"
      << tab << "const Matrix3d " << name << "::inertie = matrix3dMaker(\n"
      << tab << "  " << inertie(0,0) << ", " << inertie(0,1) << ", " << inertie(0,2) << ",\n"
      << tab << "  " << inertie(1,0) << ", " << inertie(1,1) << ", " << inertie(1,2) << ",\n"
      << tab << "  " << inertie(2,0) << ", " << inertie(2,1) << ", " << inertie(2,2) << ");\n"
      << tab << "Spatial::Inertia " << name << "::I = spatialInertiaMaker("
        << name << "::mass, "
        << name << "::CoM, "
        << name << "::inertie);\n"
      << std::endl;
  }

  void createJoint(std::ofstream & joint_hh,
                   std::ofstream & init_cc,
                   const std::string & libname,
                   int joint_type,
                   const std::string & name,
                   int label,
                   int positionInConf,
                   const Matrix3d & Xt_E,
                   const Vector3d & Xt_r,
                   const std::string & tab,
                   FloatType axis_x = 1.,
                   FloatType axis_y = 0.,
                   FloatType axis_z = 0.)
  {
    std::string libname_uc(libname);
    std::transform(libname.begin(), libname.end(), libname_uc.begin(), ::toupper);
    switch(joint_type)
    {
      case FREE_FLYER:
        joint_hh << tab << "JOINT_FREE_FLYER(" << libname_uc << ", " << name << ");\n";
        init_cc << tab << "INITIALIZE_JOINT_FREE_FLYER(" << name << ");\n";
        break;
      case REVOLUTE_AXIS_X:
        joint_hh << tab << "JOINT_REVOLUTE_AXIS_X(" << libname_uc << ", " << name << ");\n";
        init_cc << tab << "INITIALIZE_JOINT_REVOLUTE_AXIS_X(" << name << ");\n";
        break;
      case REVOLUTE_AXIS_ANY:
        joint_hh << tab << "JOINT_REVOLUTE_AXIS_ANY(" << libname_uc << ", " << name
            << ", " << axis_x << ", " << axis_y << ", " << axis_z << ");\n";
        init_cc << tab << "INITIALIZE_JOINT_REVOLUTE_AXIS_ANY(" << name
            << ", " << axis_x << ", " << axis_y << ", " << axis_z << ");\n";
        break;
    }
    init_cc
      << tab << "const std::string " << name << "::name = \"" << name << "\";\n"
      << tab << "const int " << name << "::label = " << label << ";\n"
      << tab << "const int " << name << "::positionInConf = " << positionInConf << ";\n"
      << tab << "const Spatial::Transform " << name << "::Xt = Spatial::Transform(\n"
      << tab << "  matrix3dMaker(\n"
      << tab << "    " << Xt_E(0,0) << ", " << Xt_E(0,1) << ", " << Xt_E(0,2) << ",\n"
      << tab << "    " << Xt_E(1,0) << ", " << Xt_E(1,1) << ", " << Xt_E(1,2) << ",\n"
      << tab << "    " << Xt_E(2,0) << ", " << Xt_E(2,1) << ", " << Xt_E(2,2) << "),\n"
      << tab << "  Vector3d(\n"
      << tab << "    " << Xt_r[0] << ", " << Xt_r[1] << ", " << Xt_r[2] << "));\n"
      << std::endl;
  }
} // end of namespace metapod

#endif
