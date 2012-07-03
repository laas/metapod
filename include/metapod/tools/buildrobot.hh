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
 * This file provides tools to generate metapod robot models.
 */

#ifndef METAPOD_BUILD_ROBOT_HH
# define METAPOD_BUILD_ROBOT_HH

# include "common.hh"

namespace metapod
{
  enum { FREE_FLYER, REVOLUTE };

  void createBody(std::ofstream & os,
                  const std::string & name,
                  const std::string parent_name,
                  const std::string joint_name,
                  int label,
                  FloatType mass,
                  const vector3d & CoM,
                  const matrix3d & inertie,
                  const std::string & tab,
                  int has_parent = true)
  {
    os << tab << "// Declaration of " << name << " class\n"
       << tab << "CREATE_BODY("
         << tab << name << ", "
         << tab << has_parent << ", "
         << tab << parent_name << ", "
         << tab << joint_name << ");\n"
       << tab << "const std::string " << name << "::name = \"" << name << "\";\n"
       << tab << "const int " << name << "::label = " << label << ";\n"
       << tab << "const FloatType " << name << "::mass = " << mass << ";\n"
       << tab << "const vector3d " << name << "::CoM = vector3d("
         << tab << CoM[0] << ", "
         << tab << CoM[1] << ", "
         << tab << CoM[2] << ");\n"
       << tab << "const matrix3d " << name << "::inertie = matrix3dMaker(\n"
       << tab << inertie(0,0) << ", " << inertie(0,1) << ", " << inertie(0,1) << ",\n"
       << tab << inertie(1,0) << ", " << inertie(1,1) << ", " << inertie(1,1) << ",\n"
       << tab << inertie(2,0) << ", " << inertie(2,1) << ", " << inertie(2,1) << ");\n"
       << tab << "Inertia " << name << "::I = spatialInertiaMaker("
         << tab << name << "::mass,\n"
         << tab << name << "::CoM,\n"
         << tab << name << "::inertie);\n"
       << std::endl;
  }

  void createJoint(std::ofstream & os,
                   int joint_type,
                   const std::string & name,
                   int label,
                   int positionInConf,
                   const matrix3d & Xt_E,
                   const vector3d & Xt_r,
                   const std::string & tab)
  {
    switch(joint_type)
    {
      case FREE_FLYER:
        os << tab << "JOINT_FREE_FLYER(" << name << ");\n";
        break;
      case REVOLUTE:
        os << tab << "JOINT_REVOLUTE(" << name << ");\n";
        break;
    }
    os << tab << "const std::string " << name << "::name = \"" << name << "\";\n"
       << tab << "const int " << name << "::label = " << label << ";\n"
       << tab << "const int " << name << "::positionInConf = " << positionInConf << ";\n"
       << tab << "const Transform " << name << "::Xt = Transform(\n"
       << tab << "  matrix3dMaker(\n"
       << tab << "    " << Xt_E(0,0) << ", " << Xt_E(0,1) << ", " << Xt_E(0,2) << ",\n"
       << tab << "    " << Xt_E(1,0) << ", " << Xt_E(1,1) << ", " << Xt_E(1,2) << ",\n"
       << tab << "    " << Xt_E(2,0) << ", " << Xt_E(2,1) << ", " << Xt_E(2,2) << "),\n"
       << tab << "  vector3d(\n"
       << tab << "    " << Xt_r[0] << ", " << Xt_r[1] << ", " << Xt_r[2] << "));\n"
       << std::endl;
  }
} // end of namespace metapod

#endif
