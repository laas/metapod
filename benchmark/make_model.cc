// Copyright 2011, 2012, 2013
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

#include "make_model.hh"
# include <algorithm>
# include <string>
# include <metapod/tools/buildrobot.hh>
# include <boost/filesystem/operations.hpp>

int main()
{
  const int MAX_DEPTH = 5; // NBDOF = (depth+1)² - 1;
  for(int i=1; i<=MAX_DEPTH; i++)
  {
    int NBDOF = static_cast<int>(pow(2.0,i+1)) - 1;
    std::stringstream ss_name;
    ss_name << "sample_" << NBDOF << "_dof";
    metapod::benchmark::generate_model(ss_name.str(), i);
  }
}

void metapod::benchmark::generate_model(const std::string & name, int depth)
{
  int NBDOF = static_cast<int>(pow(2.0,depth+1)) - 1;

  std::stringstream ss_body, ss_joint, ss_robot, ss_lib_cc, ss_lib_hh;
  std::stringstream ss_path;
  ss_path << "models/" << name;
  boost::filesystem::create_directory("models");
  boost::filesystem::create_directory(ss_path.str());

  ss_body << ss_path.str() << "/body.hh";
  ss_joint << ss_path.str() << "/joint.hh";
  ss_robot << ss_path.str() << "/robot.hh";
  ss_lib_hh << ss_path.str() << "/sample_" << NBDOF << "_dof.hh";
  ss_lib_cc << ss_path.str() << "/sample_" << NBDOF << "_dof.cc";
  std::ofstream body_hh(ss_body.str().c_str(), std::ofstream::out),
                joint_hh(ss_joint.str().c_str(), std::ofstream::out),
                robot_hh(ss_robot.str().c_str(), std::ofstream::out),
                lib_hh(ss_lib_hh.str().c_str(), std::ofstream::out),
                lib_cc(ss_lib_cc.str().c_str(), std::ofstream::out);

  makeLibraryHeader(lib_hh, NBDOF);
  addHeader(body_hh, joint_hh, robot_hh, lib_cc, NBDOF);

  joint_hh
    << "#ifndef METAPOD_SAMPLE_" << NBDOF << "_DOF_JOINT_HH\n"
    << "# define METAPOD_SAMPLE_" << NBDOF << "_DOF_JOINT_HH\n\n"
    << "# include \"metapod/tools/jointmacros.hh\"\n\n"
    << "namespace metapod\n"
    << "{\n"
    << "  namespace " << name << "\n"
    << "  {\n";
  body_hh
    << "#ifndef METAPOD_SAMPLE_" << NBDOF << "_DOF_BODY_HH\n"
    << "# define METAPOD_SAMPLE_" << NBDOF << "_DOF_BODY_HH\n\n"
    << "# include \"metapod/tools/bodymacros.hh\"\n\n"
    << "namespace metapod\n"
    << "{\n"
    << "  namespace " << name << "\n"
    << "  {\n";

  std::string libname_uc(name);
  std::transform(name.begin(), name.end(),
      libname_uc.begin(), ::toupper);
  robot_hh
    << "#ifndef METAPOD_SAMPLE_" << NBDOF << "_DOF_ROBOT_HH\n"
    << "# define METAPOD_SAMPLE_" << NBDOF << "_DOF_ROBOT_HH\n\n"
    << "# include \"config.hh\"\n"
    << "# ifdef " << name << "_EXPORTS\n"
    << "#   define metapod_EXPORTS\n"
    << "# endif\n"
    << "# include \"metapod/tools/common.hh\"\n"
    << "# include \"joint.hh\"\n"
    << "# include \"body.hh\"\n"
    << std::endl
    << "namespace metapod\n"
    << "{\n"
    << "  namespace " << name << "\n"
    << "  {\n"
    << "    class " << libname_uc << "_DLLAPI Robot\n"
    << "    {\n"
    << "      public:\n"
    << "        enum { NBDOF = " << NBDOF << " };\n"
    << "        enum { NBBODIES = " << NBDOF << " };\n"
    << "        static Eigen::Matrix< FloatType, NBDOF, NBDOF > H;\n"
    << "        typedef Eigen::Matrix< FloatType, NBDOF, 1 > confVector;\n";
  lib_cc
    << "# include \"sample_" << NBDOF << "_dof.hh\"\n"
    << "\n"
    << "namespace metapod\n"
    << "{\n"
    << "  namespace sample_" << NBDOF << "_dof\n"
    << "  {\n"
    << "    Eigen::Matrix< FloatType, Robot::NBDOF, Robot::NBDOF > Robot::H;\n\n";

  const std::string tab = "    ";
  // Create Root Joint
  createJoint(joint_hh, lib_cc, name.c_str(), REVOLUTE_AXIS_X, "J0", 0, 0,
              Matrix3d::Random(),
              Vector3d::Random(), tab);

  // Create Root Body
  createBody(body_hh, lib_cc, name.c_str(), "B0", "NP", "J0", 0, 1.,
             Vector3d::Random(),
             Matrix3d::Random(),
             tab);

  robot_hh << "        typedef Node< B0,\n"
           << "                      J0,\n";
  int label = 0;

  buildTree(robot_hh, joint_hh, body_hh, lib_cc, name, &label, depth);

  joint_hh
    << "  } // end of namespace " << name << std::endl
    << "} // end of namespace metapod\n\n"
    << "#endif";
  body_hh
    << "  } // end of namespace " << name << std::endl
    << "} // end of namespace metapod\n\n"
    << "#endif";
  robot_hh
    << "                    > Tree;\n"
    << "    };\n"
    << "  } // end of namespace " << name << std::endl
    << "} // end of namespace metapod\n\n"
    << "#endif";
  lib_cc
    << "  } // end of namespace " << name << std::endl
    << "} // end of namespace metapod";
}

void metapod::benchmark::buildTree(std::ofstream & robot_hh,
             std::ofstream & joint_hh,
             std::ofstream & body_hh,
             std::ofstream & lib_cc,
             const std::string & libname,
             int* label,
             int max_depth,
             int depth)
{
  int parent_label = *label;
  std::stringstream tab;
  tab << "                ";
  for(int i=0; i<depth; i++)
    tab << "      ";
  if(depth < max_depth)
  {
    for(int i=0; i<2; i++)
    {
      *label = *label+1;
      addNode(body_hh, joint_hh, lib_cc, libname, *label, parent_label);
      robot_hh << tab.str() << "Node< B" << *label << ",\n"
               << tab.str() << "      J" << *label << ",\n";
      buildTree(robot_hh, joint_hh, body_hh, lib_cc, libname,
                label, max_depth, depth+1);
      robot_hh << tab.str() << "    >" << (i==0?",":"") << "\n";
    }
  }
  else
  {
    *label = *label+1;
    addNode(body_hh, joint_hh, lib_cc, libname, *label, parent_label);
    robot_hh << tab.str() << "Node< B" << *label << ", J" << *label << ">,\n";
    *label = *label+1;
    addNode(body_hh, joint_hh, lib_cc, libname, *label, parent_label);
    robot_hh << tab.str() << "Node< B" << *label << ", J" << *label << ">\n";
  }
}

void metapod::benchmark::addNode(std::ofstream & body_hh,
                   std::ofstream & joint_hh,
                   std::ofstream & lib_cc,
                   const std::string & libname,
                   int label, int parent_label)
{
  std::stringstream name, parent_name, joint_name;
  name << "B" << label;
  parent_name << "B" << parent_label;
  joint_name << "J" << label;
  const std::string tab = "    ";
  createBody(body_hh, lib_cc, libname.c_str(), name.str(),
             parent_name.str(), joint_name.str(),
             label, 1., Vector3d::Random(), Matrix3d::Random(), tab);
  createJoint(joint_hh, lib_cc, libname.c_str(),
              REVOLUTE_AXIS_X, joint_name.str(), label, label,
              Matrix3d::Random(), Vector3d::Random(), tab);
}

void metapod::benchmark::makeLibraryHeader(std::ofstream & lib_hh, int nbdof)
{
  addLicense(lib_hh);
  lib_hh
    << "/* \n"
    << " * This file has been auto-generated by the make_model tool.\n"
    << " * You can modify it if you know what you're doing. It contains the\n"
    << " * Header of the " << nbdof << " dof sample model, used for benchmarking.\n"
    << " */\n"
    << std::endl
    << "#ifndef METAPOD_SAMPLE_" << nbdof << "_DOF_HH\n"
    << "# define METAPOD_SAMPLE_" << nbdof << "_DOF_HH\n"
    << std::endl
    << "# ifdef _MSC_VER\n"
    << "#  pragma warning( push )\n"
    << "// disable warning C4251: need to have DLL interface\n"
    << "// disable warning C4503: decorated name length exceeded\n"
    << "#  pragma warning( disable: 4251 4503 )\n"
    << "# endif\n\n"
    << "# include \"robot.hh\"\n\n"
    << "# ifdef _MSC_VER\n"
    << "#  pragma warning( pop )\n"
    << "# endif\n\n"
    << "#endif";
}

void metapod::benchmark::addHeader(std::ofstream & body_hh,
                                   std::ofstream & joint_hh,
                                   std::ofstream & robot_hh,
                                   std::ofstream & lib_cc,
                                   int nbdof)
{
  addLicense(body_hh);
  addLicense(joint_hh);
  addLicense(robot_hh);
  addLicense(lib_cc);

  body_hh
    << "/*\n"
    << " * This file has been auto-generated by the make_model tool.\n"
    << " * You can modify it if you know what you're doing. It contains the\n"
    << nbdof << " dof sample model bodies declaration.\n"
    << " */\n\n";
  joint_hh
    << "/*\n"
    << " * This file has been auto-generated by the make_model tool.\n"
    << " * You can modify it if you know what you're doing. It contains the\n"
    << " * " << nbdof << " dof sample model joints declaration.\n"
    << " */\n\n";
  robot_hh
    << "/*\n"
    << " * This file has been auto-generated by the make_model tool.\n"
    << " * You can modify it if you know what you're doing. It contains the\n"
    << " * " << nbdof << " dof sample model structure definition.\n"
    << " */\n\n";
  lib_cc
    << "/* \n"
    << " * This file has been auto-generated by the make_model tool.\n"
    << " * You can modify it if you know what you're doing. It contains the\n"
    << " * " << nbdof << " dof sample model, used for benchmarking.\n"
    << " */\n\n";
}

void metapod::benchmark::addLicense(std::ofstream & of)
{
  of
    << "// Copyright 2011, 2012,\n"
    << "//\n"
    << "// Maxime Reis\n"
    << "//\n"
    << "// JRL/LAAS, CNRS/AIST\n"
    << "//\n"
    << "// This file is part of metapod.\n"
    << "// metapod is free software: you can redistribute it and/or modify\n"
    << "// it under the terms of the GNU Lesser General Public License as published by\n"
    << "// the Free Software Foundation, either version 3 of the License, or\n"
    << "// (at your option) any later version.\n"
    << "//\n"
    << "// metapod is distributed in the hope that it will be useful,\n"
    << "// but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
    << "// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
    << "// GNU Lesser General Public License for more details.\n"
    << "// You should have received a copy of the GNU Lesser General Public License\n"
    << "// along with metapod.  If not, see <http://www.gnu.org/licenses/>.\n\n";
}
