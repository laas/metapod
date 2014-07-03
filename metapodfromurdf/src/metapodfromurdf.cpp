// Copyright (c) 2012, 2013 Aldebaran Robotics. All rights reserved
// Use of this source code is governed by a BSD-style license that can be
// found in the COPYING.bsd file
#include <iostream>
#include <sstream>
#include <string>
#include <fstream>
#include <sstream>
#include <boost/version.hpp>
#include <boost/program_options.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <vector>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <Eigen/Dense>
#include <boost/tokenizer.hpp>
#ifdef USE_URDF_FROM_ROS_FUERTE
# include <ros/console.h>
# include <urdf_interface/model.h>
# include <urdf/model.h>
# define logInform(...) ROS_INFO(__VA_ARGS__)
# define logError(...) ROS_ERROR(__VA_ARGS__)
#else
# include <console_bridge/console.h>
# include <boost/filesystem.hpp>
# include <sstream>
# include <fstream>
# include <urdf_model/model.h>
# include <urdf_parser/urdf_parser.h>
#endif
#include <metapod/robotbuilder/robotbuilder.hh>


typedef metapod::RobotBuilder::Status Status;
const Status STATUS_SUCCESS = metapod::RobotBuilder::STATUS_SUCCESS;
const Status STATUS_FAILURE = metapod::RobotBuilder::STATUS_FAILURE;

// Utility functions
Eigen::Vector3d toEigen(urdf::Vector3 v)
{
  return Eigen::Vector3d(v.x, v.y, v.z);
}

Eigen::Matrix3d toEigen(urdf::Rotation q, double epsilon = 1e-16)
{
  Eigen::Quaterniond tmp_q(q.w, q.x, q.y, q.z);
  Eigen::Matrix3d R;
  R = tmp_q;
  // crush values that are almost zero
  if (epsilon) {
    for(int i=0; i<3; ++i) {
      for(int j=0; j<3; ++j) {
        if (std::abs(R(i, j)) < epsilon)
          R(i, j) = 0.;
      }
    }
  }
  return R;
}

// LinkComparer: for use with std::sort.
class LinkComparer
{
public:
  LinkComparer();
  Status init(const std::vector<std::string>& joint_names);
  // Returns true when link0 sorts before link1
  bool operator()(boost::shared_ptr<urdf::Link> link0,
                  boost::shared_ptr<urdf::Link> link1);
private:
  std::vector<std::string> joint_ordering_;
};

LinkComparer::LinkComparer() {}

Status LinkComparer::init(const std::vector<std::string>& joint_names)
{
  // TODO: check joint_names has no dupe
  joint_ordering_ = joint_names;
  return STATUS_SUCCESS;
}

bool LinkComparer::operator()(boost::shared_ptr<urdf::Link> link0,
                              boost::shared_ptr<urdf::Link> link1)
{
  const std::string& joint0 = link0->parent_joint->name;
  const std::string& joint1 = link1->parent_joint->name;
  std::vector<std::string>::const_iterator it0, it1;
  it0 = std::find(joint_ordering_.begin(), joint_ordering_.end(), joint0);
  it1 = std::find(joint_ordering_.begin(), joint_ordering_.end(), joint1);
  if (it0 == joint_ordering_.end()) {
    if (it1 == joint_ordering_.end()) {
      // none are in the list, use normal ordering
      return (joint0 < joint1);
    } else {
      // joint1 is in the list, it comes first
      return false;
    }
  } else {
    if (it1 == joint_ordering_.end()) {
      // joint0 is in the list, it comes first
      return true;
    } else {
      // both are in the list, use list ordering
      return (it0 < it1);
    }
  }
}

// Function that recursively calls itself while traversing the URDF tree.
Status addSubTree(
    metapod::RobotBuilder& builder, const LinkComparer& link_comparer,
    boost::shared_ptr<const urdf::Link> root,
    const std::string& parent_body_name,
    bool prefer_fixed_axis,
    const std::map<std::string, int>& joint_dof_index,
    bool has_parent)
{
  const std::string tab("\t");

  std::vector<boost::shared_ptr<urdf::Link> > children = root->child_links;

  // add root itself
  // convert the joint
  boost::shared_ptr<urdf::Joint> jnt = root->parent_joint;
  unsigned int metapod_joint_type;
  switch (jnt->type) {
    case urdf::Joint::REVOLUTE:
    case urdf::Joint::CONTINUOUS: {
      if (prefer_fixed_axis &&
          jnt->axis.x == 1. && jnt->axis.y == 0. && jnt->axis.z == 0.) {
        logInform("Adding joint '%s' as a REVOLUTE_AXIS_X joint",
                  jnt->name.c_str());
        metapod_joint_type = metapod::RobotBuilder::REVOLUTE_AXIS_X;
      } else if (prefer_fixed_axis &&
                 jnt->axis.x == 0. && jnt->axis.y == 1. && jnt->axis.z == 0.) {
        logInform("Adding joint '%s' as a REVOLUTE_AXIS_Y joint",
                  jnt->name.c_str());
        metapod_joint_type = metapod::RobotBuilder::REVOLUTE_AXIS_Y;
      } else if (prefer_fixed_axis &&
                 jnt->axis.x == 0. && jnt->axis.y == 0. && jnt->axis.z == 1.) {
        logInform("Adding joint '%s' as a REVOLUTE_AXIS_Z joint",
                  jnt->name.c_str());
        metapod_joint_type = metapod::RobotBuilder::REVOLUTE_AXIS_Z;
      } else {
        logInform("Adding joint '%s' as a REVOLUTE_AXIS_ANY joint",
                  jnt->name.c_str());
        metapod_joint_type = metapod::RobotBuilder::REVOLUTE_AXIS_ANY;
      }
      break;
    }
    case urdf::Joint::FLOATING: {
      metapod_joint_type = metapod::RobotBuilder::FREE_FLYER;
      logInform("Adding joint '%s' as a FREE_FLYER joint", jnt->name.c_str());
      break;
    }
    default: {
      logError("Joint '%s' is of unknown type", jnt->name.c_str());
      return STATUS_FAILURE;
      break;
    }
  }
  // constructs the optional inertia
  double mass = 1.;
  Eigen::Vector3d center_of_mass = Eigen::Vector3d::Zero();
  Eigen::Matrix3d rotational_inertia = Eigen::Matrix3d::Identity();
  if (root->inertial) {
    mass = root->inertial->mass; // TODO: check mass >0
    urdf::Pose& p = root->inertial->origin;
    center_of_mass = toEigen(p.position);
    // metapod expects the rotational inertia in a frame aligned with the frame
    // of the link, but with origin at center of mass
    // urdf specifies it in a frame whose origin is the link center of mass,
    // and whose basis is arbitrary
    // So, let's rotate it! (Yeah!)
    Eigen::Matrix3d R = toEigen(p.rotation);
    Eigen::Matrix3d tmp;
    tmp << root->inertial->ixx, root->inertial->ixy, root->inertial->ixz,
      root->inertial->ixy, root->inertial->iyy, root->inertial->iyz,
      root->inertial->ixz, root->inertial->iyz, root->inertial->izz;
    rotational_inertia.noalias() = R * tmp * R.transpose();
  }
  int dof_index = -1;
  std::map<std::string, int>::const_iterator it =
    joint_dof_index.find(jnt->name);
  if (it != joint_dof_index.end())
    dof_index = it->second;
  Status status = builder.addLink(
      has_parent ? parent_body_name : std::string("NP"),
      jnt->name,
      metapod_joint_type,
      toEigen(jnt->parent_to_joint_origin_transform.rotation).transpose(), // R_joint_parent
      toEigen(jnt->parent_to_joint_origin_transform.position), // r_parent_joint
      root->name,
      mass,
      center_of_mass,
      rotational_inertia,
      toEigen(jnt->axis),
      dof_index);
  if (status == STATUS_FAILURE)
    return STATUS_FAILURE;

  std::sort(children.begin(), children.end(), link_comparer);
  for (size_t i=0; i<children.size(); ++i) {
    const bool has_parent = true;
    status = addSubTree(builder, link_comparer,children[i], root->name,
                        prefer_fixed_axis, joint_dof_index, has_parent);
    if (status == STATUS_FAILURE)
      return STATUS_FAILURE;
  }
  return STATUS_SUCCESS;
}

Status treeFromUrdfModel(const urdf::ModelInterface& robot_model,
                         metapod::RobotBuilder& builder,
                         const LinkComparer& link_comparer,
                         bool prefer_fixed_axis,
                         const std::map<std::string, int>& joint_dof_index) {
  //  add all children
  const bool has_parent = false;
  for (size_t i=0; i<robot_model.getRoot()->child_links.size(); ++i) {
    // TODO: what happens when there are two robots?
    Status status = addSubTree(builder, link_comparer,
                               robot_model.getRoot()->child_links[i],
                               std::string("GROUND"),
                               prefer_fixed_axis,
                               joint_dof_index,
                               has_parent);
    if (status == STATUS_FAILURE)
      return STATUS_FAILURE;
  }
  return STATUS_SUCCESS;
}

namespace po = boost::program_options;

int main(int argc, char** argv) {
  // Declare a group of options that will be
  // allowed only on command line
  po::options_description generic("");
  generic.add_options()
    ("help", "produce help message")
    ("config-file", po::value<std::string>(),
     "a config file which can store options");

  // Declare a group of options that will be
  // allowed both on command line and in
  // config file
  po::options_description config("");
  config.add_options()
    ("name", po::value<std::string>(),
     "the robot name")
    ("metapod-default-float-type", po::value<std::string>(),
     "default metapod float type")
    ("libname", po::value<std::string>(),
     "the library name, used for DLL symbol import/export. If omitted, the "
     "value of the name option will be used")
    ("directory", po::value<std::string>(),
     "directory where the files will be generated")
    ("license-file", po::value<std::string>(),
     "license text, will be copied on top of every generated file")
    ("joint-dof-index", po::value<std::vector<std::string> >(),
     "joint name to dof index mapping, in the form "
     "joint_name:dof_index, this option should be passed either for "
     "every joint of for none at all")
    ("joint", po::value<std::vector<std::string> >(),
     "joint name, pass several of them to specify joints ordering")
    ("prefer-fixed-axis",
     "use REVOLUTE_AXIS_X instead of REVOLUTE_AXIS_ANY when possible.");


  // Hidden options, will be allowed both on command line and
  // in config file, but will not be shown to the user.
  po::options_description hidden("Hidden options");
  hidden.add_options()
    ("input-file", po::value<std::string>(),
     "input file in urdf format");

  po::options_description cmdline_options, config_file_options;
  cmdline_options.add(generic).add(config).add(hidden);
  // we skip generic, because there is no use for the --help and
  // --config-file options in the config file
  config_file_options.add(config).add(hidden);

  po::positional_options_description pos;
  pos.add("input-file", -1);

  po::options_description visible(
      "Usage:\n "
      "metapodfromurdf [options] input-file\n\n"
      "Options");
  visible.add(generic).add(config);
  po::variables_map vm;
  std::map<std::string, int> joint_dof_index;
  metapod::RobotBuilder builder;
  LinkComparer link_comparer;
  try {
    po::store(po::command_line_parser(argc, argv).
              options(cmdline_options).positional(pos).run(),
              vm);
    if (vm.count("help")) {
      std::cout << visible << "\n";
      return 0;
    }
    // deal with mandatory options
    // with boost >= 1.42, we'll declare these options as required and
    // this check won't be necessary anymore.
    if (vm.count("input-file") == 0) {
      std::cout << visible << "\n";
      return 1;
    }
    if (vm.count("config-file")) {
      std::ifstream stream(vm["config-file"].as<std::string>().c_str());
      if (stream.is_open()) {
        po::store(po::parse_config_file(stream, config_file_options, true),
                  vm);
      }
    }
    po::notify(vm);
    if (vm.count("joint-dof-index"))
    {
      std::vector<std::string> pairs =
        vm["joint-dof-index"].as<std::vector<std::string> >();
      for (std::vector<std::string>::const_iterator it = pairs.begin();
           it != pairs.end();
           ++it) {
        // Tokenize the string on the ":" delimiter.
        std::vector< std::string > tokens;
        boost::split(tokens, *it, boost::is_any_of( ":" ));

        using boost::program_options::validation_error;
        int value = -1;
        if ((tokens.size() != 2) || !(std::stringstream(tokens[1]) >> value)) {
#if BOOST_VERSION >= 104200
          throw validation_error(validation_error::invalid_option_value,
                                 *it, "joint-dof-index");
#else
          std::stringstream msg("invalid joint-dof-index option value:");
          msg << *it;
          throw validation_error(msg.str());
#endif
        }
        joint_dof_index[tokens[0]] = value;
      }
      builder.set_use_dof_index(true);
    }
  }
  catch(boost::program_options::error) {
    std::cout << visible << "\n";
    return 0;
  }
  const std::string input_file = vm["input-file"].as<std::string>();
#ifdef USE_URDF_FROM_ROS_FUERTE
  boost::shared_ptr<urdf::Model> robot_model = boost::make_shared<urdf::Model>();
  if (!robot_model->initFile(input_file)) {
    logError("Could not generate robot model from file '%s'", input_file.c_str());
    return STATUS_FAILURE;
  }
#else
  std::stringstream buffer;
  try
  {
    std::fstream file(input_file.c_str(),
                      std::ios::in);
    if(!file)
    {
      //TODO
      std::stringstream errorMessage;
      errorMessage << "'" << input_file << "' could not be opened.";
      throw std::invalid_argument(errorMessage.str());
    }
    buffer << file.rdbuf();
  }
  catch(const boost::filesystem::filesystem_error&)
  {
    //TODO
    std::stringstream errorMessage;
    errorMessage << "'" << input_file << "' could not be opened.";
    throw std::invalid_argument(errorMessage.str());
  }
  boost::shared_ptr<urdf::ModelInterface> robot_model =
      urdf::parseURDF(buffer.str());
  if (!robot_model)
  {
    logError("Could not generate robot model from file '%s'", input_file.c_str());
    return STATUS_FAILURE;
  }
#endif
  if (vm.count("name")) {
    builder.set_name(vm["name"].as<std::string>());
  }
  if (vm.count("metapod-default-float-type")) {
    builder.set_metapod_default_float_type(vm["metapod-default-float-type"].as<std::string>());
  }
  
  if (vm.count("libname")) {
    builder.set_libname(vm["libname"].as<std::string>());
  }
  else if (vm.count("name")) {
    builder.set_libname(vm["name"].as<std::string>());
  }
  
  if (vm.count("directory")) {
    builder.set_directory(vm["directory"].as<std::string>());
  }
  if (vm.count("license-file")) {
    std::ifstream stream(vm["license-file"].as<std::string>().c_str());
    if (stream.is_open()) {
        builder.set_license(std::string(std::istreambuf_iterator<char>(stream),
                                        std::istreambuf_iterator<char>()));
    }
  }
  if (vm.count("joint")) {
    link_comparer.init(vm["joint"].as<std::vector<std::string> >());
  }
  bool prefer_fixed_axis = false;
  if (vm.count("prefer-fixed-axis")) {
    prefer_fixed_axis = true;
  }
  Status status = treeFromUrdfModel(*robot_model, builder, link_comparer,
                                    prefer_fixed_axis, joint_dof_index);
  if (status == STATUS_FAILURE) {
    return STATUS_FAILURE;
  }
  return builder.write();
}
