#include "robotbuilder_p.hh"
#include "txttemplate.hh"
#include <iostream>
#include <cassert>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <boost/tokenizer.hpp>
#include <boost/filesystem/operations.hpp>

#if BOOST_VERSION <=104000
# include <boost/filesystem/convenience.hpp>
#endif

#include <boost/algorithm/string/join.hpp>
# include <metapod/tools/constants.hh>

namespace {

template <typename T>
std::string to_string(T x)
{
  std::ostringstream ss;
  ss << x;
  return ss.str();
}
// coin up a link/node name from joint name and body name.
// currently simply return the body name.
std::string node_name(const metapod::RobotModel& model, int link_id)
{
  return model.body_name(link_id);
}

// text of the template source files
extern "C" const char config_hh[];
extern "C" const size_t config_hh_len;
extern "C" const char init_hh[];
extern "C" const size_t init_hh_len;
extern "C" const char init_cc[];
extern "C" const size_t init_cc_len;

}

namespace metapod {

RobotBuilderP::RobotBuilderP()
  : nb_dof_(0),
    node_depth_(0),
    is_initialized_(false),
    use_dof_index_(false)
{}

RobotBuilderP::~RobotBuilderP()
{}

RobotBuilder::Status RobotBuilderP::set_directory(const std::string & directory)
{
  if (is_initialized_)
    {
      std::cerr
        << "ERROR: one cannot call set_directory() after having called addLink()"
        << std::endl;
      return RobotBuilder::STATUS_FAILURE;
    }
  directory_ = directory;
  return RobotBuilder::STATUS_SUCCESS;
}

RobotBuilder::Status RobotBuilderP::set_name(const std::string & name)
{
  if (is_initialized_)
    {
      std::cerr
        << "ERROR: one cannot call set_name() after having called addLink()"
        << std::endl;
      return RobotBuilder::STATUS_FAILURE;
    }
  name_ = name;
  return RobotBuilder::STATUS_SUCCESS;
}

RobotBuilder::Status RobotBuilderP::set_libname(const std::string & libname)
{
  if (is_initialized_)
    {
      std::cerr
        << "ERROR: one cannot call set_libname() after having called addLink()"
        << std::endl;
    }
  libname_ = libname;
  return RobotBuilder::STATUS_SUCCESS;
}

RobotBuilder::Status RobotBuilderP::set_use_dof_index(bool flag)
{
  if (is_initialized_)
    {
      std::cerr
        << "ERROR: one cannot call set_use_dof_index() after having called addLink()"
        << std::endl;
      return RobotBuilder::STATUS_FAILURE;
    }
  use_dof_index_ = flag;
  return RobotBuilder::STATUS_SUCCESS;
}

RobotBuilder::Status RobotBuilderP::set_license(const std::string& text)
{
  if (is_initialized_)
    {
      std::cerr
        << "ERROR: one cannot call set_license() after having called addLink()"
        << std::endl;
      return RobotBuilder::STATUS_FAILURE;
    }
  license_ = text;
  return RobotBuilder::STATUS_SUCCESS;
}

void RobotBuilderP::writeTemplate(const std::string& output_filename,
                                  const std::string& input_template,
                                  const ReplMap &repl) const
{
  assert(is_initialized_);
  std::stringstream output_path;
  output_path << directory_ << "/" << output_filename;
  std::ofstream output_stream;
  output_stream.open(output_path.str().c_str());
  output_stream << metapod::TxtTemplate(input_template).format(repl);
  output_stream.close();
}


RobotBuilder::Status RobotBuilderP::init()
{
  assert(!is_initialized_);

  // set up the map of replacements (for the ones that are known
  // already).
  std::string libname_uc(libname_);
  std::transform(libname_.begin(), libname_.end(), libname_uc.begin(),
                 ::toupper);
  replacements_[std::string("LIBRARY_NAME")] = libname_uc;
  std::stringstream export_symbol;
  export_symbol << libname_ << "_EXPORTS";
  replacements_["EXPORT_SYMBOL"] = export_symbol.str();
  replacements_["ROBOT_CLASS_NAME"] = name_;
  replacements_["ROBOT_NAME"] = name_;
  replacements_["LICENSE"] = license_;

  is_initialized_ = true;
  return RobotBuilder::STATUS_SUCCESS;
}

// parent_body_name: "NP" (no parent) or the parent body name
RobotBuilder::Status RobotBuilderP::addLink(const std::string& parent_body_name,
                                            const std::string& joint_name,
                                            unsigned int joint_type,
                                            const Eigen::Matrix3d & R_joint_parent,
                                            const Eigen::Vector3d & r_parent_joint,
                                            const std::string& body_name,
                                            double body_mass,
                                            const Eigen::Vector3d & body_center_of_mass,
                                            const Eigen::Matrix3d & body_rotational_inertia,
                                            const Eigen::Vector3d & joint_axis,
                                            int dof_index)
{

  if (!is_initialized_)
    {
      RobotBuilder::Status status = init();
      if (status == RobotBuilder::STATUS_FAILURE)
        {
          return RobotBuilder::STATUS_FAILURE;
        }
    }
  // check body_name
  if (body_name == "NP")
    {
      std::cerr
        << "ERROR: one cannot name a body 'NP'. This name stands for "
        << "'no parent' and is reserved"
        << std::endl;
      return RobotBuilder::STATUS_FAILURE;
    }
  // find an homonym
  int homonym_id = model_.find_link_by_body_name(body_name);
  if (homonym_id != NO_NODE)
    {
      std::cerr
        << "ERROR: there is already a body named '" << body_name << "'"
        << std::endl;
      return RobotBuilder::STATUS_FAILURE;
    }

  // TODO: check joint_Xt_E is a real rotation matrix
  // TODO: check body_name is a valid class name
  // TODO: check joint name
  // TODO: check joint_name is a valid class name

  // find the parent
  int parent_id = model_.find_link_by_body_name(parent_body_name);
  if (parent_id == NO_NODE)
    {
      std::cerr
        << "ERROR: could not find parent body named '" << parent_body_name << ". "
        << "Check the name and the order you add bodies in."
        << std::endl;
      return RobotBuilder::STATUS_FAILURE;
    }

  if (model_.nb_children(parent_id) >= MAX_NB_CHILDREN_PER_NODE)
    {
      std::cerr
        << "ERROR: a node cannot have more than " << MAX_NB_CHILDREN_PER_NODE
        << " children per node."
        << std::endl;
      return RobotBuilder::STATUS_FAILURE;
    }

  if (model_.nb_links() >= MAX_NB_JOINTS)
    {
      std::cerr
        << "ERROR: a model cannot have more than " << MAX_NB_JOINTS
        << " joints."
        << std::endl;
      return RobotBuilder::STATUS_FAILURE;
    }

  // deal with joint type
  unsigned int joint_nb_dof;
  switch (joint_type) {
  case RobotBuilder::REVOLUTE_AXIS_ANY:
  case RobotBuilder::REVOLUTE_AXIS_X:
  case RobotBuilder::REVOLUTE_AXIS_Y:
  case RobotBuilder::REVOLUTE_AXIS_Z:
    {
      joint_nb_dof = 1;
      break;
    }
  case RobotBuilder::FREE_FLYER:
    {
      joint_nb_dof = 6;
      break;
    }
  default:
    {
      std::cerr
        << "ERROR: Joint '" << joint_name << "' is of unknown type"
        << std::endl;
      return RobotBuilder::STATUS_FAILURE;
    }
  }
  // deal with joint_index
  int joint_position_in_conf = -1;
  if (use_dof_index_)
    {
      if (dof_index < 0)
        {
          std::cerr
            << "ERROR: dof_index for joint '" << joint_name << "' is inconsitent"
            << std::endl;
          return RobotBuilder::STATUS_FAILURE;
        }
      joint_position_in_conf = dof_index;
    }
  else
    {
      joint_position_in_conf = nb_dof_;
    }
  // add the link for real
  int link_id = model_.nb_links();
  model_.add_link(Link(
                       link_id,
                       parent_id,
                       joint_name,
                       joint_type,
                       R_joint_parent,
                       r_parent_joint,
                       body_name,
                       body_mass,
                       body_center_of_mass,
                       body_rotational_inertia,
                       joint_axis,
                       joint_position_in_conf));
  nb_dof_ += joint_nb_dof;
  return RobotBuilder::STATUS_SUCCESS;
}

void RobotBuilderP::writeLink(int link_id, TmpStreams &out) const
{
  std::string joint_type, joint_rotation_type;
  switch(model_.joint_type(link_id))
    {
    case metapod::RobotBuilder::FREE_FLYER:
      joint_type = "FreeFlyerJoint";
      joint_rotation_type = "Spatial::RotationMatrix";
      break;
    case metapod::RobotBuilder::REVOLUTE_AXIS_X:
      joint_type = "RevoluteAxisXJoint";
      joint_rotation_type = "Spatial::RotationMatrixAboutX";
      break;
    case metapod::RobotBuilder::REVOLUTE_AXIS_Y:
      joint_type = "RevoluteAxisYJoint";
      joint_rotation_type = "Spatial::RotationMatrixAboutY";
      break;
    case metapod::RobotBuilder::REVOLUTE_AXIS_Z:
      joint_type = "RevoluteAxisZJoint";
      joint_rotation_type = "Spatial::RotationMatrixAboutZ";
      break;
    case metapod::RobotBuilder::REVOLUTE_AXIS_ANY:
      joint_type = "RevoluteAxisAnyJoint";
      joint_rotation_type = "Spatial::RotationMatrix";
      break;
    }

  Eigen::IOFormat comma_fmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                            ", ", ", ");
  const int parent_id = model_.parent_id(link_id);
  ReplMap repl(replacements_);
  repl["node_id"] = ::to_string(link_id);
  repl["node_name"] = ::node_name(model_, link_id);
  repl["dof_index"] = ::to_string(model_.dof_index(link_id));
  repl["joint_type"] = joint_type;
  repl["joint_rotation_type"] = joint_rotation_type;
  repl["joint_name"] = model_.joint_name(link_id);

  const Eigen::Matrix3d &R_joint_parent = model_.R_joint_parent(link_id);
  if (R_joint_parent.isApprox(Eigen::Matrix3d::Identity())) {
    repl["R_joint_parent_type"] = "Spatial::RotationMatrixIdentity";
    repl["X_joint_parent_type"] = "Spatial::TransformT<Spatial::RotationMatrixIdentity>";
    repl["R_joint_parent"] = "Spatial::RotationMatrixIdentity()";
  } else {
    repl["R_joint_parent_type"] = "Spatial::RotationMatrix";
    repl["X_joint_parent_type"] = "Spatial::Transform";
    std::stringstream ss0;
    ss0 << "matrix3dMaker("
        << model_.R_joint_parent(link_id).format(comma_fmt)
        << ")";
    repl["R_joint_parent"] = ss0.str();
  }
  std::stringstream ss1;
  ss1 << "Vector3d("
      << model_.r_parent_joint(link_id).format(comma_fmt)
      << ")";
  repl["r_parent_joint"] = ss1.str();
  repl["body_name"] = model_.body_name(link_id);
  repl["body_mass"] = ::to_string(model_.body_mass(link_id));
  std::stringstream ss2;
  ss2 << "Vector3d("
      << model_.body_center_of_mass(link_id).format(comma_fmt)
      << ")";
  repl["body_center_of_mass"] = ss2.str();
  std::stringstream ss3;
  ss3 << "matrix3dMaker("
      << model_.body_rotational_inertia(link_id).format(comma_fmt)
      << ")";
  repl["body_rotational_inertia"] = ss3.str();
  repl["parent_id"] = ::to_string(parent_id);

  // fill childX_id
  for (int i = 0; i<MAX_NB_CHILDREN_PER_NODE; ++i)
    {
      std::stringstream key;
      key << "child" << i << "_id";
      repl[key.str()] = ::to_string(model_.child_id(link_id, i));
    }

  bool is_last_link = (link_id == model_.nb_links()-1);

  const TxtTemplate tpl0(
                         "    @node_name@ = @node_id@");
  out.nodeid_enum_definition << tpl0.format(repl);
  if (!is_last_link)
    out.nodeid_enum_definition << ",\n";

  const TxtTemplate tpl1(
      "\n"
      "  class Node@node_id@ {\n"
      "  public:\n"
      "    Node@node_id@();\n"
      "    static const int id = @node_id@;\n"
      "    static const std::string joint_name;\n"
      "    static const std::string body_name;\n"
      "    static const @X_joint_parent_type@ Xt;\n"
      "    static const int q_idx = @dof_index@;\n"
      "    typedef @joint_type@ Joint;\n"
      "    static const int parent_id = @parent_id@;\n"
      "    static const int child0_id = @child0_id@;\n"
      "    static const int child1_id = @child1_id@;\n"
      "    static const int child2_id = @child2_id@;\n"
      "    static const int child3_id = @child3_id@;\n"
      "    static const int child4_id = @child4_id@;\n"
      "    static const FloatType mass;\n"
      "    static const Spatial::Inertia I; // in body frame\n"
      "    Spatial::TransformT<Spatial::rm_mul_op<@joint_rotation_type@, @R_joint_parent_type@>::rm> sXp;\n"
      "    Eigen::Matrix<FloatType, 6, Joint::NBDOF> joint_F; // used by crba\n"
      "    Joint joint;\n"
      "    Body body;\n"
      "  };\n");
  out.node_type_definitions << tpl1.format(repl);

  const TxtTemplate tpl2(
                         "      Node@node_id@");
  out.nodes_type_list << tpl2.format(repl);
  if (!is_last_link)
    out.nodes_type_list << ",\n";

  const TxtTemplate tpl3(
                         "template <> struct Nodes <@ROBOT_CLASS_NAME@, @node_id@> "
                         "{typedef @ROBOT_CLASS_NAME@::Node@node_id@ type;};\n");
  out.map_node_id_to_type << tpl3.format(repl);

  // fill bits for init.cc
  const TxtTemplate tpl4(
      "const std::string @ROBOT_CLASS_NAME@::Node@node_id@::joint_name = std::string(\"@joint_name@\");\n"
      "const std::string @ROBOT_CLASS_NAME@::Node@node_id@::body_name = std::string(\"@body_name@\");\n"
      "const @X_joint_parent_type@ @ROBOT_CLASS_NAME@::Node@node_id@::Xt = @X_joint_parent_type@(\n"
      "    @R_joint_parent@,\n"
      "    @r_parent_joint@);\n"
      "const FloatType @ROBOT_CLASS_NAME@::Node@node_id@::mass = @body_mass@;\n"
      "const Spatial::Inertia @ROBOT_CLASS_NAME@::Node@node_id@::I = spatialInertiaMaker(\n"
      "    @body_mass@,\n"
      "    @body_center_of_mass@,\n"
      "    @body_rotational_inertia@);\n\n"
      "@ROBOT_CLASS_NAME@::Node@node_id@::Node@node_id@():\n"
      "  joint(@joint_args@) {}\n\n");
  if (model_.joint_type(link_id) == RobotBuilder::REVOLUTE_AXIS_ANY)
    {
      std::stringstream ss;
      ss << model_.joint_axis(link_id).format(comma_fmt);
      repl["joint_args"] = ss.str();
    }
  out.init_nodes << tpl4.format(repl);

}

RobotBuilder::Status RobotBuilderP::write() const
{
  if (!is_initialized_)
    {
      return RobotBuilder::STATUS_FAILURE;
    }

  // create the directory (and its parents if necessary)
  boost::filesystem::create_directories(directory_);

  // fill the replacements we already know
  ReplMap repl(replacements_);
  repl[std::string("ROBOT_NB_DOF")] = ::to_string(nb_dof_);
  repl[std::string("ROBOT_NB_BODIES")] = ::to_string(model_.nb_links());
  TmpStreams streams;
  // add the links
  for (int i = 0; i != model_.nb_links(); ++i)
    {
      writeLink(i, streams);
    }

  // complete the replacements map

  repl["nodeid_enum_definition"] = streams.nodeid_enum_definition.str();
  repl["node_type_definitions"] = streams.node_type_definitions.str();
  repl["nodes_type_list"] = streams.nodes_type_list.str();
  repl["map_node_id_to_type"] = streams.map_node_id_to_type.str();

  for (int i = 0; i<MAX_NB_CHILDREN_PER_NODE; ++i)
    {
      std::stringstream key;
      key << "root_child" << i << "_id";
      repl[key.str()] = ::to_string(model_.child_id(NO_PARENT, i));
    }

  // init.cc
  repl["init_nodes"] = streams.init_nodes.str();

  // generate files from template and replacements
  const std::string config_hh_templ(::config_hh, ::config_hh_len);
  writeTemplate("config.hh", config_hh_templ, repl);

  const std::string init_hh_templ(::init_hh, ::init_hh_len);
  writeTemplate(name_ + ".hh", init_hh_templ, repl);

  const std::string init_cc_templ(::init_cc, ::init_cc_len);
  writeTemplate(name_ + ".cc", init_cc_templ, repl);
  return RobotBuilder::STATUS_SUCCESS;
}

}
