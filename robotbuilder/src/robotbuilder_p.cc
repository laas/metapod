#include "robotbuilder_p.hh"
#include "txttemplate.hh"
#include <iostream>
#include <cassert>
#include <algorithm>
#include <cctype>
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

void RobotBuilderP::writeTemplate(
    const std::string& output_filename,
    const std::string& input_template)
{
  assert(is_initialized_);
  std::stringstream output_path;
  output_path << directory_ << "/" << output_filename;
  std::ofstream output_stream;
  output_stream.open(output_path.str().c_str());
  output_stream << metapod::TxtTemplate(input_template).format(replacements_);
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
RobotBuilder::Status RobotBuilderP::addLink(
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

void RobotBuilderP::writeLink(int link_id)
{
  std::string joint_type;
  switch(model_.joint_type(link_id))
  {
  case metapod::RobotBuilder::FREE_FLYER:
    joint_type = std::string("FreeFlyerJoint");
    break;
  case metapod::RobotBuilder::REVOLUTE_AXIS_X:
    joint_type = std::string("RevoluteAxisXJoint");
    break;
  case metapod::RobotBuilder::REVOLUTE_AXIS_ANY:
    joint_type = std::string("RevoluteAxisAnyJoint");
    break;
  }

  Eigen::IOFormat comma_fmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                            ", ", ", ");
  const int parent_id = model_.parent_id(link_id);
  std::map<std::string, std::string> repl(replacements_);
  repl["node_id"] = ::to_string(link_id);
  repl["node_name"] = ::node_name(model_, link_id);
  repl["dof_index"] = ::to_string(model_.dof_index(link_id));
  repl["joint_type"] = ::to_string(joint_type);
  repl["joint_name"] = model_.joint_name(link_id);
  std::stringstream ss0;
  ss0 << "matrix3dMaker("
      << model_.R_joint_parent(link_id).format(comma_fmt)
      << ")";
  repl["R_joint_parent"] = ss0.str();
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
  nodeid_enum_definition_ss_ << tpl0.format(repl);
  if (!is_last_link)
    nodeid_enum_definition_ss_ << ",\n";

  const TxtTemplate tpl1(
      "\n"
      "  class Node@node_id@ {\n"
      "  public:\n"
      "    Node@node_id@();\n"
      "    static const int id = @node_id@;\n"
      "    static const std::string joint_name;\n"
      "    static const std::string body_name;\n"
      "    static const Spatial::Transform Xt;\n"
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
      "    Spatial::Transform sXp;\n"
      "    Eigen::Matrix<FloatType, 6, Joint::NBDOF> joint_F; // used by crba\n"
      "    Joint joint;\n"
      "    Body body;\n"
      "  };\n");
  node_type_definitions_ss_ << tpl1.format(repl);

  const TxtTemplate tpl2(
      "      Node@node_id@");
  nodes_type_list_ss_ << tpl2.format(repl);
  if (!is_last_link)
    nodes_type_list_ss_ << ",\n";

  const TxtTemplate tpl3(
      "template <> struct Nodes <@ROBOT_CLASS_NAME@, @node_id@> "
      "{typedef @ROBOT_CLASS_NAME@::Node@node_id@ type;};\n");
  map_node_id_to_type_ss_ << tpl3.format(repl);

  // fill bits for init.cc
  const TxtTemplate tpl4(
      "const std::string @ROBOT_CLASS_NAME@::Node@node_id@::joint_name = std::string(\"@joint_name@\");\n"
      "const std::string @ROBOT_CLASS_NAME@::Node@node_id@::body_name = std::string(\"@body_name@\");\n"
      "const Spatial::Transform @ROBOT_CLASS_NAME@::Node@node_id@::Xt = Spatial::Transform(\n"
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
  init_nodes_ss_ << tpl4.format(repl);
}

RobotBuilder::Status RobotBuilderP::write()
{
  if (!is_initialized_)
  {
    return RobotBuilder::STATUS_FAILURE;
  }

  // create the directory (and its parents if necessary)
  boost::filesystem::create_directories(directory_);

  // fill the replacements we already know
  replacements_[std::string("ROBOT_NB_DOF")] = ::to_string(nb_dof_);
  replacements_[std::string("ROBOT_NB_BODIES")] = ::to_string(model_.nb_links());

  // add the links
  for (int i = 0; i != model_.nb_links(); ++i)
  {
    writeLink(i);
  }

  // complete the replacements map

  replacements_["nodeid_enum_definition"] = nodeid_enum_definition_ss_.str();
  replacements_["node_type_definitions"] = node_type_definitions_ss_.str();
  replacements_["nodes_type_list"] = nodes_type_list_ss_.str();
  replacements_["map_node_id_to_type"] = map_node_id_to_type_ss_.str();

  for (int i = 0; i<MAX_NB_CHILDREN_PER_NODE; ++i)
  {
    std::stringstream key;
    key << "root_child" << i << "_id";
    replacements_[key.str()] = ::to_string(model_.child_id(NO_PARENT, i));
  }

  // init.cc
  replacements_["init_nodes"] = init_nodes_ss_.str();

  // generate files from template and replacements
  const std::string config_hh_templ(::config_hh, ::config_hh_len);
  writeTemplate("config.hh", config_hh_templ);

  const std::string init_hh_templ(::init_hh, ::init_hh_len);
  writeTemplate(name_ + ".hh", init_hh_templ);

  const std::string init_cc_templ(::init_cc, ::init_cc_len);
  writeTemplate(name_ + ".cc", init_cc_templ);
  return RobotBuilder::STATUS_SUCCESS;
}

}
