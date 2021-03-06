#include "robotbuilder_p.hh"
#include "txttemplate.hh"
#include <iostream>
#include <cassert>
#include <algorithm>
#include <cctype>
#include <sstream>
#include <boost/tokenizer.hpp>
#include <boost/filesystem/operations.hpp>
#include <set>

#if BOOST_VERSION <=104000
# include <boost/filesystem/convenience.hpp>
#endif

#include <boost/algorithm/string/join.hpp>
#include <metapod/tools/constants.hh>
#ifdef _MSC_VER
# include <stdio.h> // for _set_output_format
#endif

namespace {

template <typename T>
std::string to_string(T x)
{
  std::ostringstream ss;
  ss << x;
  return ss.str();
}

template <>
std::string to_string<bool>(bool x)
{
  std::ostringstream ss;
  x? ss << "true" : ss << "false";
  return ss.str();
}

bool isLetter(char c)
{
  return ('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z');
}

bool isLetterOrNumberOrUnderscore(char c)
{
  return isLetter(c) || ('0' <= c && c <= '9') || ( c == '_');
}

bool isNotLetterOrNumberOrUnderscore(char c)
{
  return !isLetterOrNumberOrUnderscore(c);
}

std::set<std::string> getReservedKeywords()
{
  std::set<std::string> s;
  s.insert("asm");
  s.insert("auto");
  s.insert("bool");
  s.insert("break");
  s.insert("case");
  s.insert("catch");
  s.insert("char");
  s.insert("class");
  s.insert("const");
  s.insert("const_cast");
  s.insert("continue");
  s.insert("default");
  s.insert("delete");
  s.insert("do");
  s.insert("double");
  s.insert("dynamic_cast");
  s.insert("else");
  s.insert("enum");
  s.insert("explicit");
  s.insert("export");
  s.insert("extern");
  s.insert("false");
  s.insert("float");
  s.insert("for");
  s.insert("friend");
  s.insert("goto");
  s.insert("if");
  s.insert("inline");
  s.insert("int");
  s.insert("long");
  s.insert("mutable");
  s.insert("namespace");
  s.insert("new");
  s.insert("operator");
  s.insert("private");
  s.insert("protected");
  s.insert("public");
  s.insert("register");
  s.insert("reinterpret_cast");
  s.insert("return");
  s.insert("short");
  s.insert("signed");
  s.insert("sizeof");
  s.insert("static");
  s.insert("static_cast");
  s.insert("struct");
  s.insert("switch");
  s.insert("template");
  s.insert("this");
  s.insert("throw");
  s.insert("true");
  s.insert("try");
  s.insert("typedef");
  s.insert("typeid");
  s.insert("typename");
  s.insert("union");
  s.insert("unsigned");
  s.insert("using");
  s.insert("virtual");
  s.insert("void");
  s.insert("volatile");
  s.insert("wchar_t");
  s.insert("while");
  s.insert("and");
  s.insert("and_eq");
  s.insert("bitand");
  s.insert("bitor");
  s.insert("compl");
  s.insert("not");
  s.insert("not_eq");
  s.insert("or");
  s.insert("or_eq");
  s.insert("xor");
  s.insert("xor_eq");
  return s;

}

static
bool isReservedKeyword(const std::string& name)
{
  static const std::set<std::string> reserved_keywords(getReservedKeywords());
  return (std::find(reserved_keywords.begin(), reserved_keywords.end(), name)
          != reserved_keywords.end());
}

static
bool isValidIdentifier(const std::string& name)
{
  return (!name.empty() &&
          isLetter(name[0]) &&
          (std::find_if(++(name.begin()), name.end(),
                           isNotLetterOrNumberOrUnderscore)
              == name.end()) &&
          !isReservedKeyword(name));
}

// coin up a link/node name from joint name and body name.
// currently simply return the body name.
static
std::string node_name(const std::string& /*joint_name*/,
                      const std::string& body_name)
{
  return body_name;
}

static
std::string node_name(const metapod::RobotModel& model, int link_id)
{
  return node_name(model.joint_name(link_id),
                     model.body_name(link_id));
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
    is_initialized_(false),
    use_dof_index_(false)
{}

RobotBuilderP::~RobotBuilderP()
{}

RobotBuilder::Status RobotBuilderP::set_directory(const std::string & directory)
{
  directory_ = directory;
  return RobotBuilder::STATUS_SUCCESS;
}

RobotBuilder::Status RobotBuilderP::set_name(const std::string & name)
{
  if (!::isValidIdentifier(name)) {
    std::cerr
        << "ERROR: name \"" << name << "\" is invalid."
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  name_ = name;
  return RobotBuilder::STATUS_SUCCESS;
}

RobotBuilder::Status RobotBuilderP::set_libname(const std::string& name)
{
  if (!::isValidIdentifier(name)) {
    std::cerr
        << "ERROR: libname \"" << name << "\" is invalid."
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  libname_ = name;
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
  license_ = text;
  return RobotBuilder::STATUS_SUCCESS;
}

void RobotBuilderP::writeTemplate(const std::string& output_filename,
                                  const std::string& input_template,
                                  const ReplMap &replacements) const
{
  assert(is_initialized_);
  std::stringstream output_path;
  output_path << directory_ << "/" << output_filename;
  std::ofstream output_stream;
  output_stream.open(output_path.str().c_str());
  output_stream << metapod::TxtTemplate(input_template).format(replacements);
  output_stream.close();
}


RobotBuilder::Status RobotBuilderP::init()
{
  assert(!is_initialized_);
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
                                            bool fwdDyn,
                                            int dof_index)
{

  if (!is_initialized_) {
    RobotBuilder::Status status = init();
    if (status == RobotBuilder::STATUS_FAILURE) {
      return RobotBuilder::STATUS_FAILURE;
    }
  }
  // find an homonym joint
  int joint_homonym_id = model_.find_link_by_joint_name(joint_name);
  if (joint_homonym_id != NO_NODE) {
    std::cerr
        << "ERROR: there is already a joint named '" << joint_name << "'"
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  // check body_name
  if (body_name == "NP") {
    std::cerr
        << "ERROR: one cannot name a body 'NP'. This name stands for "
        << "'no parent' and is reserved"
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  // find an homonym body
  int body_homonym_id = model_.find_link_by_body_name(body_name);
  if (body_homonym_id != NO_NODE) {
    std::cerr
        << "ERROR: there is already a body named '" << body_name << "'"
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  // check node name is ok.
  std::string node_name = ::node_name(joint_name, body_name);
  if (!::isValidIdentifier(node_name)) {
    std::cerr
        << "ERROR: node name \"" << node_name << "\" is invalid."
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  // TODO: check joint_Xt_E is a real rotation matrix

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
  switch (joint_type) {
  case RobotBuilder::REVOLUTE_AXIS_ANY:
  case RobotBuilder::REVOLUTE_AXIS_X:
  case RobotBuilder::REVOLUTE_AXIS_Y:
  case RobotBuilder::REVOLUTE_AXIS_Z:
  case RobotBuilder::FREE_FLYER:
    break;
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
                       fwdDyn,
                       joint_position_in_conf));
  nb_dof_ += model_.joint_dof(link_id);
  return RobotBuilder::STATUS_SUCCESS;
}

void RobotBuilderP::writeLink(int link_id, const ReplMap &replacements,
                              TmpStreams &out) const
{
  std::string joint_type, joint_rotation_type;
  switch(model_.joint_type(link_id))
    {
    case metapod::RobotBuilder::FREE_FLYER:
      joint_type = "FreeFlyerJoint<FloatType>";
      joint_rotation_type = "Spatial::RotationMatrixTpl<FloatType>";
      break;
    case metapod::RobotBuilder::REVOLUTE_AXIS_X:
      joint_type = "RevoluteAxisXJoint<FloatType>";
      joint_rotation_type = "Spatial::RotationMatrixAboutXTpl<FloatType>";
      break;
    case metapod::RobotBuilder::REVOLUTE_AXIS_Y:
      joint_type = "RevoluteAxisYJoint<FloatType>";
      joint_rotation_type = "Spatial::RotationMatrixAboutYTpl<FloatType>";
      break;
    case metapod::RobotBuilder::REVOLUTE_AXIS_Z:
      joint_type = "RevoluteAxisZJoint<FloatType>";
      joint_rotation_type = "Spatial::RotationMatrixAboutZTpl<FloatType>";
      break;
    case metapod::RobotBuilder::REVOLUTE_AXIS_ANY:
      joint_type = "RevoluteAxisAnyJoint<FloatType>";
      joint_rotation_type = "Spatial::RotationMatrixTpl<FloatType>";
      break;
    }

  Eigen::IOFormat comma_fmt(Eigen::StreamPrecision, Eigen::DontAlignCols,
                            ", ", ", ");
  const int parent_id = model_.parent_id(link_id);
  ReplMap repl(replacements);
  repl["node_id"] = ::to_string(link_id);
  repl["node_name"] = ::node_name(model_, link_id);
  repl["dof_index"] = ::to_string(model_.dof_index(link_id));
  repl["joint_type"] = joint_type;
  repl["joint_rotation_type"] = joint_rotation_type;
  repl["joint_name"] = model_.joint_name(link_id);
  repl["jointFwdDyn"] = ::to_string(model_.fwdDyn(link_id));
  
  const Eigen::Matrix3d &R_joint_parent = model_.R_joint_parent(link_id);
  if (R_joint_parent.isApprox(Eigen::Matrix3d::Identity())) {
    repl["R_joint_parent_type"] = "Spatial::RotationMatrixIdentityTpl<FloatType>";
    repl["X_joint_parent_type"] = "Spatial::TransformT<FloatType, Spatial::RotationMatrixIdentityTpl<FloatType> >";
    repl["R_joint_parent"] = "Spatial::RotationMatrixIdentityTpl<FloatType>()";
  } else {
    repl["R_joint_parent_type"] = "Spatial::RotationMatrixTpl<FloatType>";
    repl["X_joint_parent_type"] = "Spatial::TransformT<FloatType, Spatial::RotationMatrixTpl<FloatType> >";
    std::stringstream ss0;
    ss0 << "matrix3dMaker<FloatType>("
        << model_.R_joint_parent(link_id).format(comma_fmt)
        << ")";
    repl["R_joint_parent"] = ss0.str();
  }
  std::stringstream ss1;
  ss1 << "Vector3dTpl<FloatType>::Type("
      << model_.r_parent_joint(link_id).format(comma_fmt)
      << ")";
  repl["r_parent_joint"] = ss1.str();
  repl["body_name"] = model_.body_name(link_id);
  repl["body_mass"] = ::to_string(model_.body_mass(link_id));
  std::stringstream ss2;
  ss2 << "Vector3dTpl<FloatType>::Type("
      << model_.body_center_of_mass(link_id).format(comma_fmt)
      << ")";
  repl["body_center_of_mass"] = ss2.str();
  std::stringstream ss3;
  ss3 << "matrix3dMaker<FloatType>("
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
      "  class @LIBRARY_NAME@_DLLAPI Node@node_id@ {\n"
      "  public:\n"
      "    Node@node_id@();\n"
      "    static const int id = @node_id@;\n"
      "    static const std::string joint_name;\n"
      "    static const bool jointFwdDyn = @jointFwdDyn@; // <dynamics> fwd_dyn field, used by chda\n"
      "    static const bool jointNuOfFwdDyn = initNuFwdDyn< @ROBOT_CLASS_NAME@<FloatType>, @ROBOT_CLASS_NAME@<FloatType>::Node@node_id@ >::value; // subtree supported by at least one fwdDyn joint\n"
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
      "    Spatial::TransformT<FloatType, typename Spatial::rm_mul_op<FloatType,@joint_rotation_type@, @R_joint_parent_type@ >::rm> sXp;\n"
      "    Eigen::Matrix<FloatType, 6, Joint::NBDOF> joint_F; // used by crba\n"
      "    Joint joint;\n"
      "    Body<FloatType> body;\n"
      "  };\n");
  out.node_type_definitions << tpl1.format(repl);

  const TxtTemplate tpl2(
                         "      Node@node_id@");
  out.nodes_type_list << tpl2.format(repl);
  if (!is_last_link)
    out.nodes_type_list << ",\n";

  const TxtTemplate tpl3(
                         "template <typename FloatType> struct Nodes < @ROBOT_CLASS_NAME@<FloatType>, @node_id@> "
                         "{typedef typename @ROBOT_CLASS_NAME@<FloatType>::Node@node_id@ type;};\n");
  out.map_node_id_to_type << tpl3.format(repl);

  // fill bits for init.cc
  const TxtTemplate tpl4(
      "typedef double FloatType;\n"
      "template <> const std::string @ROBOT_CLASS_NAME@<FloatType>::Node@node_id@::joint_name = std::string(\"@joint_name@\");\n"
      "template <> const std::string @ROBOT_CLASS_NAME@<FloatType>::Node@node_id@::body_name = std::string(\"@body_name@\");\n"
      "template <> const @X_joint_parent_type@ @ROBOT_CLASS_NAME@<FloatType>::Node@node_id@::Xt = @X_joint_parent_type@(\n"
      "    @R_joint_parent@,\n"
      "    @r_parent_joint@);\n"
      "template <> @ROBOT_CLASS_NAME@<FloatType>::Node@node_id@::Node@node_id@():\n"
      "  joint(@joint_args@) {}\n\n");
  if (model_.joint_type(link_id) == RobotBuilder::REVOLUTE_AXIS_ANY)
    {
      std::stringstream ss;
      ss << model_.joint_axis(link_id).format(comma_fmt);
      repl["joint_args"] = ss.str();
    }
  out.init_nodes << tpl4.format(repl);
  const TxtTemplate tpl5(
      "    spatialInertiaMaker<FloatType>(\n"
      "        @body_mass@,\n"
      "        @body_center_of_mass@,\n"
      "        @body_rotational_inertia@),\n");
  out.init_inertias << tpl5.format(repl);
}

RobotBuilder::Status RobotBuilderP::write() const
{
  if (!is_initialized_) {
    std::cerr
        << "ERROR: the robot has no link."
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }

  // check name and libname
  if (name_.empty()) {
    std::cerr
        << "ERROR: the robot name has not been provided."
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }
  if (libname_.empty()) {
    std::cerr
        << "ERROR: the robot library name has not been provided."
        << std::endl;
    return RobotBuilder::STATUS_FAILURE;
  }

  // create the directory (and its parents if necessary)
  boost::filesystem::create_directories(directory_);
#ifdef _MSC_VER
  // by default, MC VS prints numbers in scentific format with 3 digits for
  // the exponent. Eg: 3.2110E-005. We only want two digits like other
  // platforms do. Eg: 3.2110E-05.
  unsigned int old_exponent_format = _set_output_format(_TWO_DIGIT_EXPONENT);
#endif

  // fill the replacements we already know
  ReplMap repl;
  std::string libname_uc(libname_);
  std::transform(libname_.begin(), libname_.end(), libname_uc.begin(),
                 ::toupper);
  repl["LIBRARY_NAME"] = libname_uc;
  std::stringstream export_symbol;
  export_symbol << libname_ << "_EXPORTS";
  repl["EXPORT_SYMBOL"] = export_symbol.str();
  repl["ROBOT_CLASS_NAME"] = name_;
  repl["ROBOT_NAME"] = name_;
  repl["LICENSE"] = license_;
  repl["ROBOT_NB_DOF"] = ::to_string(nb_dof_);
  repl["ROBOT_NB_BODIES"] = ::to_string(model_.nb_links());

  // add the links to the temporary streams
  TmpStreams streams;
  for (int i = 0; i != model_.nb_links(); ++i)
    writeLink(i, repl, streams);

  // complete the replacements map
  repl["nodeid_enum_definition"] = streams.nodeid_enum_definition.str();
  repl["node_type_definitions"] = streams.node_type_definitions.str();
  repl["nodes_type_list"] = streams.nodes_type_list.str();
  repl["fwdDyn_joints_dof"] = ::to_string(model_.fwdDyn_joints_dof());
  repl["map_node_id_to_type"] = streams.map_node_id_to_type.str();

  for (int i = 0; i<MAX_NB_CHILDREN_PER_NODE; ++i) {
    std::stringstream key;
    key << "root_child" << i << "_id";
    repl[key.str()] = ::to_string(model_.child_id(NO_PARENT, i));
  }

  // init.cc
  repl["init_nodes"] = streams.init_nodes.str();
  repl["init_inertias"] = streams.init_inertias.str();

  // generate files from template and replacements
  const std::string config_hh_templ(::config_hh, ::config_hh_len);
  writeTemplate("config.hh", config_hh_templ, repl);

  const std::string init_hh_templ(::init_hh, ::init_hh_len);
  writeTemplate(name_ + ".hh", init_hh_templ, repl);

  const std::string init_cc_templ(::init_cc, ::init_cc_len);
  writeTemplate(name_ + ".cc", init_cc_templ, repl);
#ifdef _MSC_VER
  // restore orginal exponent formatting (warning: this may never be reached
  // if an exception is thrown in between).
  _set_output_format(old_exponent_format);
#endif
  return RobotBuilder::STATUS_SUCCESS;
}

}
