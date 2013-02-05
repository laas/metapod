#include "robotmodel.hh"
#include <cassert>
#include <metapod/tools/constants.hh>
#include <limits>

namespace {

class CompareLinkBodyName
{
public:
  CompareLinkBodyName(const std::string& name):
      name_(name)
  {}
  bool operator()(const metapod::Link& link)
  {
    if (link.body_name_ == name_)
      return true;
    else
      return false;
  }
private:
  const std::string name_;
};

class CompareLinkJointName
{
public:
  CompareLinkJointName(const std::string& name):
      name_(name)
  {}
  bool operator()(const metapod::Link& link)
  {
    if (link.joint_name_ == name_)
      return true;
    else
      return false;
  }
private:
  const std::string name_;
};

}

namespace metapod {

Link::Link(
    int id,
    int parent_id,
    const std::string& joint_name,
    unsigned int joint_type,
    const Eigen::Matrix3d & R_joint_parent,
    const Eigen::Vector3d & r_parent_joint,
    const std::string& body_name,
    double body_mass,
    const Eigen::Vector3d & body_center_of_mass,
    const Eigen::Matrix3d & body_rotational_inertia,
    const Eigen::Vector3d & joint_axis,
    int dof_index):
  id_(id),
  parent_id_(parent_id),
  joint_name_(joint_name),
  joint_type_(joint_type),
  R_joint_parent_(R_joint_parent),
  r_parent_joint_(r_parent_joint),
  body_name_(body_name),
  body_mass_(body_mass),
  body_center_of_mass_(body_center_of_mass),
  body_rotational_inertia_(body_rotational_inertia),
  joint_axis_(joint_axis),
  dof_index_(dof_index)
{}


const std::string RobotModel::NP_ = "NP";

int RobotModel::nb_links() const
{
  return static_cast<int>(links_.size());
}
int RobotModel::parent_id(int link_id) const
{
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].parent_id_;
}

const std::string& RobotModel::joint_name(int link_id) const
{
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].joint_name_;
}

int RobotModel::joint_type(int link_id) const
{
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].joint_type_;
}

const Eigen::Matrix3d& RobotModel::R_joint_parent(int link_id) const
{
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].R_joint_parent_;
}

const Eigen::Vector3d& RobotModel::r_parent_joint(int link_id) const
{
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].r_parent_joint_;
}

const std::string& RobotModel::body_name(int link_id) const
{
  if (link_id == NO_PARENT)
    return NP_;
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].body_name_;
}

double RobotModel::body_mass(int link_id) const
{
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].body_mass_;
}

const Eigen::Vector3d& RobotModel::body_center_of_mass(int link_id) const
{
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].body_center_of_mass_;
}

const Eigen::Matrix3d& RobotModel::body_rotational_inertia(int link_id) const
{
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].body_rotational_inertia_;
}

const Eigen::Vector3d& RobotModel::joint_axis(int link_id) const
{
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].joint_axis_;
}

int RobotModel::dof_index(int link_id) const
{
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return links_[link_id].dof_index_;
}

int RobotModel::nb_children(int link_id) const
{
  if (link_id == NO_PARENT)
    return static_cast<int>(roots_id_.size());
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  return static_cast<int>(links_[link_id].child_id_.size());
}

int RobotModel::child_id(int link_id, unsigned int rank) const
{
  if (link_id == NO_PARENT)
  {
    if (rank < roots_id_.size())
      return roots_id_[rank];
    return NO_CHILD;
  }
  assert(link_id >= 0 && static_cast<size_t>(link_id) < links_.size());
  if (rank < links_[link_id].child_id_.size())
    return links_[link_id].child_id_[rank];
  return NO_CHILD;
}

void RobotModel::add_link(const Link& link)
{
  // we assume the caller as filled the link with the proper id.
  // We might want to change this policy in the following way: set the id
  // ourselves and return it to the caller.
  assert(link.id_ == static_cast<int>(links_.size()));
  // we use int as index type but store links in a std::vector which uses
  // size_t as index type. Check we won't overflow.
  assert(links_.size() < static_cast<size_t>(std::numeric_limits<int>::max()));

  const int parent_id = link.parent_id_;
  if (parent_id == NO_PARENT)
  {
    // we use int as index type but store links in a std::vector which uses
    // size_t as index type. Check we won't overflow.
    assert(roots_id_.size() <
        static_cast<size_t>(std::numeric_limits<int>::max()));
    roots_id_.push_back(link.id_);
  }
  else
  {
    assert(parent_id >= 0 && static_cast<size_t>(parent_id) < links_.size());
    // we use int as index type but store links in a std::vector which uses
    // size_t as index type. Check we won't overflow.
    assert(links_[parent_id].child_id_.size() <
        static_cast<size_t>(std::numeric_limits<int>::max()));
    links_[parent_id].child_id_.push_back(link.id_);
  }
  links_.push_back(link);
}

int RobotModel::find_link_by_body_name(const std::string& name) const
{
  if (name == "NP")
    return NO_PARENT;
  std::vector<Link>::const_iterator it =
      std::find_if(links_.begin(), links_.end(), ::CompareLinkBodyName(name));
  if (it == links_.end())
    return NO_NODE;
  return it->id_;
}

int RobotModel::find_link_by_joint_name(const std::string& name) const
{
  std::vector<Link>::const_iterator it =
      std::find_if(links_.begin(), links_.end(), ::CompareLinkJointName(name));
  if (it == links_.end())
    return NO_NODE;
  return it->id_;
}

}
