// Copyright 2013
//
// Olivier STASSE, (LAAS, CNRS)
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
 * This file contains the includes and class definitions necessary for the
 * whole project.
 */

#ifndef METAPOD_ROBOT_BASE_HH
# define METAPOD_ROBOT_BASE_HH

# include <boost/fusion/sequence.hpp>
# include <boost/fusion/include/sequence.hpp>
# include <boost/fusion/include/vector.hpp>

#include <metapod/tools/depth_first_traversal.hh>

namespace metapod
{
  template <typename DerivedClass> class RobotBase;
  template <typename NodeDerivedClass> class NodeBase;
  
  template <typename NodeDerivedClass>
  struct print_node_base
  {
    std::ostream &print(std::ostream &os,
                        const NodeDerivedClass &aNodeDerived)  const
    {
      os << "ID: " << aNodeDerived.id << std::endl;
      os << "Joint name:" << aNodeDerived.joint_name << std::endl;
      os << "Body name :" << aNodeDerived.body_name  << std::endl;
      os << "Xt: " << aNodeDerived.Xt << std::endl;
      os << "q_idx: " << aNodeDerived.q_idx << std::endl;
      os << "mass : " << aNodeDerived.mass << std::endl;
      os << "I: " << aNodeDerived.I << std::endl;
      os << "sXp: " << aNodeDerived.sXp << std::endl;
      os << "joint_F: " << aNodeDerived.joint_F << std::endl;
      //      os << "Joint : " << aNodeDerived.joint << std::endl;
      //      os << "Body : " << aNodeDerived.body << std::endl;
      return os;
    }
  };

  template <typename NodeDerivedClass>
  struct NodeBase
  {
    friend std::ostream &operator<<(std::ostream &os, 
                                    const NodeDerivedClass &aNodeDerived) 
    {
      print_node_base<NodeDerivedClass> aprinter;
      return aprinter.print(os,aNodeDerived);
    }
  };
  
  namespace internal {
    /**! \addtogroup RobotBase 
       \brief Template to display recursively information on the robot. */
    template < typename Robot, int node_id> struct print_nodes
    {
      typedef typename Nodes<Robot, node_id>::type Node;
      
      static void discover(const Robot& robot, std::ostream & os)
      {
        const Node& node = boost::fusion::at_c<node_id>(robot.nodes);
        os << node;
      }
      static void finish(const Robot&, std::ostream & ) {}
    };

    /** \addtogroup RobotBase 
       ! \brief Template to compute the mass from the description. */
    template < typename Robot, int node_id> struct compute_mass
    {
      typedef typename Nodes<Robot, node_id>::type Node;
      
      static void discover(FloatType & lmass)
      {
        lmass += Node::mass ;
      }
      static void finish(FloatType & ) {}
    };

  } // end of namespace metapod::internal.
  
  
  template  <typename DerivedClass> 
  struct print_robot_base
  {

    std::ostream & print(std::ostream &os,
                         const DerivedClass &aDerivedRobot)
    {
      os << "Nb dofs: " << DerivedClass::NBDOF << std::endl;
      os << "Nb of bodies: " << DerivedClass::NBBODIES << std::endl;
  
      depth_first_traversal<internal::print_nodes, DerivedClass>::run(aDerivedRobot, os);
      
      os << "H:" << aDerivedRobot.H << std::endl;
      return os;
    }
  };

  /** \addtogroup RobotBase RobotClass template is providing common helper methods
      to robots in general. Based upon the information specific to the robot it
      computes various classical information such as mass, center of mass vectors.
   */
  template <typename DerivedClass>
  class METAPOD_DLLAPI RobotBase
  {
  private:
    /** ! \brief Mass of the robot. */
    FloatType mass_;

  public:
    friend struct print_robot_base<DerivedClass>;
    
    /** \brief Default constructor of RobotBase. */
    RobotBase(): 
      mass_(0)
      {
        depth_first_traversal<internal::compute_mass,DerivedClass>::run(mass_);
      }
    
    friend std::ostream &operator<<(std::ostream &os, 
                                    const DerivedClass &arobot)
    {
      print_robot_base<DerivedClass> aprinter;
      return aprinter.print(os,arobot);
    }

  };

}

#endif
