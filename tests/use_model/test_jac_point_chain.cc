// Copyright 2012,
//
// Antonio El Khoury (JRL/LAAS, CNRS/AIST)
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
 * This test computes the jacobian on a test model with a reference
 * configuration, then compares the computed jacobian with the
 * reference jacobian
 */

// Common test tools
# include "../common.hh"

using namespace metapod;
using namespace CURRENT_MODEL_NAMESPACE;

namespace metapod
{
  /// \addtogroup jac_point_chain_robot Point in Chain Articular Jacobian Test Algorithm
  ///
  /// Compute point articular jacobian for all combinations robot
  /// bodies by calling recursively jac_point_chain. This algorithm is
  /// not optimal and is not meant to be used except for testing
  /// purposes.
  ///
  /// \{

  template< typename Robot, typename Tree, bool bcalc = true >
  struct jac_point_chain_robot_internal_loop1;

  template< typename Robot, typename Tree1, typename Tree2, int rootNbDof,
            bool bcalc = true >
  struct jac_point_chain_robot_internal_loop2;

  /// \brief Point articular jacobian algorithm.
  ///
  /// \tparam Robot Robot type for which jacobian for all bodies is
  /// computed.
  ///
  /// \tparam bcalc Boolean type to specify whether all body
  /// transforms need to be updated with respect to the robot
  /// configuration.
  template< typename Robot, bool bcalc = true >
  struct jac_point_chain_robot
  {
    typedef Eigen::Matrix< FloatType, 6*Robot::NBBODIES*Robot::NBBODIES,
                           Robot::NBDOF, Eigen::RowMajor >
    jacobian_t;

    static void run(const typename Robot::confVector & q,
                    jacobian_t & J)
    {
      J.setZero ();

      // Call internal jacobian routine.
      jac_point_chain_robot_internal_loop1< Robot, typename Robot::Tree,
        bcalc >::run(q, J);
    }
  };

  /// \}
  ///

  template< typename Robot, typename Tree, bool bcalc >
  struct jac_point_chain_robot_internal_loop1
  {
    typedef typename jac_point_chain_robot< Robot, bcalc>::jacobian_t
    robotJacobian_t;

    typedef Tree Node;

    static void run(const typename Robot::confVector & q,
                    robotJacobian_t & J)
    {
      // Call second internal loop.
      jac_point_chain_robot_internal_loop2< Robot, Tree, typename Robot::Tree,
        Robot::Tree::Joint::NBDOF, bcalc >::run(q, J);

      // recursion on children
      jac_point_chain_robot_internal_loop1< Robot,
        typename Node::Child0, bcalc >::run(q, J);
      jac_point_chain_robot_internal_loop1< Robot,
        typename Node::Child1, bcalc >::run(q, J);
      jac_point_chain_robot_internal_loop1< Robot,
        typename Node::Child2, bcalc >::run(q, J);
      jac_point_chain_robot_internal_loop1< Robot,
        typename Node::Child3, bcalc >::run(q, J);
      jac_point_chain_robot_internal_loop1< Robot,
        typename Node::Child4, bcalc >::run(q, J);
    }
  };

  /// \brief Specialization of jac_point_chain_robot_internal_loop1: Stop
  /// recursion on first tree leaves.
  template< typename Robot, bool bcalc >
  struct jac_point_chain_robot_internal_loop1< Robot, NC, bcalc>
  {
    typedef typename jac_point_chain_robot< Robot, bcalc>::jacobian_t
    robotJacobian_t;

    static void run(const typename Robot::confVector &,
                    robotJacobian_t &) {}
  };

  template< typename Robot, typename Tree1, typename Tree2, int rootNbDof,
            bool bcalc >
  struct jac_point_chain_robot_internal_loop2
  {
    typedef typename jac_point_chain_robot< Robot, bcalc>::jacobian_t
    robotJacobian_t;
    typedef typename jac_point_chain< Robot,
                                      typename Tree1::Body,
                                      typename Tree2::Body,
                                      0,
                                      true,
                                      bcalc>::jacobian_t
    bodyJacobian_t;

    typedef Tree1 Node1;
    typedef Tree2 Node2;
    typedef typename Node1::Body Body1;
    typedef typename Node2::Body Body2;

    static void run(const typename Robot::confVector & q,
                    robotJacobian_t & J)
    {
      // Compute jacobian sub-block.
      bodyJacobian_t subJ = bodyJacobian_t::Zero();
      jac_point_chain< Robot, Body1, Body2, 6, false, bcalc >
        ::run(q, vector3d(0,0,0), subJ);
      J.template block<6,Robot::NBDOF>
        (6*Robot::NBBODIES*Body1::label + 6*Body2::label, 0) = subJ;

      // recursion on children
      jac_point_chain_robot_internal_loop2< Robot, Tree1,
        typename Node2::Child0, rootNbDof, bcalc >::run(q, J);
      jac_point_chain_robot_internal_loop2< Robot, Tree1,
        typename Node2::Child1, rootNbDof,  bcalc >::run(q, J);
      jac_point_chain_robot_internal_loop2< Robot, Tree1,
        typename Node2::Child2, rootNbDof,  bcalc >::run(q, J);
      jac_point_chain_robot_internal_loop2< Robot, Tree1,
        typename Node2::Child3, rootNbDof,  bcalc >::run(q, J);
      jac_point_chain_robot_internal_loop2< Robot, Tree1,
        typename Node2::Child4, rootNbDof,  bcalc >::run(q, J);
    }
  };

  /// \brief Specialization of jac_point_chain_robot_internal_loop2:
  /// handle case wherer root joint is free-flyer joint separately.
  template< typename Robot, typename Tree1, typename Tree2, bool bcalc >
  struct jac_point_chain_robot_internal_loop2< Robot, Tree1, Tree2, 6, bcalc>
  {
    typedef typename jac_point_chain_robot< Robot, bcalc>::jacobian_t
    robotJacobian_t;
    typedef typename jac_point_chain< Robot,
                                      typename Tree1::Body,
                                      typename Tree2::Body,
                                      0,
                                      true,
                                      bcalc>::jacobian_t
    bodyJacobian_t;

    typedef Tree1 Node1;
    typedef Tree2 Node2;
    typedef typename Node1::Body Body1;
    typedef typename Node2::Body Body2;

    static void run(const typename Robot::confVector & q,
                    robotJacobian_t & J)
    {
      // Compute jacobian sub-block.
      bodyJacobian_t subJ = bodyJacobian_t::Zero();
      jac_point_chain< Robot, Body1, Body2, 0, true, bcalc >
        ::run(q, vector3d(0,0,0), subJ);
      J.template block<6,Robot::NBDOF>
        (6*Robot::NBBODIES*Body1::label + 6*Body2::label, 0) = subJ;

      // recursion on children
      jac_point_chain_robot_internal_loop2< Robot, Tree1,
        typename Node2::Child0, 6, bcalc >::run(q, J);
      jac_point_chain_robot_internal_loop2< Robot, Tree1,
        typename Node2::Child1, 6, bcalc >::run(q, J);
      jac_point_chain_robot_internal_loop2< Robot, Tree1,
        typename Node2::Child2, 6, bcalc >::run(q, J);
      jac_point_chain_robot_internal_loop2< Robot, Tree1,
        typename Node2::Child3, 6, bcalc >::run(q, J);
      jac_point_chain_robot_internal_loop2< Robot, Tree1,
        typename Node2::Child4, 6, bcalc >::run(q, J);
    }
  };

  /// \brief Specialization of jac_point_chain_robot_internal_loop2: Stop
  /// recursion on second tree leaves.
  template< typename Robot, typename Tree1, int rootNbDof, bool bcalc >
  struct jac_point_chain_robot_internal_loop2< Robot, Tree1, NC, rootNbDof,
                                               bcalc>
  {
    typedef typename jac_point_chain_robot< Robot, bcalc>::jacobian_t
    robotJacobian_t;

    static void run(const typename Robot::confVector &,
                    robotJacobian_t &) {}
  };

  /// \brief Specialization of jac_point_chain_robot_internal_loop2:
  /// Stop recursion on second tree leaves wheen root joint is
  /// free-flyer joint.
  template< typename Robot, typename Tree1, bool bcalc >
  struct jac_point_chain_robot_internal_loop2< Robot, Tree1, NC, 6, bcalc>
  {
    typedef typename jac_point_chain_robot< Robot, bcalc>::jacobian_t
    robotJacobian_t;

    static void run(const typename Robot::confVector &,
                    robotJacobian_t &) {}
  };
}

BOOST_AUTO_TEST_CASE (test_jac_point_chain_robot)
{
  // Set configuration vectors (q) to reference values.
  Robot::confVector q;
  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  initConf< Robot >::run(qconf, q);
  qconf.close();

  // Compute the jacobian and print the result in a log file.
  jac_point_chain_robot< Robot >::jacobian_t J =
      jac_point_chain_robot< Robot >::jacobian_t::Zero();
  jac_point_chain_robot< Robot >::run(q, J);
  const char result_file[] = "jac_point_chain_robot.log";
  std::ofstream log(result_file, std::ofstream::out);
  log << J << std::endl;;
  log.close();

  // Compare results with reference file
  compareLogs(result_file, TEST_DIRECTORY "/jac_point_chain_robot.ref", 1e-3);
}
