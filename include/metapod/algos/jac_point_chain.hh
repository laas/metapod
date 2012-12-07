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

/// Implementation of articular jacobian of point given an influencing
/// chain of joints.

#ifndef METAPOD_JAC_POINT_CHAIN_HH
# define METAPOD_JAC_POINT_CHAIN_HH

# include <metapod/tools/common.hh>
# include <metapod/tools/deepest_common_body.hh>
# include <metapod/algos/jac_point.hh>

namespace metapod
{

  /// \addtogroup jac_point_chain Point in Chain Articular Jacobian Algorithm
  ///
  /// Compute articular jacobian J of a point (in body coordinates)
  /// attached to a body in a specific sub-chain of the robot
  /// kinematic tree. This jacobian is such that v = J*dq is the point
  /// frame spatial motion vector in world coordinates, under the sole
  /// influence of joints in the sub-chain and (optionally) a fictive
  /// free-flyer joint at the base of the sub-chain.
  ///
  /// Such a jacobian can be used to control the relative pose of a body
  /// with respect to another body.
  ///
  /// \{

  template< typename Robot, typename Body,
            unsigned int offset = 0, bool includeFreeFlyer = true >
  struct jac_point_chain_internal_start;

  template< typename Robot, typename Body,
            unsigned int offset = 0, bool includeFreeFlyer = true >
  struct jac_point_chain_internal_end;

  template< typename Robot, typename StartBody,
            unsigned int offset = 0, bool includeFreeFlyer = true >
  struct jac_point_chain_internal_freeflyer;

  /// \brief Point in Chain Articular Jacobian Algorithm.
  ///
  /// \tparam Robot Robot type for which jacobian is computed.
  /// \tparam StartBody Start body type in sub-chain for which
  /// jacobian is computed.
  /// \tparam EndBody End body type in sub-chain for which
  /// jacobian is computed.
  /// \tparam offset Rank of first non zero jacobian column.
  /// \tparam includeFreeFlyer Boolean type to specify the
  /// contribution of a fictive free-flyer joint superposed with the
  /// joint of StartBody.
  /// \tparam call_bcalc Boolean type to specify whether all body
  /// transforms need to be updated with respect to the robot
  /// configuration.
  template< typename Robot, typename StartBody, typename EndBody,
            unsigned int offset = 0, bool includeFreeFlyer = true,
            bool call_bcalc = true >
  struct jac_point_chain {
    typedef Eigen::Matrix< FloatType, 6,
                           Robot::NBDOF
                           + offset
                           - 6*(1-includeFreeFlyer) >
    jacobian_t;

    /// \brief Compute the articular jacobian J.
    ///
    /// \param q Configuration vector: it is used to update all body
    /// spatial transforms if bcalc is equal to true.
    ///
    /// \param e_p Coordinates of point in EndBody coordinates.
    /// \retval J Computed jacobian of size 6x(NBDOF+offset) if
    /// free-flyer is included, 6x(NBDOF-6+offset) otherwise.
    static void run(const typename Robot::confVector & q,
                    const vector3d & e_p,
                    jacobian_t & J)
    {
      // Update body transformations.
      if (call_bcalc)
      {
        bcalc < Robot >::run(q);
      }

      // Compute point coordinates in world frame.
      vector3d p = EndBody::iX0.applyInv(e_p);

      // Get deepest common body label.
      int label;
      deepest_common_body< StartBody, EndBody >::run(label);

      // Call internal jacobian routines.
      jac_point_chain_internal_start< Robot, StartBody,
        offset, includeFreeFlyer >::run(label, p, J);
      jac_point_chain_internal_end< Robot, EndBody,
        offset, includeFreeFlyer >::run(label, p, J);
      jac_point_chain_internal_freeflyer< Robot, StartBody,
        offset, includeFreeFlyer >::run(p, J);
    }
  };
  /// \}

  /// \brief Internal point in chain articular jacobian algorithm
  /// routine.
  ///
  /// Compute root to start body contribution in jacobian.
  ///
  /// \sa jac_point_chain
  template< typename Robot, typename Body,
            unsigned int offset, bool includeFreeFlyer >
  struct jac_point_chain_internal_start
  {
    typedef typename Body::Joint Joint;
    typedef typename jac_point_chain<Robot, Body, Body,
                                     offset, includeFreeFlyer >::jacobian_t
    jacobian_t;

    static void run(const int & label,
                    const vector3d & p,
                    jacobian_t & J) __attribute__ ((hot))
    {
      // FIXME: Stop condition: avoid if condition and use template
      // specialization instead.
      if (label == Body::label)
        return;

      // Compute jacobian block for current node. Formula is given by:
      // Ji = - pX0 * (iX0)^(-1) * Si,
      // where pX0 is the word transform in the point frame,
      // iX0 is the world transform in the ith body frame,
      // Si is the ith joint motion subspace matrix.
      J.template
        block<6,Joint::NBDOF>(0,Joint::positionInConf
                              + offset
                              - 6*(1 - includeFreeFlyer)) =
        - Joint::applyToS(Body::iX0.inverse ().toPointFrame (p));

      // Recurse over body parent.
      jac_point_chain_internal_start<Robot, typename Body::Parent,
        offset, includeFreeFlyer >::run(label, p, J);
    }
  };

  /// \brief Specialization of jac_point_chain_internal_start: stop
  /// recursion at root of the Tree.
  template< typename Robot, unsigned int offset, bool includeFreeFlyer >
  struct jac_point_chain_internal_start< Robot, NP, offset, includeFreeFlyer >
  {
    typedef typename jac_point_chain<Robot, NP, NP,
                                     offset, includeFreeFlyer >::jacobian_t
    jacobian_t;

    static void run(const int & label,
                    const vector3d &,
                    jacobian_t &) {}
  };

  /// \brief Internal point in chain articular jacobian algorithm
  /// routine.
  ///
  /// Compute root to end body contribution in jacobian.
  ///
  /// \sa jac_point_chain
  template< typename Robot, typename Body,
            unsigned int offset, bool includeFreeFlyer >
  struct jac_point_chain_internal_end
  {
    typedef typename Body::Joint Joint;
    typedef typename jac_point_chain<Robot, Body, Body,
                                     offset, includeFreeFlyer >::jacobian_t
    jacobian_t;

    static void run(const int & label,
                    const vector3d & p,
                    jacobian_t & J) __attribute__ ((hot))
    {
      // FIXME: Stop condition: avoid if condition and use template
      // specialization instead.
      if (label == Body::label)
        return;

      // Compute jacobian block for current node. Formula is given by:
      // Ji = pX0 * (iX0)^(-1) * Si,
      // where pX0 is the word transform in the point frame,
      // iX0 is the world transform in the ith body frame,
      // Si is the ith joint motion subspace matrix.
      J.template
        block<6,Joint::NBDOF>(0,Joint::positionInConf
                              + offset
                              - 6*(1 - includeFreeFlyer)) =
        Joint::applyToS(Body::iX0.inverse ().toPointFrame (p));

      // Recurse over body parent.
      jac_point_chain_internal_end< Robot, typename Body::Parent,
        offset, includeFreeFlyer >::run(label, p, J);
    }
  };

  /// \brief Specialization of jac_point_chain_internal_end: stop
  /// recursion at root of the Tree.
  template< typename Robot, unsigned int offset, bool includeFreeFlyer >
  struct jac_point_chain_internal_end< Robot, NP, offset, includeFreeFlyer >
  {
    typedef typename jac_point_chain<Robot, NP, NP,
                                     offset, includeFreeFlyer >::jacobian_t
    jacobian_t;

    static void run(const int,
                    const vector3d &,
                    jacobian_t &) {}
  };

  /// \brief Internal point in chain articular jacobian algorithm
  /// routine.
  ///
  /// Compute fictive free-flyer joint contribution if it is included
  /// in jacobian.
  ///
  /// \sa jac_point_chain
  template< typename Robot, typename StartBody, unsigned int offset >
  struct jac_point_chain_internal_freeflyer< Robot, StartBody, offset, true>
  {
    typedef typename jac_point_chain<Robot, StartBody, StartBody,
                                     offset, true >::jacobian_t
    jacobian_t;

    static void run(const vector3d & p,
                    jacobian_t & J)
    {
      // Compute jacobian block for freeflyer. Formula is given by:
      // Ji = pX0 * (sX0)^(-1) * Sff,
      // where pX0 is the word transform in the point frame,
      // sX0 is the world transform in the start body frame,
      // Sff is the motion subspace matrix of a fictive free-flyer
      // located at the start body joint.
      J.template block<3,3>(0,3+offset) = matrix3d::Identity ();
      J.template block<3,3>(3,offset) = matrix3d::Identity ();
      Spatial::Transform pXs = StartBody::iX0.inverse ().toPointFrame (p);
      J.template block<3,3>(3,3+offset) = Spatial::skew (- pXs.E() * pXs.r());
    }
  };

  /// \brief Specialization of jac_point_chain_internal_free-flyer: do
  /// nothing if freeflyer joint is not included in jacobian.
  template< typename Robot, typename StartBody, unsigned int offset >
  struct jac_point_chain_internal_freeflyer< Robot, StartBody, offset, false>
  {
    typedef typename jac_point_chain<Robot, StartBody, StartBody,
                                     offset, false >::jacobian_t
    jacobian_t;

    static void run(const vector3d &,
                    jacobian_t &) {}
  };


} // end of namespace metapod.

#endif
