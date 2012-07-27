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
 * This file contains print methods.
 */

#ifndef METAPOD_PRINT_HH
# define METAPOD_PRINT_HH

#include "metapod/tools/common.hh"

namespace metapod
{

// Print state of the robot in a stream.
template< typename Tree >
void printState(std::ostream & os)
{
  typedef Tree Node;

  os << Node::Body::name << " :" << std::endl
    << "sXp :\n" << Node::Joint::sXp << std::endl
    << "Xt :\n" << Node::Joint::Xt << std::endl
    << "Xj :\n" << Node::Joint::Xj << std::endl
    << "S :\n" << Node::Joint::S << std::endl
    << "dotS :\n" << Node::Joint::dotS << std::endl
    << "iX0 :\n" << Node::Body::iX0 << std::endl
    << "vi :\n" << Node::Body::vi << std::endl
    << "ai :\n" << Node::Body::ai << std::endl
    << "I :\n" << Node::Body::I << std::endl
    << "f :\n" << Node::Joint::f << std::endl
    << "τ :\n" << Node::Joint::torque << std::endl
    << std::endl;

  printState<typename Node::Child1>(os);
  printState<typename Node::Child2>(os);
  printState<typename Node::Child3>(os);
}

template<> inline void printState<NC>(std::ostream &){}


/*
 * Print q, dq and ddq in a stream.
 * Can be used to log a configuration that can later be loaded through the
 * initConf method.
 */
template< typename Tree > void printConf(const vectorN & q,
                                         const vectorN & dq,
                                         const vectorN & ddq,
                                         std::ostream & qlog,
                                         std::ostream & dqlog,
                                         std::ostream & ddqlog)
{
  typedef Tree Node;

  vectorN qi = q.segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);
  vectorN dqi = dq.segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);
  vectorN ddqi = ddq.segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);

  qlog << Node::Joint::name << std::endl << qi << std::endl;
  dqlog << Node::Joint::name << std::endl << dqi << std::endl;
  ddqlog << Node::Joint::name << std::endl << ddqi << std::endl;

  printConf<typename Node::Child1>(q, dq, ddq, qlog, dqlog, ddqlog);
  printConf<typename Node::Child2>(q, dq, ddq, qlog, dqlog, ddqlog);
  printConf<typename Node::Child3>(q, dq, ddq, qlog, dqlog, ddqlog);
}

template<>
inline void printConf<NC>(const vectorN &, const vectorN &, const vectorN &,
                          std::ostream &, std::ostream &, std::ostream &){}

// Print Torques of the robot in a stream.
template< typename Tree >
void printTorques(std::ostream & os)
{
  typedef Tree Node;

  os << Node::Joint::name << std::endl
     << Node::Joint::torque << std::endl
     << std::endl;

  printTorques<typename Node::Child1>(os);
  printTorques<typename Node::Child2>(os);
  printTorques<typename Node::Child3>(os);
}

template<>
inline void printTorques<NC>(std::ostream &){}

// Get Torques of the robot.
template< typename Tree >
void getTorques(vectorN& torques, unsigned& i)
{
  typedef Tree Node;

  unsigned j = 0;
  while (j < Node::Joint::NBDOF)
  {
    torques[i] = Node::Joint::torque[j];
    ++i;
    ++j;
  }

  getTorques<typename Node::Child1>(torques, i);
  getTorques<typename Node::Child2>(torques, i);
  getTorques<typename Node::Child3>(torques, i);
}

template<>
inline void getTorques<NC>(vectorN&, unsigned&){}

} // end of namespace metapod.

#endif
