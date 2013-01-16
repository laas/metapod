// Copyright 2012,
//
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

// This test traverses the tree backward and prints events

//Common test tools
#include "common.hh"
#include <metapod/tools/backward_traversal_prev.hh>

using namespace metapod;
using namespace CURRENT_MODEL_NAMESPACE;

// start at the hand and finish with the arm
#ifdef CURRENT_MODEL_IS_SIMPLE_HUMANOID
# define START_NODE LARM_LINK7
# define END_NODE LARM_LINK3
#else
# define START_NODE HAND
# define END_NODE ARM
#endif

// Print events while traversing the tree with indentation showing the level
// of recursion
template < typename Node, typename PrevNode > struct PrintBwdTraversalVisitor
{
  static void discover(std::ostream & os, int & depth)
  {
    const std::string prefix(depth, '\t');
    os << prefix << "discover: curr:"
        << Node::Joint::name << " -- " << Node::name << "\n";
    os << prefix << "          prev: "
        << PrevNode::Joint::name << " -- " << PrevNode::name << "\n";
    ++depth;
  }
  static void finish(std::ostream & os, int & depth)
  {
    --depth;
    const std::string prefix(depth, '\t');
    os << prefix << "finish: curr:"
        << Node::Joint::name << " -- " << Node::name << "\n";
    os << prefix << "        prev: "
        << PrevNode::Joint::name << " -- " << PrevNode::name << "\n";
  }
};

BOOST_AUTO_TEST_CASE (test_backward_traversal)
{
  const char result_file[] = "backward_traversal_prev.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  backward_traversal_prev<PrintBwdTraversalVisitor, Robot,
      CURRENT_MODEL_NAMESPACE::START_NODE>::run(log, depth);
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/backward_traversal_prev.ref");
}

BOOST_AUTO_TEST_CASE (test_backward_traversal_end)
{
  const char result_file[] = "backward_traversal_prev_end.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  backward_traversal_prev<PrintBwdTraversalVisitor, Robot,
      CURRENT_MODEL_NAMESPACE::START_NODE,
      CURRENT_MODEL_NAMESPACE::END_NODE
      >::run(log, depth);
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/backward_traversal_prev_end.ref");
}
