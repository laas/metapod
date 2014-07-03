// Copyright 2012, 2013
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

// Common test tools
#include "common.hh"
#include <metapod/tools/backward_traversal.hh>

using namespace metapod;

DEFAULT_FLOAT_TYPE;

// start at the hand and finish with the arm
#ifdef CURRENT_MODEL_IS_SIMPLE_HUMANOID
const int start_node = CURRENT_MODEL_ROBOT<LocalFloatType>::LARM_LINK7;
const int end_node = CURRENT_MODEL_ROBOT<LocalFloatType>::LARM_LINK3;
#else
const int start_node = CURRENT_MODEL_ROBOT<LocalFloatType>::HAND;
const int end_node = CURRENT_MODEL_ROBOT<LocalFloatType>::ARM;
#endif

// Print events while traversing the tree with indentation showing the level
// of recursion
template < typename Robot, int node_id > struct PrintBwdTraversalVisitor
{
  typedef typename Nodes<Robot, node_id>::type Node;
  static void discover(std::ostream & os, int & depth)
  {
    const std::string prefix(depth, '\t');
    os << prefix << "discover: curr:"
        << Node::joint_name << " -- " << Node::body_name << "\n";
    ++depth;
  }
  static void finish(std::ostream & os, int & depth)
  {
    --depth;
    const std::string prefix(depth, '\t');
    os << prefix << "finish: curr:"
        << Node::joint_name << " -- " << Node::body_name << "\n";
  }
};

BOOST_AUTO_TEST_CASE (test_backward_traversal)
{
  const char result_file[] = "backward_traversal.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  backward_traversal<PrintBwdTraversalVisitor, CURRENT_MODEL_ROBOT<LocalFloatType>,
      start_node>::run(log, depth);
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/backward_traversal.ref");
}

BOOST_AUTO_TEST_CASE (test_backward_traversal_end)
{
  const char result_file[] = "backward_traversal_end.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  backward_traversal<PrintBwdTraversalVisitor, CURRENT_MODEL_ROBOT<LocalFloatType>,
      start_node, end_node>::run(log, depth);
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/backward_traversal_end.ref");
}
