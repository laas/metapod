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

// This test traverses the tree depth-first and prints events

// Common test tools
#include "common.hh"
#include <metapod/tools/depth_first_traversal.hh>

using namespace metapod;

// Print events while traversing the tree with indentation showing the level
// of recursion
template < typename Robot, int node_id > struct PrintDFTraversalVisitor
{
  static void discover(std::ostream & os, int & depth)
  {
    const std::string prefix(depth, '\t');
    typedef typename Nodes<Robot, node_id>::type Node;
    os << prefix << "discover: "
        << Node::joint_name << " -- " << Node::body_name << "\n";
    ++depth;
  }
  static void finish(std::ostream & os, int & depth)
  {
    --depth;
    const std::string prefix(depth, '\t');
    typedef typename Nodes<Robot, node_id>::type Node;
    os << prefix << "finish: "
        << Node::joint_name << " -- " << Node::body_name << "\n";
  }
};

DEFAULT_FLOAT_TYPE;

BOOST_AUTO_TEST_CASE (test_depth_first_traversal)
{
  typedef CURRENT_MODEL_ROBOT<LocalFloatType> CURRENT_MODEL_ROBOT_LFT;
  const char result_file[] = "depth_first_traversal.log";
  std::ofstream log(result_file, std::ofstream::out);
  int depth = 0;
  depth_first_traversal<PrintDFTraversalVisitor, CURRENT_MODEL_ROBOT_LFT>::run(log, depth);
  log.close();
  // Compare results with reference file
  compareTexts(result_file, TEST_DIRECTORY "/depth_first_traversal.ref");
}
