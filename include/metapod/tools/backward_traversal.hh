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

//
// Implementation of the "backward" traversal algorithm: from a Node,
// traverse the tree until reaching the root.
//
// To use it, create a visitor
//
//     template<Node> class MyVisitor
//     {
//       static void discover(MyArg & arg)
//       {
//         //... do something
//       };
//       static void finish(MyArg & arg)
//       {
//         //... do something
//       };
//     }
//
// where Node is the Node being visited.
//
// Then visit each node with it:
//
//     MyArg myarg;
//     backward_traversal<MyVisitor,
//                        simple_humanoid::Robot,
//                        StartNode,
//                        EndNode>::run(myarg);
//
// This will walk the tree jumping from a StartNode to its ancestors, and call
// MyVisitor<Node>::discover(myarg) and
// MyVisitor<Node>::finish(myarg) at each visited node, until it
// reaches EndNode. If you do not provide the EndNote argument, it defaults
// to NP (no parent), the root of the kinematic tree.
//
// There are also variants with no argument (for both run() and visit()) and
// two arguments.

#ifndef METAPOD_BACKWARD_TRAVERSAL_HH
# define METAPOD_BACKWARD_TRAVERSAL_HH

# include "metapod/tools/common.hh"
# include "metapod/tools/is_ancestor.hh"
# include <boost/static_assert.hpp>

namespace metapod
{
  template< template <typename AnyNode> class Visitor,
            typename Node, typename EndNode>
  struct backward_traversal_internal
  {
    static void run()
    {
      Visitor<Node>::discover();
      backward_traversal_internal<Visitor, typename Node::Parent, EndNode>::run();
      Visitor<Node>::finish();
    }

    template<typename Arg0>
    static void run(Arg0 & arg0)
    {
      Visitor<Node>::discover(arg0);
      backward_traversal_internal<Visitor, typename Node::Parent, EndNode>::run(arg0);
      Visitor<Node>::finish(arg0);
    }

    template<typename Arg0, typename Arg1>
    static void run(Arg0 & arg0, Arg1 & arg1)
    {
      Visitor<Node>::discover(arg0, arg1);
      backward_traversal_internal<Visitor, typename Node::Parent, EndNode>::run(arg0, arg1);
      Visitor<Node>::finish(arg0, arg1);
    }
  };

  // end recursion when we reach EndNode
  template< template <typename AnyNode> class Visitor,
            typename EndNode>
  struct backward_traversal_internal<Visitor, EndNode, EndNode >
  {
    static void run() {}

    template<typename Arg0>
    static void run(Arg0 & arg0) {}

    template<typename Arg0, typename Arg1>
    static void run(Arg0 &, Arg1 & ) {}
  };

  // the Robot template argument is currently unused but is kept for
  // consistency with other algorithms.
  // Generic implementation: requires that StartNode != EndNote
  template< template <typename AnyNode> class Visitor,
            typename Robot,
            typename StartNode,
            typename EndNode=NP >
  struct backward_traversal
  {
    BOOST_STATIC_ASSERT(( is_ancestor<EndNode, StartNode>::value ));
    static void run()
    {
      backward_traversal_internal<Visitor, StartNode, EndNode>::run();
    }

    template<typename Arg0>
    static void run(Arg0 & arg0)
    {
      backward_traversal_internal<Visitor, StartNode, EndNode>::run(arg0);
    }

    template<typename Arg0, typename Arg1>
    static void run(Arg0 & arg0, Arg1 & arg1)
    {
      backward_traversal_internal<Visitor, StartNode, EndNode>::run(arg0, arg1);
    }
  };

  // Specialization: deal with the case where the user called the algorithm
  // with EndNode==StartNode
  template< template <typename AnyNode> class Visitor,
            typename Robot,
            typename EndNode >
  struct backward_traversal<Visitor, Robot, EndNode, EndNode >
  {
    static void run() {}

    template<typename Arg0>
    static void run(Arg0 & ) {}

    template<typename Arg0, typename Arg1>
    static void run(Arg0 & , Arg1 & ) {}
  };
}
#endif
