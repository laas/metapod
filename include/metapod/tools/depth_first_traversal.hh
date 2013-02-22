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

//
// Implementation of the depth-first traversal algorithm
//
// To use it, create a visitor
//
//     template<typename Robot, int node_id> class MyVisitor
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
// Then visit each node with it:
//
//     MyArg myarg;
//     depth_first_traversal<MyVisitor, simple_arm>::run(myarg);
//
// This will walk down the tree, and call its discover(myarg) and
// finish(myarg) methods at each node.
//
// There are variants with 0, 1, 2, 3 and 4 arguments for both run() and visit()
//
// Note: we might add a third visitor hook called "examine" which would be
//       called on each child node before recursing deeper in the tree.

#ifndef METAPOD_DEPTH_FIRST_TRAVERSAL_HH
# define METAPOD_DEPTH_FIRST_TRAVERSAL_HH

# include "metapod/tools/common.hh"

namespace metapod {
namespace internal {

template< template <typename AnyRobot, int any_node_id> class Visitor,
          typename Robot,
          int node_id >
struct depth_first_traversal_internal
{
  typedef typename Nodes<Robot, node_id>::type Node;

  static void run()
  {
    Visitor<Robot, node_id>::discover();
    depth_first_traversal_internal<Visitor, Robot, Node::child0_id>::run();
    depth_first_traversal_internal<Visitor, Robot, Node::child1_id>::run();
    depth_first_traversal_internal<Visitor, Robot, Node::child2_id>::run();
    depth_first_traversal_internal<Visitor, Robot, Node::child3_id>::run();
    depth_first_traversal_internal<Visitor, Robot, Node::child4_id>::run();
    Visitor<Robot, node_id>::finish();
  }

  template<typename Arg>
  static void run(Arg & arg)
  {
    Visitor<Robot, node_id>::discover(arg);
    depth_first_traversal_internal<Visitor, Robot, Node::child0_id>::run(arg);
    depth_first_traversal_internal<Visitor, Robot, Node::child1_id>::run(arg);
    depth_first_traversal_internal<Visitor, Robot, Node::child2_id>::run(arg);
    depth_first_traversal_internal<Visitor, Robot, Node::child3_id>::run(arg);
    depth_first_traversal_internal<Visitor, Robot, Node::child4_id>::run(arg);
    Visitor<Robot, node_id>::finish(arg);
  }

  template<typename Arg0, typename Arg1>
  static void run(Arg0 & arg0, Arg1 & arg1)
  {
    Visitor<Robot, node_id>::discover(arg0, arg1);
    depth_first_traversal_internal<Visitor, Robot, Node::child0_id>::run(arg0, arg1);
    depth_first_traversal_internal<Visitor, Robot, Node::child1_id>::run(arg0, arg1);
    depth_first_traversal_internal<Visitor, Robot, Node::child2_id>::run(arg0, arg1);
    depth_first_traversal_internal<Visitor, Robot, Node::child3_id>::run(arg0, arg1);
    depth_first_traversal_internal<Visitor, Robot, Node::child4_id>::run(arg0, arg1);
    Visitor<Robot, node_id>::finish(arg0, arg1);
  }

  template<typename Arg0, typename Arg1, typename Arg2>
  static void run(Arg0 & arg0, Arg1 & arg1, Arg2 & arg2)
  {
    Visitor<Robot, node_id>::discover(arg0, arg1, arg2);
    depth_first_traversal_internal<Visitor, Robot, Node::child0_id>::run(arg0, arg1, arg2);
    depth_first_traversal_internal<Visitor, Robot, Node::child1_id>::run(arg0, arg1, arg2);
    depth_first_traversal_internal<Visitor, Robot, Node::child2_id>::run(arg0, arg1, arg2);
    depth_first_traversal_internal<Visitor, Robot, Node::child3_id>::run(arg0, arg1, arg2);
    depth_first_traversal_internal<Visitor, Robot, Node::child4_id>::run(arg0, arg1, arg2);
    Visitor<Robot, node_id>::finish(arg0, arg1, arg2);
  }

  template<typename Arg0, typename Arg1, typename Arg2, typename Arg3>
  static void run(Arg0 & arg0, Arg1 & arg1, Arg2 & arg2, Arg3 & arg3)
  {
    Visitor<Robot, node_id>::discover(arg0, arg1, arg2, arg3);
    depth_first_traversal_internal<Visitor, Robot, Node::child0_id>::run(arg0, arg1, arg2, arg3);
    depth_first_traversal_internal<Visitor, Robot, Node::child1_id>::run(arg0, arg1, arg2, arg3);
    depth_first_traversal_internal<Visitor, Robot, Node::child2_id>::run(arg0, arg1, arg2, arg3);
    depth_first_traversal_internal<Visitor, Robot, Node::child3_id>::run(arg0, arg1, arg2, arg3);
    depth_first_traversal_internal<Visitor, Robot, Node::child4_id>::run(arg0, arg1, arg2, arg3);
    Visitor<Robot, node_id>::finish(arg0, arg1, arg2, arg3);
  }
};

template< template <typename AnyRobot, int any_node_id> class Visitor, typename Robot>
struct depth_first_traversal_internal<Visitor, Robot, NO_CHILD>
{
  static void run() {}

  template<typename Arg>
  static void run(Arg & ) {}

  template<typename Arg0, typename Arg1>
  static void run(Arg0 & , Arg1 & ) {}

  template<typename Arg0, typename Arg1, typename Arg2>
  static void run(Arg0 & , Arg1 & , Arg2 & ) {}

  template<typename Arg0, typename Arg1, typename Arg2, typename Arg3>
  static void run(Arg0 & , Arg1 & , Arg2 &, Arg3 &) {}
};

} // end of namespace metapod::internal

template< template <typename AnyRobot, int any_node_id> class Visitor,
          typename Robot >
struct depth_first_traversal
{
  static void run()
  {
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child0_id>::run();
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child1_id>::run();
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child2_id>::run();
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child3_id>::run();
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child4_id>::run();
  }

  template<typename Arg>
  static void run(Arg & arg)
  {
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child0_id>::run(arg);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child1_id>::run(arg);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child2_id>::run(arg);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child3_id>::run(arg);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child4_id>::run(arg);
  }

  template<typename Arg0, typename Arg1>
  static void run(Arg0 & arg0, Arg1 & arg1)
  {
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child0_id>::run(arg0, arg1);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child1_id>::run(arg0, arg1);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child2_id>::run(arg0, arg1);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child3_id>::run(arg0, arg1);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child4_id>::run(arg0, arg1);
  }

  template<typename Arg0, typename Arg1, typename Arg2>
  static void run(Arg0 & arg0, Arg1 & arg1, Arg2 & arg2)
  {
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child0_id>::run(arg0, arg1, arg2);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child1_id>::run(arg0, arg1, arg2);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child2_id>::run(arg0, arg1, arg2);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child3_id>::run(arg0, arg1, arg2);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child4_id>::run(arg0, arg1, arg2);
  }

  template<typename Arg0, typename Arg1, typename Arg2, typename Arg3>
  static void run(Arg0 & arg0, Arg1 & arg1, Arg2 & arg2, Arg3 & arg3)
  {
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child0_id>::run(arg0, arg1, arg2, arg3);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child1_id>::run(arg0, arg1, arg2, arg3);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child2_id>::run(arg0, arg1, arg2, arg3);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child3_id>::run(arg0, arg1, arg2, arg3);
    internal::depth_first_traversal_internal<Visitor, Robot, Robot::child4_id>::run(arg0, arg1, arg2, arg3);
  }
};
} // end of namespace metapod
#endif
