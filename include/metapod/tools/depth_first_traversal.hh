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
// Implementation of the depth-first traversal algorithm
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
// Then visit each node with it:
//
//     MyArg myarg;
//     depth_first_traversal<MyVisitor, simple_humanoid::Robot>::run(myarg);
//
// This will walk down the tree, and call MyVisitor<Node>::discover(myarg)
// and MyVisitor<Node>::finish(myarg) at each node.
//
// There are also variants with no argument (for both run() and visit()) and
// two and three arguments.
//
// Note: we might add a third visitor hook called "examine" which would be
//       called on each child node before recursing deeper in the tree.

#ifndef METAPOD_DEPTH_FIRST_TRAVERSAL_HH
# define METAPOD_DEPTH_FIRST_TRAVERSAL_HH

# include "metapod/tools/common.hh"

namespace metapod
{
  template< template <typename AnyNode> class Visitor,
            typename Node >
  struct depth_first_traversal_internal
  {
    static void run()
    {
      Visitor<Node>::discover();
      depth_first_traversal_internal<Visitor, typename Node::Child0>::run();
      depth_first_traversal_internal<Visitor, typename Node::Child1>::run();
      depth_first_traversal_internal<Visitor, typename Node::Child2>::run();
      depth_first_traversal_internal<Visitor, typename Node::Child3>::run();
      depth_first_traversal_internal<Visitor, typename Node::Child4>::run();
      Visitor<Node>::finish();
    }

    template<typename Arg>
    static void run(Arg & arg)
    {
      Visitor<Node>::discover(arg);
      depth_first_traversal_internal<Visitor, typename Node::Child0>::run(arg);
      depth_first_traversal_internal<Visitor, typename Node::Child1>::run(arg);
      depth_first_traversal_internal<Visitor, typename Node::Child2>::run(arg);
      depth_first_traversal_internal<Visitor, typename Node::Child3>::run(arg);
      depth_first_traversal_internal<Visitor, typename Node::Child4>::run(arg);
      Visitor<Node>::finish(arg);
    }

    template<typename Arg0, typename Arg1>
    static void run(Arg0 & arg0, Arg1 & arg1)
    {
      Visitor<Node>::discover(arg0, arg1);
      depth_first_traversal_internal<Visitor, typename Node::Child0>::run(arg0, arg1);
      depth_first_traversal_internal<Visitor, typename Node::Child1>::run(arg0, arg1);
      depth_first_traversal_internal<Visitor, typename Node::Child2>::run(arg0, arg1);
      depth_first_traversal_internal<Visitor, typename Node::Child3>::run(arg0, arg1);
      depth_first_traversal_internal<Visitor, typename Node::Child4>::run(arg0, arg1);
      Visitor<Node>::finish(arg0, arg1);
    }

    template<typename Arg0, typename Arg1, typename Arg2>
    static void run(Arg0 & arg0, Arg1 & arg1, Arg2 & arg2)
    {
      Visitor<Node>::discover(arg0, arg1, arg2);
      depth_first_traversal_internal<Visitor, typename Node::Child0>::run(arg0, arg1, arg2);
      depth_first_traversal_internal<Visitor, typename Node::Child1>::run(arg0, arg1, arg2);
      depth_first_traversal_internal<Visitor, typename Node::Child2>::run(arg0, arg1, arg2);
      depth_first_traversal_internal<Visitor, typename Node::Child3>::run(arg0, arg1, arg2);
      depth_first_traversal_internal<Visitor, typename Node::Child4>::run(arg0, arg1, arg2);
      Visitor<Node>::finish(arg0, arg1, arg1);
    }
  };

  template< template <typename AnyNode> class Visitor>
  struct depth_first_traversal_internal<Visitor, NC >
  {
    static void run() {}

    template<typename Arg>
    static void run(Arg & ) {}

    template<typename Arg0, typename Arg1>
    static void run(Arg0 & , Arg1 & ) {}

    template<typename Arg0, typename Arg1, typename Arg2>
    static void run(Arg0 & , Arg1 & , Arg2 & ) {}
  };

  template< template <typename Node> class Visitor,
            typename Robot >
  struct depth_first_traversal
  {
    static void run()
    {
      depth_first_traversal_internal<Visitor, typename Robot::Tree>::run();
    }

    template<typename Arg>
    static void run(Arg & arg)
    {
      depth_first_traversal_internal<Visitor, typename Robot::Tree>::run(arg);
    }

    template<typename Arg0, typename Arg1>
    static void run(Arg0 & arg0, Arg1 & arg1)
    {
      depth_first_traversal_internal<Visitor, typename Robot::Tree>::run(arg0, arg1);
    }

    template<typename Arg0, typename Arg1, typename Arg2>
    static void run(Arg0 & arg0, Arg1 & arg1, Arg2 & arg2)
    {
      depth_first_traversal_internal<Visitor, typename Robot::Tree>::run(arg0, arg1, arg2);
    }
  };
}
#endif
