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
 * Implementation of the Recursive Newton Euler Algorithm,
 * based on Featherstone's Rigid Body Dynamics Algorithms.
 */

#ifndef METAPOD_RNEA_HH
# define METAPOD_RNEA_HH

# include "metapod/tools/common.hh"

namespace metapod
{

  static GiNaC::lst lst_vi, lst_ai, lst_vi_ai;
  static std::ofstream RNEA("symbolic_rnea.hh", std::ofstream::out);

  /** Templated Recursive Newton-Euler Algorithm.
    * Takes the multibody tree type as template parameter,
    * and recursively proceeds on the Nodes.
    */
  template< typename Tree, typename confVector, bool HasParent = false > struct rnea {};

  template< typename Tree, typename confVector > struct rnea< Tree, confVector, true >
  {
    typedef Tree Node;

    static void run(const confVector & q,
                    const confVector & dq,
                    const confVector & ddq) __attribute__ ((hot))
    {
      for(int i=0; i<3; i++)
      {
        lst_vi.append(Node::Body::vi_symbol.w()[i]);
        lst_vi.append(Node::Body::vi_symbol.v()[i]);
        lst_ai.append(Node::Body::ai_symbol.w()[i]);
        lst_ai.append(Node::Body::ai_symbol.v()[i]);
        lst_vi_ai.append(Node::Body::vi_symbol.w()[i]);
        lst_vi_ai.append(Node::Body::vi_symbol.v()[i]);
        lst_vi_ai.append(Node::Body::ai_symbol.w()[i]);
        lst_vi_ai.append(Node::Body::ai_symbol.v()[i]);
      }
      GiNaC::lst lst;
      for(int i=0; i<q.size(); i++)
      {
        lst.append(cos(q(i))); lst.append(sin(q(i)));
        lst.append(dq(i)); lst.append(ddq(i));
      }

      std::ofstream SLOG;
      std::string logdir = "symbolic_log/";
      std::string logname;

      // Extract subvector corresponding to current Node
      Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > qi =
        q.template segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);
      Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > dqi =
        dq.template segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);
      Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > ddqi =
        ddq.template segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);

      /* forward computations follow */
      // Jcalc: update sXp, S, dotS, cj, vj
      Node::Joint::jcalc(qi, dqi);

      // SYMBOLIC
      logname = logdir + Node::Joint::name + "_sXp.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Joint::sXp << std::endl;
      SLOG.close();

      // iX0 = iXλ(i) * λ(i)X0
      // vi = iXλ(i) * vλ(i) + vj
      // ai = iXλ(i) * aλ(i) + Si * ddqi + cj + vi x vj
      Node::Body::iX0 = Node::Joint::sXp*Node::Body::Parent::iX0;
      Node::Body::iX0.collect(lst);

      // SYMBOLIC
      logname = logdir + Node::Body::name + "_iX0.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Body::iX0 << std::endl;
      SLOG.close();

      Node::Body::vi = Node::Joint::sXp*Node::Body::Parent::vi
                     + Node::Joint::vj;
      Node::Body::vi.collect(lst);

      // SYMBOLIC
      logname = logdir + Node::Body::name + "_vi.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Body::vi << std::endl;
      SLOG.close();

      Node::Body::ai = sum(Node::Joint::sXp*Node::Body::Parent::ai,
                           Motion(Node::Joint::S * ddqi),
                           Node::Joint::cj,
                           (Node::Body::vi_symbol^Node::Joint::vj));
      Node::Body::ai.collect(lst_vi);

      // SYMBOLIC
      logname = logdir + Node::Body::name + "_ai.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Body::ai << std::endl;
      SLOG.close();

      // fi = Ii * ai + vi x* (Ii * vi) - iX0* * fix
      vector3d global_CoM = Node::Body::iX0*Node::Body::CoM;

      vector3d gravity_force = vector3d(0, 0, GRAVITY_CST);
      vector3d gravity_torque = global_CoM.cross(gravity_force);

      Force Fext = Force(gravity_torque,gravity_force);
      Force Fext_in_0 = Force(Node::Body::iX0 * ( Node::Body::mass * Fext
                                                - Node::Body::Fext ));

      // SYMBOLIC
      logname = logdir + Node::Body::name + "_Fext.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Fext_in_0 << std::endl;
      SLOG.close();

      Node::Joint::f = sum((Node::Body::I * Node::Body::ai_symbol),
                           (Node::Body::vi_symbol^( Node::Body::I * Node::Body::vi_symbol )),
                           (Node::Body::Fext_in_0_symbol));
//                           (Node::Body::iX0 * ( Node::Body::mass * Fext
//                                              - Node::Body::Fext )));

      // recursion on children
      rnea< typename Node::Child1, confVector, true >::run(q, dq, ddq);
      rnea< typename Node::Child2, confVector, true >::run(q, dq, ddq);
      rnea< typename Node::Child3, confVector, true >::run(q, dq, ddq);

      // backward computations follow
      // τi = SiT * fi
      Node::Joint::torque = Node::Joint::S.transpose()*Node::Joint::f.toVector();
//      Node::Joint::torque.collect(lst);

      // SYMBOLIC
      logname = logdir + Node::Joint::name + "_torque.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Joint::torque << std::endl;
      SLOG.close();

      // fλ(i) = fλ(i) + λ(i)Xi* * fi
      Node::Body::Parent::Joint::f = Node::Body::Parent::Joint::f
                                   + Node::Joint::sXp.applyInv(Node::Joint::f);
//      Node::Body::Parent::Joint::f.collect(GiNaC::lst(cos(q(Node::Joint::positionInConf)), sin(q(Node::Joint::positionInConf))));
//      Node::Body::Parent::Joint::f.collect(lst_vi_ai);

      // SYMBOLIC
      logname = logdir + Node::Joint::name + "_f.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Joint::f << std::endl;
      SLOG.close();

    }
  };

  template< typename Tree, typename confVector > struct rnea< Tree, confVector, false >
  {
    typedef Tree Node;

    static void run(const confVector & q,
                    const confVector & dq,
                    const confVector & ddq)
    {
      for(int i=0; i<3; i++)
      {
        lst_vi.append(Node::Body::vi_symbol.w()[i]);
        lst_vi.append(Node::Body::vi_symbol.v()[i]);
        lst_ai.append(Node::Body::ai_symbol.w()[i]);
        lst_ai.append(Node::Body::ai_symbol.v()[i]);
        lst_vi_ai.append(Node::Body::vi_symbol.w()[i]);
        lst_vi_ai.append(Node::Body::vi_symbol.v()[i]);
        lst_vi_ai.append(Node::Body::ai_symbol.w()[i]);
        lst_vi_ai.append(Node::Body::ai_symbol.v()[i]);
      }
      GiNaC::lst lst;
      for(int i=0; i<q.size(); i++)
      {
        lst.append(cos(q(i))); lst.append(sin(q(i)));
        lst.append(dq(i)); lst.append(ddq(i));
      }

      std::ofstream SLOG;
      std::string logdir = "symbolic_log/";
      std::string logname;

      // Extract subvector corresponding to current Node
      Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > qi =
        q.template segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);
      Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > dqi =
        dq.template segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);
      Eigen::Matrix< FloatType, Node::Joint::NBDOF, 1 > ddqi =
        ddq.template segment<Node::Joint::NBDOF>(Node::Joint::positionInConf);

      /* forward computations follow */
      // Jcalc: update sXp, S, dotS, cj, vj
      Node::Joint::jcalc(qi, dqi);

      // SYMBOLIC
      logname = logdir + Node::Joint::name + "_sXp.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Joint::sXp << std::endl;
      SLOG.close();

      // iX0 = iXλ(i)
      // vi = vj
      // ai = Si * ddqi + cj + vi x vj
      Node::Body::iX0 = Node::Joint::sXp;
      Node::Body::iX0.collect(lst);

      Node::Body::vi = Node::Joint::vj;

      // SYMBOLIC
      logname = logdir + Node::Body::name + "_iX0.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Body::iX0 << std::endl;
      SLOG.close();

      Node::Body::ai = sum(Motion(Node::Joint::S * ddqi),
                           Node::Joint::cj,
                           (Node::Body::vi_symbol^Node::Joint::vj));
      Node::Body::ai.collect(lst_vi);

      // SYMBOLIC
      logname = logdir + Node::Body::name + "_ai.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Body::ai << std::endl;
      SLOG.close();

      // fi = Ii * ai + vi x* (Ii * vi) - iX0* * fix
      vector3d global_CoM = Node::Body::iX0*Node::Body::CoM;

      vector3d gravity_force = vector3d(0, 0, GRAVITY_CST);
      vector3d gravity_torque = global_CoM.cross(gravity_force);

      Force Fext = Force(gravity_torque,gravity_force);
      Force Fext_in_0 = Force(Node::Body::iX0 * ( Node::Body::mass * Fext
                                                - Node::Body::Fext ));

      // SYMBOLIC
      logname = logdir + Node::Body::name + "_Fext.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Fext_in_0 << std::endl;
      SLOG.close();

      Node::Joint::f = sum((Node::Body::I * Node::Body::ai_symbol),
                           (Node::Body::vi_symbol^( Node::Body::I * Node::Body::vi_symbol )),
                           (Node::Body::Fext_in_0_symbol));
//                           (Node::Body::iX0 * ( Node::Body::mass * Fext
//                                              - Node::Body::Fext )));

      // recursion on children
      rnea< typename Node::Child1, confVector, true >::run(q, dq, ddq);
      rnea< typename Node::Child2, confVector, true >::run(q, dq, ddq);
      rnea< typename Node::Child3, confVector, true >::run(q, dq, ddq);

      // backward computations follow
      // τi = SiT * fi
      Node::Joint::torque = Node::Joint::S.transpose()
                          * Node::Joint::f.toVector();

      // SYMBOLIC
      logname = logdir + Node::Joint::name + "_torque.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      for(int i=0; i<6; i++)
        SLOG << Node::Joint::torque[i] << std::endl;
      SLOG.close();

      RNEA.close();
    }
  };

  /**
    \brief  Specialization, to stop recursion on leaves of the Tree
  */
  template< typename confVector > struct rnea< NC, confVector, true >
  {
    static void run(const confVector &,
                    const confVector &,
                    const confVector &) {}
  };

} // end of namespace metapod.

#endif
