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

/*
      std::ofstream SLOG;
      std::string logdir = "symbolic_log/";
      std::string logname;
*/

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

/*
      // SYMBOLIC
      logname = logdir + Node::Joint::name + "_sXp.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Joint::sXp << std::endl;
      SLOG.close();
*/

      // iX0 = iXλ(i) * λ(i)X0
      // vi = iXλ(i) * vλ(i) + vj
      // ai = iXλ(i) * aλ(i) + Si * ddqi + cj + vi x vj
      Node::Body::iX0 = Node::Joint::sXp*Node::Body::Parent::iX0_symbol;
      Node::Body::iX0.collect(lst);

/*
      // SYMBOLIC
      logname = logdir + Node::Body::name + "_iX0.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Body::iX0 << std::endl;
      SLOG.close();
*/

      Node::Body::vi = Node::Joint::sXp*Node::Body::Parent::vi_symbol
                     + Node::Joint::vj;
      Node::Body::vi.collect(lst);

/*
      // SYMBOLIC
      logname = logdir + Node::Body::name + "_vi.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Body::vi << std::endl;
      SLOG.close();
*/

      Node::Body::ai = sum(Node::Joint::sXp*Node::Body::Parent::ai_symbol,
                           Motion(Node::Joint::S * ddqi),
                           Node::Joint::cj,
                           (Node::Body::vi^Node::Joint::vj));
//      Node::Body::ai.collect(lst_vi);
      Node::Body::ai.collect(lst);

/*
      // SYMBOLIC
      logname = logdir + Node::Body::name + "_ai.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Body::ai << std::endl;
      SLOG.close();
*/

      // fi = Ii * ai + vi x* (Ii * vi) - iX0* * fix
      vector3d global_CoM = Node::Body::iX0*Node::Body::CoM;

      vector3d gravity_force = vector3d(0, 0, GRAVITY_CST);
      vector3d gravity_torque = global_CoM.cross(gravity_force);

      Force Fext = Force(gravity_torque,gravity_force);
      Node::Body::Fext_in_0 = Force(Node::Body::iX0 * ( Node::Body::mass * Fext
                                                      - Node::Body::Fext ));

/*
      // SYMBOLIC
      logname = logdir + Node::Body::name + "_Fext.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Body::Fext_in_0 << std::endl;
      SLOG.close();
*/

      Node::Joint::f = sum((Node::Body::I * Node::Body::ai),
                           (Node::Body::vi^( Node::Body::I * Node::Body::vi )),
                           (Node::Body::Fext_in_0));
//                           (Node::Body::iX0 * ( Node::Body::mass * Fext
//                                              - Node::Body::Fext )));

//*
      // WRITE SYMBOLIC EXPRESSIONS IN .CC FILE
      // sXp
      for(int i=0; i<3; i++)
      {
        for(int j=0; j<3; j++)
          RNEA << Node::Joint::name << "::sXp.m_E(" << i << "," << j << ") = " << Node::Joint::sXp.E()(i,j) << ";" << std::endl;
        RNEA << Node::Joint::name << "::sXp.m_r[" << i << "] = " << Node::Joint::sXp.r()[i] << ";" << std::endl;
      }
/*
          RNEA << "E(" << i << "," << j << ") = " << Node::Joint::sXp.E()(i,j) << ";" << std::endl;
        RNEA << "r[" << i << "] = " << Node::Joint::sXp.r()[i] << ";" << std::endl;
      }
      RNEA << Node::Joint::name << "::sXp = Transform(E,r);" << std::endl;
*/

      // iX0
      for(int i=0; i<3; i++)
      {
        for(int j=0; j<3; j++)
          RNEA << Node::Body::name << "::iX0.m_E(" << i << "," << j << ") = " << Node::Body::iX0.E()(i,j) << ";" << std::endl;
        RNEA << Node::Body::name << "::iX0.m_r[" << i << "] = " << Node::Body::iX0.r()[i] << ";" << std::endl;
      }
//      RNEA << Node::Body::name << "::iX0 = Transform(E,r);" << std::endl;

      // vi
      for(int i=0; i<3; i++)
      {
        RNEA
          << "w[" << i << "] = " << Node::Body::vi.w()[i] << ";" << std::endl
          << "v[" << i << "] = " << Node::Body::vi.v()[i] << ";" << std::endl
          << std::endl;
      }
      RNEA << Node::Body::name << "::vi = Motion(w,v);" << std::endl;
        
      // ai
      for(int i=0; i<3; i++)
      {
        RNEA
          << "w[" << i << "] = " << Node::Body::ai.w()[i] << ";" << std::endl
          << "v[" << i << "] = " << Node::Body::ai.v()[i] << ";" << std::endl
          << std::endl;
      }
      RNEA << Node::Body::name << "::ai = Motion(w,v);" << std::endl;

      // Fext_in_0
      for(int i=0; i<3; i++)
      {
        RNEA
          << "n[" << i << "] = " << Node::Body::Fext_in_0.n()[i] << ";" << std::endl
          << "f[" << i << "] = " << Node::Body::Fext_in_0.f()[i] << ";" << std::endl
          << std::endl;
      }
      RNEA << Node::Body::name << "::Fext_in_0 = Force(n,f);" << std::endl;
//*/


      // recursion on children
      rnea< typename Node::Child1, confVector, true >::run(q, dq, ddq);
      rnea< typename Node::Child2, confVector, true >::run(q, dq, ddq);
      rnea< typename Node::Child3, confVector, true >::run(q, dq, ddq);

      // backward computations follow
      // τi = SiT * fi
//      Node::Joint::torque = Node::Joint::S.transpose()*Node::Joint::f.toVector();
//      Node::Joint::torque.collect(lst);

/*
      // SYMBOLIC
      logname = logdir + Node::Joint::name + "_torque.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Joint::torque << std::endl;
      SLOG.close();
*/

      // fλ(i) = fλ(i) + λ(i)Xi* * fi
      Node::Body::Parent::Joint::f = Node::Body::Parent::Joint::f
                                   + Node::Joint::sXp_symbol.applyInv(Node::Joint::f_symbol);
//      Node::Body::Parent::Joint::f.collect(lst_vi_ai);

      // f
      for(int i=0; i<3; i++)
      {
        RNEA
          << "n[" << i << "] = " << Node::Joint::f.n()[i] << ";" << std::endl
          << "f[" << i << "] = " << Node::Joint::f.f()[i] << ";" << std::endl
          << std::endl;
      }
      RNEA << Node::Joint::name << "::f = Force(n,f);" << std::endl;

      // torque
//        RNEA << Node::Joint::name << "::torque = " << Node::Joint::torque << ";" << std::endl;

/*
      // SYMBOLIC
      logname = logdir + Node::Joint::name + "_f.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Joint::f << std::endl;
      SLOG.close();
*/

    }
  };

  template< typename Tree, typename confVector > struct rnea< Tree, confVector, false >
  {
    typedef Tree Node;

    static void run(const confVector & q,
                    const confVector & dq,
                    const confVector & ddq)
    {
      RNEA << "#include \"common.hh\""                                 << std::endl
           << "using namespace simplehumanoid;"                    << std::endl
           << "typedef Eigen::Matrix< FloatType, Robot::nbDof, 1 > confVector;" << std::endl
           << std::endl
           << "BOOST_AUTO_TEST_CASE (test_symbolic_rnea)"          << std::endl
           << "{"                                                  << std::endl
           << "  matrix3d E;"                                      << std::endl
           << "  vector3d r;"                                      << std::endl
           << "  vector3d v, w;"                                   << std::endl
           << "  vector3d n, f;"                                   << std::endl
           << "  confVector q, dq, ddq;"                           << std::endl
           << "  confVector cq, sq;"                               << std::endl
           << std::endl
           << "  long TICKS_PER_SECOND = 1e6;" << std::endl
           << "  struct timeval tv_start, tv_stop;" << std::endl
           << "  int N1 = 10000;" << std::endl
           << "  int N2 = 1;" << std::endl
           << std::endl
           << "  std::ofstream perf_log(\"rnea_perf.log\", std::ofstream::out);" << std::endl
           << std::endl
           << "  long time_usec = 0;" << std::endl
           << "  long inner_loop_time;" << std::endl
           << "  // Outer loop : generate random configuration" << std::endl
           << "  for(int i=0; i<N1; i++)" << std::endl
           << "  {" << std::endl
           << "    q = confVector::Random();" << std::endl
           << "    dq = confVector::Random();" << std::endl
           << "    ddq = confVector::Random();" << std::endl
           << "    for(int i=0; i<Robot::nbDof; i++)" << std::endl
           << "      cq[i] = cos(q[i]);"              << std::endl
           << "    for(int i=0; i<Robot::nbDof; i++)" << std::endl
           << "      sq[i] = sin(q[i]);"              << std::endl
           << "    ::gettimeofday(&tv_start, NULL);" << std::endl
           << "    // Inner loop : The timer precision is 1µs, which is not high enough to" << std::endl
           << "    // give proper result on a single iteration " << std::endl
           << "    for(int k=0; k<N2; k++)" << std::endl
           << "    {"                       << std::endl
           << std::endl;
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

/*
      std::ofstream SLOG;
      std::string logdir = "symbolic_log/";
      std::string logname;
*/

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

/*
      // SYMBOLIC
      logname = logdir + Node::Joint::name + "_sXp.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Joint::sXp << std::endl;
      SLOG.close();
*/

      // iX0 = iXλ(i)
      // vi = vj
      // ai = Si * ddqi + cj + vi x vj
      Node::Body::iX0 = Node::Joint::sXp;
      Node::Body::iX0.collect(lst);

      Node::Body::vi = Node::Joint::vj;

/*
      // SYMBOLIC
      logname = logdir + Node::Body::name + "_iX0.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Body::iX0 << std::endl;
      SLOG.close();
*/

      Node::Body::ai = sum(Motion(Node::Joint::S * ddqi),
                           Node::Joint::cj,
                           (Node::Body::vi^Node::Joint::vj));
      Node::Body::ai.collect(lst_vi);

/*
      // SYMBOLIC
      logname = logdir + Node::Body::name + "_ai.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Body::ai << std::endl;
      SLOG.close();
*/

      // fi = Ii * ai + vi x* (Ii * vi) - iX0* * fix
      vector3d global_CoM = Node::Body::iX0*Node::Body::CoM;

      vector3d gravity_force = vector3d(0, 0, GRAVITY_CST);
      vector3d gravity_torque = global_CoM.cross(gravity_force);

      Force Fext = Force(gravity_torque,gravity_force);
      Node::Body::Fext_in_0 = Force(Node::Body::iX0 * ( Node::Body::mass * Fext
                                                      - Node::Body::Fext ));

/*
      // SYMBOLIC
      logname = logdir + Node::Body::name + "_Fext.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      SLOG << Node::Body::Fext_in_0 << std::endl;
      SLOG.close();
*/

      Node::Joint::f = sum((Node::Body::I * Node::Body::ai_symbol),
                           (Node::Body::vi_symbol^( Node::Body::I * Node::Body::vi_symbol )),
                           (Node::Body::Fext_in_0));
//                           (Node::Body::iX0 * ( Node::Body::mass * Fext
//                                              - Node::Body::Fext )));

      // WRITE SYMBOLIC EXPRESSIONS IN .CC FILE
      // sXp
      for(int i=0; i<3; i++)
      {
        for(int j=0; j<3; j++)
          RNEA << "E(" << i << "," << j << ") = " << Node::Joint::sXp.E()(i,j) << ";" << std::endl;
        RNEA << "r[" << i << "] = " << Node::Joint::sXp.r()[i] << ";" << std::endl;
      }
      RNEA << Node::Joint::name << "::sXp = Transform(E,r);" << std::endl;

      // iX0
      for(int i=0; i<3; i++)
      {
        for(int j=0; j<3; j++)
          RNEA << "E(" << i << "," << j << ") = " << Node::Body::iX0.E()(i,j) << ";" << std::endl;
        RNEA << "r[" << i << "] = " << Node::Body::iX0.r()[i] << ";" << std::endl;
      }
      RNEA << Node::Body::name << "::iX0 = Transform(E,r);" << std::endl;

      // vi
      for(int i=0; i<3; i++)
      {
        RNEA
          << "w[" << i << "] = " << Node::Body::vi.w()[i] << ";" << std::endl
          << "v[" << i << "] = " << Node::Body::vi.v()[i] << ";" << std::endl
          << std::endl;
      }
      RNEA << Node::Body::name << "::vi = Motion(w,v);" << std::endl;
        
      // ai
      for(int i=0; i<3; i++)
      {
        RNEA
          << "w[" << i << "] = " << Node::Body::ai.w()[i] << ";" << std::endl
          << "v[" << i << "] = " << Node::Body::ai.v()[i] << ";" << std::endl
          << std::endl;
      }
      RNEA << Node::Body::name << "::ai = Motion(w,v);" << std::endl;

      // Fext_in_0
      for(int i=0; i<3; i++)
      {
        RNEA
          << "n[" << i << "] = " << Node::Body::Fext_in_0.n()[i] << ";" << std::endl
          << "f[" << i << "] = " << Node::Body::Fext_in_0.f()[i] << ";" << std::endl
          << std::endl;
      }
      RNEA << Node::Body::name << "::Fext_in_0 = Force(n,f);" << std::endl;


      // recursion on children
      rnea< typename Node::Child1, confVector, true >::run(q, dq, ddq);
      rnea< typename Node::Child2, confVector, true >::run(q, dq, ddq);
      rnea< typename Node::Child3, confVector, true >::run(q, dq, ddq);

      // backward computations follow
      // τi = SiT * fi
/*
      Node::Joint::torque = Node::Joint::S.transpose()
                          * Node::Joint::f.toVector();
*/

      // f
      for(int i=0; i<3; i++)
      {
        RNEA
          << "n[" << i << "] = " << Node::Joint::f.n()[i] << ";" << std::endl
          << "f[" << i << "] = " << Node::Joint::f.f()[i] << ";" << std::endl
          << std::endl;
      }
      RNEA << Node::Joint::name << "::f = Force(n,f);" << std::endl;

/*
      // torque
      for(int i=0; i<6; i++)
      {
        RNEA
          << Node::Joint::name << "::torque[" << i << "] = " << Node::Joint::torque[i] << ";" << std::endl;
      }
*/

/*
      // SYMBOLIC
      logname = logdir + Node::Joint::name + "_torque.log";
      SLOG.open(logname.c_str(), std::ofstream::out);
      for(int i=0; i<6; i++)
        SLOG << Node::Joint::torque[i] << std::endl;
      SLOG.close();
*/

      RNEA << "  }"                                 << std::endl
           << "    ::gettimeofday(&tv_stop, NULL);" << std::endl
           << std::endl
           << "    inner_loop_time = ( tv_stop.tv_sec - tv_start.tv_sec ) * TICKS_PER_SECOND" << std::endl
           << "               + ( tv_stop.tv_usec - tv_start.tv_usec );" << std::endl
           << "    time_usec += inner_loop_time;" << std::endl
           << "    // Log inner_loop_time to allow for statistical computations " << std::endl
           << "    perf_log << (double)inner_loop_time/(double)N2 << std::endl;" << std::endl
           << "  }" << std::endl
           << "  // Output global average execution time" << std::endl
           << "  std::cout" << std::endl
           << "    << \"RNEA execution time = \" << (double)time_usec/(double)(N1*N2) << \"µs\"" << std::endl
           << "    << std::endl;" << std::endl
           << "  perf_log.close();" << std::endl
           << "}" << std::endl;
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
