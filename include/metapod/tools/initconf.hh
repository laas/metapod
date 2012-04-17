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
 * This file contains a method setting a configuration vector from a file.
 */

#ifndef METAPOD_INITCONF_HH
# define METAPOD_INITCONF_HH

# include "common.hh"
# include <boost/lexical_cast.hpp>

namespace metapod
{

  // Look for a string in a std::ofstream
  void findString(std::string s_, std::ifstream & is)
  {
    is.clear(); is.seekg(0);
    std::string s;
    while(is >> s)
    {
      if(!s.compare(s_))
        break;
    }
  }
  
  // Return vector constructed from log file,
  // as printed by the printConf method.
  template< typename Tree, typename confVector > struct initConf
  {
    static void run(std::ifstream & log, confVector & v)
    {
      typedef Tree Node;
      findString(Node::Joint::name, log);
      for(int i=0; i<Node::Joint::NBDOF; i++)
        log >> v[Node::Joint::positionInConf+i];
      initConf<typename Node::Child1, confVector >::run(log,v);
      initConf<typename Node::Child2, confVector >::run(log,v);
      initConf<typename Node::Child3, confVector >::run(log,v);
    }
  };
  
  template< typename confVector > struct initConf< NC, confVector >
  {
    static void run(std::ifstream &, confVector &){}
  };

  template< typename Tree, typename confVector > struct initConfSymbolic
  {
    static void run(confVector & q, confVector & dq, confVector & ddq)
    {
      typedef Tree Node;
      std::string q_str, dq_str, ddq_str;
      for(int i=0; i<Node::Joint::NBDOF; i++)
      {
//        q_str = Node::Joint::name + "::q[" + boost::lexical_cast<std::string>(i) + "]";
//        dq_str = Node::Joint::name + "::q[" + boost::lexical_cast<std::string>(i) + "]";
//        ddq_str = Node::Joint::name + "::q[" + boost::lexical_cast<std::string>(i) + "]";

        q_str = "q[" + boost::lexical_cast<std::string>(Node::Joint::positionInConf+i) + "]";
        dq_str = "dq[" + boost::lexical_cast<std::string>(Node::Joint::positionInConf+i) + "]";
        ddq_str = "ddq[" + boost::lexical_cast<std::string>(Node::Joint::positionInConf+i) + "]";
//        q_str = "q" + boost::lexical_cast<std::string>(Node::Joint::label) + "_" + boost::lexical_cast<std::string>(i);
//        dq_str = "dq" + boost::lexical_cast<std::string>(Node::Joint::label) + "_" + boost::lexical_cast<std::string>(i);
//        ddq_str = "ddq" + boost::lexical_cast<std::string>(Node::Joint::label) + "_" + boost::lexical_cast<std::string>(i);

        q[Node::Joint::positionInConf + i] = GiNaC::symbol(q_str);
        dq[Node::Joint::positionInConf + i] = GiNaC::symbol(dq_str);
        ddq[Node::Joint::positionInConf + i] = GiNaC::symbol(ddq_str);
      }
      initConfSymbolic< typename Node::Child1, confVector >::run(q, dq, ddq);
      initConfSymbolic< typename Node::Child2, confVector >::run(q, dq, ddq);
      initConfSymbolic< typename Node::Child3, confVector >::run(q, dq, ddq);
    }
  };

  template< typename confVector > struct initConfSymbolic< NC, confVector >
  {
    static void run(confVector &, confVector &, confVector &){}
  };

  template< typename Tree, typename confVector >
  struct jcalcSymbolic
  {
    static void run(const confVector & q, const confVector & dq)
    {
      typedef Tree Node;
      std::string E_ij, r_i;
      matrix3d E;
      vector3d r;
      for (int i=0; i<3; i++)
        for(int j=0; j<3; j++)
        {
          E_ij = "X" + boost::lexical_cast<std::string>(Node::Joint::label) + "_" + boost::lexical_cast<std::string>(i) + "_" + boost::lexical_cast<std::string>(j) + "::E";
          E(i,j) = GiNaC::symbol(E_ij);
        }
      for(int i=0; i<3; i++)
      {
        r_i = "X" + boost::lexical_cast<std::string>(Node::Joint::label) + "_" + boost::lexical_cast<std::string>(i) + "::r";
        r(i) = GiNaC::symbol(r_i);
      }
      Node::Joint::sXp = Spatial::Transform(E,r);

      switch(Node::Joint::type)
      {
        case metapod::REVOLUTE:
          Node::Joint::vj.w(vector3d(dq[Node::Joint::positionInConf], 0, 0));
          break;
        case metapod::FREE_FLYER:
          matrix3d localR;
          FloatType cPsi   = cos(q(Node::Joint::positionInConf + 3)), sPsi   = sin(q(Node::Joint::positionInConf + 3)),
                    cTheta = cos(q(Node::Joint::positionInConf + 4)), sTheta = sin(q(Node::Joint::positionInConf + 4)),
                    cPhi   = cos(q(Node::Joint::positionInConf + 5)), sPhi   = sin(q(Node::Joint::positionInConf + 5));
          localR(0,0) = cTheta * cPhi;
          localR(0,1) = cTheta * sPhi;
          localR(0,2) = -sTheta;
          localR(1,0) = -cPsi * sPhi + cPhi * sPsi * sTheta;
          localR(1,1) = cPsi * cPhi + sPsi * sTheta * sPhi;
          localR(1,2) = cTheta * sPsi;
          localR(2,0) = cPsi * cPhi * sTheta + sPhi * sPsi;
          localR(2,1) = -cPhi * sPsi + cPsi * sTheta * sPhi;
          localR(2,2) = cPsi * cTheta;
          for (int i=0; i<3; i++)
            for(int j=0; j<3; j++)
            {
//              Node::Joint::S(i,j) = localR(i,j);
//              Node::Joint::S(i+3, j+3) = localR(i,j);
//                std::cerr << Node::Joint::name << "::S(" << i << "," << j << ") = " << localR(i,j) << std::endl;
            }

          vector6d dqi;
          for(int i=0; i<6; i++)
            dqi[i] = dq(Node::Joint::positionInConf+i);
          vector6d tmp = Node::Joint::S * dqi;
          Node::Joint::vj = Motion(tmp);
//          Node::Joint::vj = Motion(Node::Joint::S*dqi);
          break;
      }

      jcalcSymbolic< typename Node::Child1, confVector >::run(q, dq);
      jcalcSymbolic< typename Node::Child2, confVector >::run(q, dq);
      jcalcSymbolic< typename Node::Child3, confVector >::run(q, dq);
    }
  };

  template< typename confVector >
  struct jcalcSymbolic< NC, confVector >
  {
    static void run(const confVector &, const confVector &) {}
  };

  template< typename Tree > struct initSymbols
  {
    typedef Tree Node;
    static void run()
    {
      vector3d vi_w = vector3d(GiNaC::symbol(Node::Body::name + "::vi.w()[0]"),
                               GiNaC::symbol(Node::Body::name + "::vi.w()[1]"),
                               GiNaC::symbol(Node::Body::name + "::vi.w()[2]"));
      vector3d vi_v = vector3d(GiNaC::symbol(Node::Body::name + "::vi.v()[0]"),
                               GiNaC::symbol(Node::Body::name + "::vi.v()[1]"),
                               GiNaC::symbol(Node::Body::name + "::vi.v()[2]"));
      Node::Body::vi_symbol = Spatial::Motion(vi_w, vi_v);          

      vector3d ai_w = vector3d(GiNaC::symbol(Node::Body::name + "::ai.w()[0]"),
                               GiNaC::symbol(Node::Body::name + "::ai.w()[1]"),
                               GiNaC::symbol(Node::Body::name + "::ai.w()[2]"));
      vector3d ai_v = vector3d(GiNaC::symbol(Node::Body::name + "::ai.v()[0]"),
                               GiNaC::symbol(Node::Body::name + "::ai.v()[1]"),
                               GiNaC::symbol(Node::Body::name + "::ai.v()[2]"));
      Node::Body::ai_symbol = Spatial::Motion(ai_w, ai_v);

      vector3d F_n = vector3d(GiNaC::symbol(Node::Body::name + "::Fext.n()[0]"),
                               GiNaC::symbol(Node::Body::name + "::Fext.n()[1]"),
                               GiNaC::symbol(Node::Body::name + "::Fext.n()[2]"));
      vector3d F_f = vector3d(GiNaC::symbol(Node::Body::name + "::Fext.f()[0]"),
                               GiNaC::symbol(Node::Body::name + "::Fext.f()[1]"),
                               GiNaC::symbol(Node::Body::name + "::Fext.f()[2]"));
      Node::Body::Fext_in_0_symbol = Spatial::Force(F_n, F_f);

      initSymbols< typename Node::Child1 >::run();
      initSymbols< typename Node::Child2 >::run();
      initSymbols< typename Node::Child3 >::run();
    }
  };

  template<> struct initSymbols<NC>
  {
    static void run() {}
  };

} // end of namespace metapod.

#endif
