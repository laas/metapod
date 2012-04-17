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
 * This test applies the rnea on a test model with a reference configuration, then compares the computed torques with the reference torques
 */

#ifndef METAPOD_TEST_GINAC_CC
# define METAPOD_TEST_GINAC_CC

// Common test tools
# include "common.hh"

using namespace simplehumanoid;
typedef Eigen::Matrix< FloatType, Robot::nbDof, 1 > confVector;

BOOST_AUTO_TEST_CASE (test_ginac)
{
  confVector q, dq, ddq;

  initSymbols< Robot::Tree >::run();
  initConfSymbolic< Robot::Tree, confVector >::run(q, dq, ddq);
//  jcalcSymbolic< Robot::Tree, confVector >::run(q, dq);

  rnea< Robot::Tree, confVector >::run(q, dq, ddq);
  std::ofstream log("rnea.log", std::ofstream::out);
//  printTorquesSymbolic<Robot::Tree>(log);

//  std::cout << LLEG_ANKLE_R::name << "::sXp = \n" << LLEG_ANKLE_R::sXp << std::endl;
//  std::cout << LLEG_ANKLE_R::name << "::vj = \n" << LLEG_ANKLE_R::vj << std::endl;
//  std::cout << LLEG_LINK6::name << "::vi = \n" << LLEG_LINK6::vi << std::endl;

//  log << LLEG_ANKLE_R::name << "::torque =\n" << LLEG_ANKLE_R::torque << std::endl;
//  log << LARM_WRIST_R::name << "::torque =\n" << LARM_WRIST_R::torque << std::endl;
//  log << LARM_ELBOW::name << "::torque =\n" << LARM_ELBOW::torque << std::endl;
//  log << LARM_SHOULDER_P::name << "::torque =\n" << LARM_SHOULDER_P::torque << std::endl;
//  log << CHEST::name << "::torque =\n" << CHEST::torque << std::endl;
//  log << WAIST::name << "::torque[0] =\n" << WAIST::torque[0] << std::endl;
//  log << WAIST::name << "::torque[2] =\n" << WAIST::torque[2] << std::endl;
//  log << WAIST::name << "::torque[4] =\n" << WAIST::torque[4] << std::endl;

  GiNaC::lst lst1;
  lst1 = cos(q(0)), sin(q(0)),
         dq(0), ddq(0),
         cos(q(1)), sin(q(1)),
         dq(1), ddq(1),
         cos(q(2)), sin(q(2)),
         dq(2), ddq(2),
         cos(q(3)), sin(q(3)),
         dq(3), ddq(3),
         cos(q(4)), sin(q(4)),
         dq(4), ddq(4),
         cos(q(5)), sin(q(5)),
         dq(5), ddq(5),
         cos(q(6)), sin(q(6)),
         dq(6), ddq(6),
         cos(q(7)), sin(q(7)),
         dq(7), ddq(7),
         cos(q(8)), sin(q(8)),
         dq(8), ddq(8),
         cos(q(9)), sin(q(9)),
         dq(9), ddq(9),
         cos(q(10)), sin(q(10)),
         dq(10), ddq(10),
         cos(q(11)), sin(q(11)),
         dq(11), ddq(11),
         cos(q(12)), sin(q(12)),
         dq(12), ddq(12),
         cos(q(13)), sin(q(13)),
         dq(13), ddq(13),
         cos(q(14)), sin(q(14)),
         dq(14), ddq(14),
         cos(q(15)), sin(q(15)),
         dq(15), ddq(15),
         cos(q(16)), sin(q(16)),
         dq(16), ddq(16),
         cos(q(17)), sin(q(17)),
         dq(17), ddq(17),
         cos(q(18)), sin(q(18)),
         dq(18), ddq(18),
         cos(q(19)), sin(q(19)),
         dq(19), ddq(19),
         cos(q(20)), sin(q(20)),
         dq(20), ddq(20),
         cos(q(21)), sin(q(21)),
         dq(21), ddq(21),
         cos(q(22)), sin(q(22)),
         dq(22), ddq(22),
         cos(q(23)), sin(q(23)),
         dq(23), ddq(23),
         cos(q(24)), sin(q(24)),
         dq(24), ddq(24),
         cos(q(25)), sin(q(25)),
         dq(25), ddq(25),
         cos(q(26)), sin(q(26)),
         dq(26), ddq(26),
         cos(q(27)), sin(q(27)),
         dq(27), ddq(27),
         cos(q(28)), sin(q(28)),
         dq(28), ddq(28),
         cos(q(29)), sin(q(29)),
         dq(29), ddq(29),
         cos(q(30)), sin(q(30)),
         dq(30), ddq(30),
         cos(q(31)), sin(q(31)),
         dq(31), ddq(31),
         cos(q(32)), sin(q(32)),
         dq(32), ddq(32),
         cos(q(33)), sin(q(33)),
         dq(33), ddq(33),
         cos(q(34)), sin(q(34)),
         dq(34), ddq(34);

  GiNaC::lst lst2;
  lst2 = 
         cos(q(34)), sin(q(34)),
         cos(q(33)), sin(q(33)),
         cos(q(32)), sin(q(32)),
         cos(q(31)), sin(q(31)),
         cos(q(30)), sin(q(30)),
         cos(q(29)), sin(q(29)),
         cos(q(28)), sin(q(28)),
         cos(q(27)), sin(q(27)),
         cos(q(26)), sin(q(26)),
         cos(q(25)), sin(q(25)),
         cos(q(24)), sin(q(24)),
         cos(q(23)), sin(q(23)),
         cos(q(22)), sin(q(22)),
         cos(q(21)), sin(q(21)),
         cos(q(20)), sin(q(20)),
         cos(q(19)), sin(q(19)),
         cos(q(18)), sin(q(18)),
         cos(q(17)), sin(q(17)),
         cos(q(16)), sin(q(16)),
         cos(q(15)), sin(q(15)),
         cos(q(14)), sin(q(14)),
         cos(q(13)), sin(q(13)),
         cos(q(12)), sin(q(12)),
         cos(q(11)), sin(q(11)),
         cos(q(10)), sin(q(10)),
         cos(q(9)), sin(q(9)),
         cos(q(8)), sin(q(8)),
         cos(q(7)), sin(q(7)),
         cos(q(6)), sin(q(6)),
         cos(q(5)), sin(q(5)),
         cos(q(4)), sin(q(4)),
         cos(q(3)), sin(q(3)),
         cos(q(2)), sin(q(2)),
         cos(q(1)), sin(q(1)),
         cos(q(0)), sin(q(0)),

//  GiNaC::ex ex1 = LLEG_ANKLE_R::torque.expand();
//  ex1 = ex1.collect(lst1);
//  log << "LLEG_ANKLE_R::torque\n" << ex1 << std::endl;

  log.close();
}

#endif
