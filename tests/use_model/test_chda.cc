// Copyright 2014
//
// Nuno Guedelha (LAAS, CNRS)
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

// This test applies the chda (composite hybrid dynamics algorithm) on a test model with a reference configuration,
// then compares the computed torques and accelerations with the reference 
// torques and accelerations

// Common test tools
#include "common.hh"
#include <metapod/timer/timer.hh>
#include <metapod/algos/chda.hh>
#include <metapod/algos/rnea.hh>

using namespace metapod;

typedef double LocalFloatType;
BOOST_AUTO_TEST_CASE (test_chda)
{
  typedef CURRENT_MODEL_ROBOT<LocalFloatType> Robot;
  // Set configuration vectors (q, dq, ddq, torques) to reference values.
  Robot::confVector q, dq, ddq, torques, ref_ddq, ref_torques;
  Robot robot;
  
  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  std::ifstream dqconf(TEST_DIRECTORY "/dq.conf");
  std::ifstream ddqconf(TEST_DIRECTORY "/chdaDdq.ref");

  initConf<Robot>::run(qconf, q);
  initConf<Robot>::run(dqconf, dq);
  initConf<Robot>::run(ddqconf, ref_ddq);

  rnea<Robot, true>::run(robot, q, dq, ref_ddq);
  getTorques(robot, ref_torques);
  std::ofstream refTorquesconf("chdaTorques.ref", std::ofstream::out);
  printConf<Robot>(ref_torques, refTorquesconf);
  refTorquesconf.close();
  std::ifstream torquesconf("chdaTorques.ref");

  initConf<Robot, HYBRID_DDQ>::run(ddqconf, ddq);
  initConf<Robot, HYBRID_TORQUES>::run(torquesconf, torques);

  qconf.close();
  dqconf.close();
  ddqconf.close();
  torquesconf.close();
  
  // log the ddq and torques configuration updated with FD/ID joint modes
  std::ofstream logTorquesFdIdInit("chdaTorques.conf", std::ofstream::out);
  printConf<Robot>(torques, logTorquesFdIdInit);
  logTorquesFdIdInit.close();
  std::ofstream logDdqFdIdInit("chdaDdq.conf", std::ofstream::out);
  printConf<Robot>(ddq, logDdqFdIdInit);
  logDdqFdIdInit.close();

  
  // Apply the CHDA (Hybrid Dynamics) to the metapod multibody and print the result in a log file.

  chda<Robot, false>::run(robot, q, dq, ddq, torques);
  
  // Inertia H results
  const char H_result_file[] = "chdaH.log";
  std::ofstream logH(H_result_file, std::ofstream::out);
  logH << "generalized_mass_matrix\n" << robot.H << std::endl;
  logH.close();
  
  const char torques_result_file[] = "chdaTorques.log";
  std::ofstream logTorques(torques_result_file, std::ofstream::out);
  printConf<Robot>(torques, logTorques);
  logTorques.close();
  
  const char ddq_result_file[] = "chdaDdq.log";
  std::ofstream logDdq(ddq_result_file, std::ofstream::out);
  printConf<Robot>(ddq, logDdq);
  logDdq.close();

  // Compare results with reference files
  //compareLogs(H_result_file, TEST_DIRECTORY "/crba.ref", 1e-3);  // commented because H is not computed if all joints are in Inverse Dyn mode.
  compareLogs(torques_result_file, "chdaTorques.ref", 1e-3);
  compareLogs(ddq_result_file, TEST_DIRECTORY "/chdaDdq.ref", 15e-4);
  
  //****** RANDOM VECTORS *************************************************************
  
  Timer* timer = make_timer(); timer->start(); timer->stop();
  int outer_loop_count;
  int inner_loop_count;

  for(outer_loop_count=1; outer_loop_count<100; ++outer_loop_count)
  {
    q = Robot::confVector::Random() * M_PI;
    dq = Robot::confVector::Random();
    ref_ddq = Robot::confVector::Random();
    rnea<Robot, true>::run(robot, q, dq, ref_ddq);
    getTorques(robot, ref_torques);
    
    initConf<Robot, HYBRID_DDQ, Robot::confVector>::run(ref_ddq, ddq);
    initConf<Robot, HYBRID_TORQUES, Robot::confVector>::run(ref_torques, torques);

    for(inner_loop_count=1; inner_loop_count<1000; inner_loop_count++)
    {
      timer->resume();

      // Apply the CHDA (Hybrid Dynamics) to the metapod multibody
      chda<Robot, true>::run(robot, q, dq, ddq, torques);

      timer->stop();
    }

    // compare results to ref data
    BOOST_CHECK(ref_torques.isApprox(torques, 1e-3));
    BOOST_CHECK(ref_ddq.isApprox(ddq, 1e-3));

    std::cout << "+1000 iterations OK" << std::endl;
  }
  double total_time_us = timer->elapsed_wall_clock_time_in_us();
  std::cout << "CHDA average execution time is : "
            << total_time_us/double(inner_loop_count * outer_loop_count)
            << "µs\n";
}
