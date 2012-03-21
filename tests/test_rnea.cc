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

#ifndef METAPOD_TEST_RNEA_HH
# define METAPOD_TEST_RNEA_HH

// Common test tools
# include "common.hh"

using namespace simplehumanoid;
//typedef Eigen::Matrix< FloatType, Robot::NBDOF, 1 > confVector;

BOOST_AUTO_TEST_CASE (test_rnea)
{
  // Set configuration vectors (q, dq, ddq) to reference values.
  vectorN q(Robot::nbDof), dq(Robot::nbDof), ddq(Robot::nbDof);

  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  std::ifstream dqconf(TEST_DIRECTORY "/dq.conf");
  std::ifstream ddqconf(TEST_DIRECTORY "/ddq.conf");

  initConf< Robot::Tree >::run(qconf, q);
  initConf< Robot::Tree >::run(dqconf, dq);
  initConf< Robot::Tree >::run(ddqconf, ddq);

  qconf.close();
  dqconf.close();
  ddqconf.close();

  // Apply the RNEA to the metapod multibody and print the result in a log file.
  rnea< Robot::Tree >::run(q, dq, ddq);
  std::ofstream log("rnea.log", std::ofstream::out);
  printTorques<Robot::Tree>(log);
  log.close();

  // Compare results with reference file
  FloatType x,y;
  std::string jointname;
  std::string str1, str2;
  std::ifstream result_log("rnea.log");
  std::ifstream ref_log(TEST_DIRECTORY "/rnea.ref");
  while(result_log >> str1)
  {
    if(stringToDouble(str1))
    {
      x = stringToDouble(str1);
      y = getNextDouble(ref_log);
      if(y != 0)
      {
        BOOST_CHECK(compareDouble(x,y,1e-3)
                 && "Difference found in log and reference files\
                    (rnea.log and rnea.ref).");
        if(!compareDouble(x,y,1e-3))
          std::cerr << jointname << "\n\t" << x << "\n\t" << y << std::endl;
      }
    }
    else
    {
      ref_log.clear(); ref_log.seekg(0);
      jointname = str1;
      do
      {
        ref_log >> str2;
      } while(str2.compare(jointname) && !ref_log.eof());
    }
  }
  result_log.close();
  ref_log.close();

  // Perf test
# ifdef METAPOD_PERF_TEST
  long TICKS_PER_SECOND = 1e6;
  struct timeval tv_start, tv_stop;
  int N1 = 10000;
  int N2 = 100;

  std::ofstream perf_log("rnea_perf.log", std::ofstream::out);

  long time_usec = 0;
  long inner_loop_time;
  // Outer loop : generate random configuration
  for(int i=0; i<N1; i++)
  {
    q = vectorN::Random(Robot::nbDof);
    dq = vectorN::Random(Robot::nbDof);
    ddq = vectorN::Random(Robot::nbDof);
    ::gettimeofday(&tv_start, NULL);
    // Inner loop : The timer precision is 1µs, which is not high enough to
    // give proper result on a single iteration 
    for(int k=0; k<N2; k++)
      rnea< Robot::Tree >::run(q, dq, ddq);
    ::gettimeofday(&tv_stop, NULL);
    
    inner_loop_time = ( tv_stop.tv_sec - tv_start.tv_sec ) * TICKS_PER_SECOND
               + ( tv_stop.tv_usec - tv_start.tv_usec );
    time_usec += inner_loop_time;
    // Log inner_loop_time to allow for statistical computations 
    perf_log << (double)inner_loop_time/(double)N2 << std::endl;
  }
  // Output global average execution time
  std::cout
    << "RNEA execution time = " << (double)time_usec/(double)(N1*N2) << "µs"
    << std::endl;
  perf_log.close();
# endif
}

#endif
