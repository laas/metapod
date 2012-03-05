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

BOOST_AUTO_TEST_CASE (test_rnea)
{
  // Perf test
# ifdef METAPOD_PERF_TEST
  long TICKS_PER_SECOND = 1000000;
  struct timeval tv_start, tv_stop;
  struct timezone tz;
  int N=10000;
# endif

  // Set configuration vectors (q, dq, ddq) to reference values.
  vectorN q(Robot::nbDof), dq(Robot::nbDof), ddq(Robot::nbDof);

  std::ifstream qconf(TEST_DIRECTORY "/q.conf");
  std::ifstream dqconf(TEST_DIRECTORY "/dq.conf");
  std::ifstream ddqconf(TEST_DIRECTORY "/ddq.conf");

  initConf<Robot::Tree>(qconf, q);
  initConf<Robot::Tree>(dqconf, dq);
  initConf<Robot::Tree>(ddqconf, ddq);

  qconf.close();
  dqconf.close();
  ddqconf.close();

  // Apply the RNEA to the metapod multibody and print the result in a log file.
# ifdef METAPOD_PERF_TEST
  ::gettimeofday(&tv_start, &tz);
  for(int i=0; i<N; i++)
  {
# endif
    // Run the RNEA to compute the torque
    rnea<Robot::Tree>(q, dq, ddq);
# ifdef METAPOD_PERF_TEST
  }
  ::gettimeofday(&tv_stop, &tz);
  long time_usec = ( tv_stop.tv_sec - tv_start.tv_sec ) * TICKS_PER_SECOND + ( tv_stop.tv_usec - tv_start.tv_usec );
  std::cerr << "RNEA execution time = " << (double)time_usec/(double)N << "µs" << std::endl;
# endif
  std::ofstream log("rnea.log", std::ofstream::out);
  printTorques<Robot::Tree>(log);
  log.close();

  // Compare results with reference file
  double x,y;
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
                 && "Difference found in log and reference files (rnea.log and rnea.ref).");
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
}

#endif
