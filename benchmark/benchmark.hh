// Copyright 2012,
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
 * This file contains the necessary tools for benchmarking.
 * The benchmark runs a method N times, and gives the average execution time.
 * In order to make sure the results are valid, several random configurations
 * are generated and tested. However, generating a random configuration has a
 * visible impact on the execution time. Therefore, we would like to proceed
 * as follows :
 * - A random configuration is generated
 * - The timer is started
 * - The tested method is run
 * - The timer is stopped
 * - We start over N times and compute the average execution time.
 * The problem is, the precision of the timer is 1µs, which is not high enough
 * to give proper results on a single iteration of any of the tested
 * algorithms.
 * We thus proceed a bit differently, by adding an outer loop (with N1
 * iterations), in which we generate an random configuration. Then we run N2
 * times the tested method with that configuration. The timer is started and
 * stopped outside this inner loop, thus solving the precision issue, while
 * still ensuring the results do not depend on a particular configuration.
 */


#ifndef METAPOD_BENCHMARK_HH
# define METAPOD_BENCHMARK_HH

# include <iostream>
# include <sys/time.h>

# include <metapod/tools/jac_point_robot.hh>
# include <metapod/algos/rnea.hh>
# include <metapod/algos/crba.hh>
# include <metapod/tools/jcalc.hh>
# include <metapod/tools/bcalc.hh>

namespace metapod
{
  namespace benchmark
  {
    const int N1 = 100;
    const int N2 = 1000;
    enum{ JCALC, RNEA, RNEA_WITHOUT_JCALC, CRBA, CRBA_WITHOUT_JCALC,
          JAC_POINT_ROBOT };
    const int TICKS_PER_SECOND = 1e6;
    
    class Timer
    {
      public:
        Timer() : m_start(), m_stop(), t(0) {};
    
        void start() { ::gettimeofday(&m_start, NULL); }
    
        void stop()
        {
          ::gettimeofday(&m_stop, NULL);
          t += ( m_stop.tv_sec - m_start.tv_sec ) * TICKS_PER_SECOND
               + ( m_stop.tv_usec - m_start.tv_usec );
        }
    
        void reinit() { t = 0; }
    
        double get() { return (double)t; }
    
      private:
        struct timeval m_start;
        struct timeval m_stop;
        long t;
    };

    static Timer timer;

    #define BENCHMARK(robot)                                          \
    {                                                                 \
      std::cout << "*************\n"                                  \
                << "Model NBDOF : " << robot::Robot::NBDOF << "\n  "  \
                << "  average execution time :\n";                    \
      typedef robot::Robot::confVector confVector;                    \
      confVector q, dq, ddq;                                          \
        timer.reinit();                                               \
        for(int i=0; i<N1; i++)                                       \
        {                                                             \
          q = confVector::Random();                                   \
          dq = confVector::Random();                                  \
          timer.start();                                              \
          for(int j=0; j<N2; j++)                                     \
          {                                                           \
            metapod::jcalc< robot::Robot >::run(q, dq);               \
          }                                                           \
          timer.stop();                                               \
        }                                                             \
        std::cout << "jcalc: " << timer.get()/double(N1*N2) << "µs\n"; \
        timer.reinit();                                               \
        for(int i=0; i<N1; i++)                                       \
        {                                                             \
          q = confVector::Random();                                   \
          timer.start();                                              \
          for(int j=0; j<N2; j++)                                     \
          {                                                           \
            metapod::bcalc< robot::Robot >::run(q);                   \
          }                                                           \
          timer.stop();                                               \
        }                                                             \
        std::cout << "bcalc: "                                        \
                  << timer.get()/double(N1*N2) << "µs\n";             \
        timer.reinit();                                               \
        for(int i=0; i<N1; i++)                                       \
        {                                                             \
          q = confVector::Random();                                   \
          dq = confVector::Random();                                  \
          ddq = confVector::Random();                                 \
          timer.start();                                              \
          for(int j=0; j<N2; j++)                                     \
          {                                                           \
            metapod::rnea< robot::Robot, true >::run(q, dq, ddq);     \
          }                                                           \
          timer.stop();                                               \
        }                                                             \
        std::cout << "rnea: " << timer.get()/double(N1*N2) << "µs\n"; \
        timer.reinit();                                               \
        for(int i=0; i<N1; i++)                                       \
        {                                                             \
          q = confVector::Random();                                   \
          dq = confVector::Random();                                  \
          ddq = confVector::Random();                                 \
          timer.start();                                              \
          for(int j=0; j<N2; j++)                                     \
          {                                                           \
            metapod::rnea< robot::Robot, false >::run(q, dq, ddq);    \
          }                                                           \
          timer.stop();                                               \
        }                                                             \
        std::cout << "rnea (without jcalc): "                         \
                  << timer.get()/double(N1*N2) << "µs\n";             \
        timer.reinit();                                               \
        for(int i=0; i<N1; i++)                                       \
        {                                                             \
          q = confVector::Random();                                   \
          timer.start();                                              \
          for(int j=0; j<N2; j++)                                     \
          {                                                           \
            metapod::crba< robot::Robot, true >::run(q);              \
          }                                                           \
          timer.stop();                                               \
        }                                                             \
        std::cout << "crba: " << timer.get()/double(N1*N2) << "µs\n"; \
        timer.reinit();                                               \
        for(int i=0; i<N1; i++)                                       \
        {                                                             \
          q = confVector::Random();                                   \
          timer.start();                                              \
          for(int j=0; j<N2; j++)                                     \
          {                                                           \
            metapod::crba< robot::Robot, false >::run(q);             \
          }                                                           \
          timer.stop();                                               \
        }                                                             \
        std::cout << "crba (without jcalc): "                         \
                  << timer.get()/double(N1*N2) << "µs\n";             \
        timer.reinit();                                               \
        for(int i=0; i<N1; i++)                                       \
        {                                                             \
          q = confVector::Random();                                   \
          metapod::jac_point_robot< robot::Robot >::jacobian_t J;     \
          timer.start();                                              \
          for(int j=0; j<N2; j++)                                     \
          {                                                           \
            metapod::jac_point_robot< robot::Robot, false >::run(q, J); \
          }                                                           \
          timer.stop();                                               \
        }                                                             \
        std::cout << "jac_point_robot (without bcalc): "              \
                  << timer.get()/double(N1*N2) << "µs\n";             \
        std::cout << std::endl;                                       \
    }                                                                       

  } // end of namespace benchmark
} // end of namespace metapod

#endif
