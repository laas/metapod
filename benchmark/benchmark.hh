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

# include "metapod/algos/rnea.hh"
# include "metapod/algos/crba.hh"

namespace metapod
{
  namespace benchmark
  {
    const int N1 = 100;
    const int N2 = 1000;
    enum{ JCALC, RNEA, RNEA_WITHOUT_JCALC, CRBA, CRBA_WITHOUT_JCALC };
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

    template< typename Robot, int Function > struct call_function;
    
    template< typename Robot, int Function > struct bench
    {
      static void run(int n1 = N1, int n2 = N2)
      {
        typedef typename Robot::confVector confVector;
        confVector q, dq, ddq;
        timer.reinit();
        for(int i=0; i<n1; i++)
        {
          q = confVector::Random();
          dq = confVector::Random();
          ddq = confVector::Random();
          timer.start();
          for(int j=0; j<n2; j++)
          {
            call_function< Robot, Function >::run(q, dq, ddq);
          }
          timer.stop();
        }
        std::string function;
        switch(Function)
        {
          case JCALC:
            function = "jcalc";
            break;
          case RNEA:
            function = "rnea";
            break;
          case RNEA_WITHOUT_JCALC:
            function = "rnea (without jcalc)";
            break;
          case CRBA:
            function = "crba";
            break;
          case CRBA_WITHOUT_JCALC:
            function = "crba (without jcalc)";
            break;
        }
        std::cout << "Model NBDOF : " << Robot::NBDOF << "\n  ";
        std::cout << function << " average execution time = "
                  << timer.get()/double(n1*n2) << "µs\n\n";
      }
    };

    template< typename Robot, int Function > struct call_function
    {
      typedef typename Robot::confVector confVector;
      static void run(confVector & q, confVector & dq, confVector & ddq) {};
    };

    template< typename Robot > struct call_function< Robot, JCALC >
    {
      typedef typename Robot::confVector confVector;
      static void run(confVector & q, confVector & dq, confVector & ddq)
      {
        jcalc< Robot >::run(q, dq);
      }
    };
    
    template< typename Robot > struct call_function< Robot, RNEA >
    {
      typedef typename Robot::confVector confVector;
      static void run(confVector & q, confVector & dq, confVector & ddq)
      {
        rnea< Robot, true >::run(q, dq, ddq);
      }
    };
    
    template< typename Robot > struct call_function< Robot, RNEA_WITHOUT_JCALC >
    {
      typedef typename Robot::confVector confVector;
      static void run(confVector & q, confVector & dq, confVector & ddq)
      {
        rnea< Robot, false >::run(q, dq, ddq);
      }
    };
    
    template< typename Robot > struct call_function< Robot, CRBA >
    {
      typedef typename Robot::confVector confVector;
      static void run(confVector & q, confVector & dq, confVector & ddq)
      {
        crba< Robot, true >::run(q);
      }
    };
    
    template< typename Robot > struct call_function< Robot, CRBA_WITHOUT_JCALC >
    {
      typedef typename Robot::confVector confVector;
      static void run(confVector & q, confVector & dq, confVector & ddq)
      {
        crba< Robot, false >::run(q);
      }
    };
  } // end of namespace benchmark
} // end of namespace metapod

#endif
