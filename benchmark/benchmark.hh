// Copyright 2012,
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
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
# include <vector>
# include <iostream>
# include <boost/bind.hpp>
# include <boost/function.hpp>

# include <metapod/timer/timer.hh>

# include <metapod/tools/jcalc.hh>
# include <metapod/tools/bcalc.hh>
# include <metapod/algos/rnea.hh>
# include <metapod/algos/crba.hh>
# include <metapod/tools/jac_point_robot.hh>

namespace metapod
{
  namespace benchmark
  {
    template < typename Robot >
    class Runner
    {
    public:
      typedef typename Robot::confVector confVector;
      typedef boost::function<void(const confVector & q,
                                   const confVector & dq,
                                   const confVector & ddq)> functor_t;

      Runner(functor_t f, const::std::string & msg):
        timer_(make_timer()),
        func_(f),
        msg_(msg),
        inner_loop_max_(1000),
        outer_loop_count_(0)
      {}

      Runner(const Runner& other):
        timer_(make_timer()),
        func_(other.func_),
        msg_(other.msg_),
        inner_loop_max_(other.inner_loop_max_),
        outer_loop_count_(0)
      {}

      Runner& operator=(const Runner& other)
      {
        func_ = other.func_;
        msg_ = other.msg_;
        inner_loop_max_ = other.inner_loop_max_;
        outer_loop_count_ = 0;
        return *this;
      }

      void run(const confVector & q,
               const confVector & dq,
               const confVector & ddq)
      {
        if (!outer_loop_count_)
          timer_->start();
        else
          timer_->resume();
        for(int j=0; j<inner_loop_max_; ++j)
          func_(q, dq, ddq);
        timer_->stop();
        ++outer_loop_count_;
      }

      void print()
      {
        double total_time_us = timer_->elapsed_wall_clock_time_in_us();
        std::cout << msg_ << ": "
                  << total_time_us/double(inner_loop_max_ * outer_loop_count_)
                  << "µs\n";
      }

    private:
      std::auto_ptr<Timer> const timer_;
      functor_t func_;
      std::string msg_;
      int inner_loop_max_;
      int outer_loop_count_;
    };

    // wrapping jac_point_robot directly with boost::bind
    // does not work because the J argument (an Eigen matrix) has
    // alignement constraints.
    template < typename Robot, bool call_bcalc >
    class jac_point_robot_wrapper
    {
    public:
      static void run(typename Robot::confVector q)
      {
        typename jac_point_robot<Robot, call_bcalc>::jacobian_t J;
        jac_point_robot<Robot, call_bcalc>::run(q, J);
      }
    };

    template < typename Robot >
    struct benchmark
    {
      typedef typename Robot::confVector confVector;

      static void run()
      {
        confVector q, dq, ddq;
        // vector of the algorithms we want to benchmark
        std::vector< Runner<Robot> > runners;
        runners.push_back(Runner<Robot>(
            boost::bind<void>(&jcalc<Robot>::run, _1, _2),
            std::string("jcalc")));
        runners.push_back(Runner<Robot>(
            boost::bind<void>(bcalc<Robot>::run, _1),
            std::string("bcalc")));
        runners.push_back(Runner<Robot>(
            boost::bind<void>(rnea<Robot, true>::run, _1, _2, _3),
                  std::string("rnea")));
        runners.push_back(Runner<Robot>(
            boost::bind<void>(rnea<Robot, false>::run, _1, _2, _3),
            std::string("rnea (without jcalc)")));
        runners.push_back(Runner<Robot>(
            boost::bind<void>(crba<Robot, true>::run, _1),
            std::string("crba")));
        runners.push_back(Runner<Robot>(
            boost::bind<void>(crba<Robot, false>::run, _1),
            std::string("crba (without jcalc)")));
        runners.push_back(Runner<Robot>(
            boost::bind<void>(jac_point_robot_wrapper<Robot, false>::run, _1),
            std::string("jac_point_robot (without bcalc)")));

        // tell which model we are running benchmarks on
        std::cout << "*************\n"
                  << "Model NBDOF : " << Robot::NBDOF << std::endl;
        for(int i=0; i<100; ++i)
        {
          q = confVector::Random();
          dq = confVector::Random();
          ddq = confVector::Random();
          for(typename std::vector< Runner<Robot> >::iterator runner=runners.begin();
              runner != runners.end();
              ++runner)
          {
            runner->run(q, dq, ddq);
          }
        }
        // print result
        std::cout << "  average execution time :\n";
        for(typename std::vector< Runner<Robot> >::iterator runner=runners.begin();
            runner != runners.end();
            ++runner)
         {
           runner->print();
         }
      }
    };
    #define BENCHMARK(robot) benchmark<robot::Robot>::run()
  } // end of namespace benchmark
} // end of namespace metapod

#endif
