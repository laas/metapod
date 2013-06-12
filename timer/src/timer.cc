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
 * This file contains 3 implementations of the timer.
 *
 *  * one relies on boost::timer from boost >= 1.48, it is used preferentially
 *
 *  * another relies on boost legacy timer, it shoud be used with Visual Studio
 *    when the new boost timer is not available (they confict)
 *
 *  * another one relies on gettimeofday, for unix systems, when the new boost
 *    timer is not available. Note that the boost legacy timer would "work" too
 *    but it does not measure wall-clock time.
 */
#include <metapod/timer/timer.hh>
#include <cassert>
#include <boost/version.hpp>
// check the version in case the cmake boost timer detection is flawed
#if defined(WITH_BOOST_TIMER) && BOOST_VERSION <= 104800
# undef WITH_BOOST_TIMER
#endif
#ifdef WITH_BOOST_TIMER
# include <boost/timer/timer.hpp> // new boost timer
#else
# include <boost/timer.hpp> // old boost timer
#endif
#ifdef __GNUC__
# include <sys/time.h>
# include <cstddef>
#endif

namespace metapod
{
  namespace
  {
#ifdef WITH_BOOST_TIMER
    // simple wrapper
    class TimerBoostNew : public Timer
    {
    public:
      TimerBoostNew()
      {}

      void stop()
      {
        timer_.stop();
      }

      void start()
      {
        timer_.start();
      }

      void resume()
      {
        timer_.resume();
      }

      double elapsed_wall_clock_time_in_us()
      {
        return timer_.elapsed().wall * 1e-3;
      }

    private:
      boost::timer::cpu_timer timer_;
    };
#else
    // the old boost timer also works on posix systems but on these systems it
    // measures CPU time instead of wall clock time. So let build it,
    // but avoid using it.
    class TimerBoostLegacy : public Timer
    {
    public:
      TimerBoostLegacy():
        is_running_(true),
        time_(0.0)
      {}

      void stop()
      {
        if (is_running_)
        {
          time_ += elapsed_wall_clock_time_in_us_();
          is_running_ = false;
        }
      }

      void resume()
      {
        if (!is_running_)
        {
          timer_.restart();
          is_running_ = true;
        }
      }

      void start()
      {
        time_ = 0.0;
        resume();
      }

      double elapsed_wall_clock_time_in_us()
      {
        if (is_running_)
          return time_ + elapsed_wall_clock_time_in_us_();
        else
          return time_;
      }

    private:
      double elapsed_wall_clock_time_in_us_()
      {
        assert(is_running_);
        return timer_.elapsed() * 1e6;
      }
      bool is_running_;
      double time_;
      boost::timer timer_;
    };
#endif

#ifdef __GNUC__
    class TimerGetTimeOfDay : public Timer
    {
    public:
      TimerGetTimeOfDay():
        is_running_(false),
        start_(),
        time_(0.)
      {
        start();
      }

      void stop()
      {
        if (is_running_)
          time_ += elapsed_wall_clock_time_in_us_();
        is_running_ = false;
      }

      void resume()
      {
        if (!is_running_)
        {
          ::gettimeofday(&start_, NULL);
          is_running_ = true;
        }
      }

      void start()
      {
        time_ = 0L;
        resume();
      }

      double elapsed_wall_clock_time_in_us()
      {
        if (is_running_)
          return static_cast<double>(time_ + elapsed_wall_clock_time_in_us_());
        else
          return static_cast<double>(time_);
      }

    private:
      long elapsed_wall_clock_time_in_us_()
      {
        assert(is_running_);
        struct timeval stop;
        ::gettimeofday(&stop, NULL);
        return (stop.tv_sec - start_.tv_sec) * 1000000 +
               (stop.tv_usec - start_.tv_usec);
      }
      bool is_running_;
      struct timeval start_;
      long time_;
    };
#endif
  }

  // factory function
  Timer* make_timer(void)
  {
#if defined(WITH_BOOST_TIMER)
    return new TimerBoostNew();
#elif defined _MSC_VER
    return new TimerBoostLegacy();
#elif defined __GNUC__
    return new TimerGetTimeOfDay();
#else
# pragma error()
#endif
  }
}
