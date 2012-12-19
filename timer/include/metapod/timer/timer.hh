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
 * This file defines the interface of a simple portable timer, modelled
 * after the one included in boost (starting with boost 1.48).
 */
#ifndef METAPOD_TIMER_TIMER_HH
# define METAPOD_TIMER_TIMER_HH
#include <metapod/timer/config.hh>

namespace metapod
{
  // interface of the class, modelled after boost new times (boost 1.48)
  class METAPOD_TIMER_DLLAPI Timer
  {
  public:
    Timer() {}; // timer is started automatically after construction
    virtual void stop() = 0;
    virtual void resume() = 0;
    virtual void start() = 0;
    virtual double elapsed_wall_clock_time_in_us() = 0;
    virtual ~Timer() {};
  };

  // factory function
  METAPOD_TIMER_DLLAPI Timer* make_timer(void);
}
#endif
