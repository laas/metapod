#include "../common.hh"
#include <metapod/timer/timer.hh>
#include <memory>
#include <cmath>

void burn_some_time() {
  volatile float res = 0;
  const int NB_ITERATIONS=10000;
  for(int i=0; i<NB_ITERATIONS; ++i) {
    res += std::pow(-1., 2) * std::sqrt(static_cast<double>(i));
  }
}

BOOST_AUTO_TEST_CASE(test_timer) {
  const std::auto_ptr<metapod::Timer> timer(metapod::make_timer());
  timer->start();
  burn_some_time();
  timer->stop();
  double time0 = timer->elapsed_wall_clock_time_in_us();
  burn_some_time();
  timer->resume();
  burn_some_time();
  timer->stop();
  double time1 = timer->elapsed_wall_clock_time_in_us();
  // std::cout << "time0: " << time0 << "  time1: " << time1 << std::endl;
  const double resolution = 5;
  // in theory, time1 = 2* time0
  BOOST_CHECK(time1 + resolution > time0 - resolution);
}
