// Copyright 2011, 2012, 2013
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
// Olivier Stasse (JRL/LAAS, CNRS/AIST)
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
 * This file run performance tests on metapods algorithms, on a sample model
 */
#include <sys/time.h>

#include <metapod/models/simple_humanoid/simple_humanoid.hh>
#include <metapod/algos/rnea.hh>

#include <metapod/tools/print.hh>

#include "benchmark.hh"
using namespace metapod::benchmark;

typedef double LocalFloatType;
typedef typename metapod::simple_humanoid<LocalFloatType> Robot;
typedef typename Robot::confVector confVector;

void load_vector_file(confVector &aVector,
                          std::string &filename)
{
  std::ifstream aif;
  aif.open(filename.c_str(),std::ofstream::in);
  if (aif.is_open())
    {
      for(unsigned int i=0;i<aVector.size();i++)
        aif >> aVector(i) ;
      aif.close();
    }
}

void build_file_name(int index,
                     std::string &prefix,
                     std::string &filename)
{  
  std::ostringstream aoss;
  aoss << prefix ;
  char prevfill;
  prevfill = aoss.fill('0');
  int prevwidth;
  prevwidth=aoss.width(10);
  aoss << index << ".dat";
  aoss.fill(prevfill); aoss.width(prevwidth);
  filename = aoss.str();
}

void load_state_vectors(confVector &Q, 
                        confVector &DQ,
                        confVector &DDQ,
                        confVector &Tau,
                        int index)
{
  std::string filename;
  std::string prefix;
  // Save Q.
  prefix="/tmp/q";
  build_file_name(index,prefix,filename);
  load_vector_file(Q,filename);
  // Save DQ.
  prefix="/tmp/dq";
  build_file_name(index,prefix,filename);
  load_vector_file(DQ,filename);
  // Save DDQ.
  prefix="/tmp/ddq";
  build_file_name(index,prefix,filename);
  load_vector_file(DDQ,filename);
  // Save Tau.
  prefix="/tmp/tau";
  build_file_name(index,prefix,filename);
  load_vector_file(Tau,filename);

}

#define MODE_READ 0
#define MODE_WRITE 1
#define MODE_VOID 2
#include <fenv.h>
#include <pmmintrin.h>

int main()
{
  _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

  int mode=MODE_VOID;
  /* Test computation time */
  srand (0); // init random timer
  unsigned int number_of_it = 1000000;
  //unsigned int number_of_it = 10;
  unsigned int index = 0;
  
  confVector Q; // Generalized coordinates
  confVector QDot ; // Velocity of generalized coordinates
  confVector QDDot; // Acceleration of generalized coodinates
  confVector Tau,torques ; // Torque actuations

  struct timeval tvalBefore, tvalAfter;
  
  double t_start = 0.;
  double t_end = 0.;

  double t_total = 0.;
  double t_loop_max = 0.;
  double t_loop_min = 100.;
  double SumOverTau = 0.;

  Robot robot;

  while (index < number_of_it)
    {
      /* Init randomly the configuration */
      for (unsigned int i = 0; i < Robot::NBDOF; i++) {
        if (i < 6)
          Q[i] = (5.0*(double)rand())/((double)RAND_MAX);
        else
          Q[i] = (M_PI*(double)rand())/((double)RAND_MAX);

        QDot[i] = ((double)rand())/((double)RAND_MAX);
        QDDot[i] = ((double)rand())/((double)RAND_MAX);
      }
      if (mode==MODE_READ)
        load_state_vectors(Q,QDot,QDDot,Tau,index);

      gettimeofday (&tvalBefore, NULL); // time before computing inertia
      
      metapod::rnea<Robot,true>::run(robot,Q, QDot, QDDot);
      
      gettimeofday (&tvalAfter, NULL); // time after computing inertia
      t_start = tvalBefore.tv_sec + 1e-6 * tvalBefore.tv_usec;
      t_end = tvalAfter.tv_sec + 1e-6 * tvalAfter.tv_usec;
      
      SumOverTau=0.0; getTorques(robot,torques);
      for (unsigned int i = 0; i < Robot::NBDOF; i++)
        {
          double lsot = Tau[i]-torques[i];
          SumOverTau += lsot*lsot;
        }

      if (mode==MODE_READ)
        {
          if (SumOverTau>0.0)
            {
              std::cerr << "Difference at iteration " << index << " : "
                        << SumOverTau << std::endl;
              std::cerr << "RBDL            METAPOD" << std::endl;
              std::cerr.precision(10);
              std::cerr.width(15);
              //std::cerr.setf( std::ios::fixed, std:: ios::floatfield );
              for (unsigned int i = 0; i < Robot::NBDOF; i++)
                std::cerr << Tau[i] << " " << torques[i] << " " 
                          << Q[i] << " " << QDot[i] << " " << QDDot[i] << " " << std::endl;
              return -1;
            }
          else
            std::cout << "Iteration " << index << " succeeded" << std::endl;
        }
      /* Update values */
      double t_loop = t_end - t_start;
      t_loop_max = std::max (t_loop_max, t_loop);
      t_loop_min = std::min (t_loop_min, t_loop);
      
      t_total += t_loop;
      ++index;
    }
      
  std::cout << std::endl << "Tau: " << SumOverTau << std::endl;
  std::cout << "Computation time for RNEA (" << index << " loops) : " << t_total << " s" << std::endl;
  std::cout << "Average time : " << 1e6 * t_total / (double)index << " us" << std::endl;
  //std::cout << "Max time among loops : " << 1e6 * t_loop_max << " " << MU << "s" << std::endl;
  //std::cout << "Min time among loops : " << 1e6 * t_loop_min << " " << MU << "s"  << std::endl;
  std::cout << "\n\n" << std::endl;

}
