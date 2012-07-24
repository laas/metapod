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
 * 15 dof sample model structure definition.
 */

#ifndef METAPOD_SAMPLE_15_DOF_ROBOT_HH
# define METAPOD_SAMPLE_15_DOF_ROBOT_HH

# include "metapod/tools/common.hh"
# include "joint.hh"
# include "body.hh"

namespace metapod
{
  namespace sample_15_dof
  {
    class METAPOD_DLLEXPORT Robot
    {
      public:
        enum { NBDOF = 15 };
        static Eigen::Matrix< FloatType, NBDOF, NBDOF > H;
        typedef Eigen::Matrix< FloatType, NBDOF, 1 > confVector;
        typedef Node< B0,
                      J0,
                      Node< B1,
                            J1,
                            Node< B2,
                                  J2,
                                  Node< B3, J3>,
                                  Node< B4, J4>
                                >,
                            Node< B5,
                                  J5,
                                  Node< B6, J6>,
                                  Node< B7, J7>
                                >
                          >,
                      Node< B8,
                            J8,
                            Node< B9,
                                  J9,
                                  Node< B10, J10>,
                                  Node< B11, J11>
                                >,
                            Node< B12,
                                  J12,
                                  Node< B13, J13>,
                                  Node< B14, J14>
                                >
                          >
                    > Tree;
    };
  } // end of namespace sample_15_dof
} // end of namespace metapod

#endif
