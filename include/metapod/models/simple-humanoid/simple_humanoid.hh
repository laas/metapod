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
 * Header of the simple humanoid robot model, used for test purpose.
 */

#ifndef METAPOD_SIMPLE_HUMANOID_HH
# define METAPOD_SIMPLE_HUMANOID_HH

# include "metapod/tools/common.hh"
# include "metapod/algos/rnea.hh"
# include "metapod/algos/crba.hh"
# include "metapod/algos/jac_point.hh"
# include "metapod/algos/jac_point_chain.hh"

# include "robot.hh"

extern template struct metapod::crba< metapod::simple_humanoid::Robot , true >;
extern template struct metapod::rnea< metapod::simple_humanoid::Robot , true >;
extern template struct metapod::jac_point_robot< metapod::simple_humanoid::Robot , true >;
extern template struct metapod::crba< metapod::simple_humanoid::Robot , false >;
extern template struct metapod::rnea< metapod::simple_humanoid::Robot , false >;
extern template struct metapod::jac_point_robot< metapod::simple_humanoid::Robot , false >;

#endif
