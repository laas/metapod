// Copyright 2014
//
// Olivier STASSE (LAAS/CNRS)
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
// GNU General Lesser Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod. If not, see <http://www.gnu.org/licenses/>.
#include <iostream>

#include "../common.hh"

#include <metapod/tools/joint.hh>

using namespace metapod;
using namespace metapod::Spatial;

typedef double FloatType;
typedef RotationMatrixAboutXTpl<FloatType> RX;
typedef RotationMatrixAboutYTpl<FloatType> RY;
typedef RotationMatrixAboutZTpl<FloatType> RZ;

typedef TransformT<FloatType, RotationMatrixAboutXTpl<FloatType> > TX;
typedef TransformT<FloatType, RotationMatrixAboutYTpl<FloatType> > TY;
typedef TransformT<FloatType, RotationMatrixAboutZTpl<FloatType> > TZ;
typedef TransformT<FloatType, RotationMatrixTpl<FloatType> > TG;

BOOST_AUTO_TEST_CASE(test_spatial)
{
  FloatType angle = 0.5;
  FloatType c = std::cos(angle), s = std::sin(angle);                     
  
  TX aTX; aTX = TX(RX(c,s), Eigen::Vector3d::Zero());
  TY aTY; aTY = TY(RY(c,s), Eigen::Vector3d::Zero());
  TZ aTZ; aTZ = TZ(RZ(c,s), Eigen::Vector3d::Zero());
  TG aTG,aTG2;

  aTG = aTY * aTX;
  aTG2 = aTZ * aTG;

  std::cout << "aTG2: " << std::endl
            << " aTG: " << aTZ * aTY * aTX << aTG2 << std::endl;
}

