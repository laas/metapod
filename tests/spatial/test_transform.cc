// Copyright 2013
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

#include <metapod/tools/common.hh>

using namespace metapod;
using namespace metapod::Spatial;

template< typename RotationClass>
struct TestTransform
{
  void run() const
  {
    FloatType angle = 0.5;
    FloatType c = std::cos(angle), s = std::sin(angle);                     


    TransformT<RotationClass> Xj;  
    Xj = Spatial::TransformT<RotationClass>
      (RotationClass(c,s),
       Vector3d::Zero());	    
    std::cout << "Xj: " << std::endl << Xj << std::endl;
    
    TransformT<RotationMatrixIdentity> Xt;
    Xt = Spatial::TransformT<Spatial::RotationMatrixIdentity>
      (Spatial::RotationMatrixIdentity(),Vector3d(0.0,0.0,0.0));
    
    TransformT<typename rm_mul_op<RotationClass,RotationMatrixIdentity>::rm > sXp;
    sXp = Xj *Xt;
    std::cout  << sXp << std::endl;
  }
};

BOOST_AUTO_TEST_CASE(test_transform)
{

  TestTransform<Spatial::RotationMatrixAboutX> testx;
  testx.run();
  TestTransform<Spatial::RotationMatrixAboutY> testy;
  testy.run();
  TestTransform<Spatial::RotationMatrixAboutZ> testz;
  testz.run();
}
