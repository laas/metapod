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

#include <metapod/tools/joint.hh>


using namespace metapod;
using namespace metapod::Spatial;

template< typename FloatType, typename RotationClass>
struct TestTransform
{
  METAPOD_TYPEDEFS;
  void run() const
  {
    FloatType angle = 0.5;
    FloatType c = std::cos(angle), s = std::sin(angle);                     


    TransformT<FloatType,RotationClass> Xj;  
    Xj = Spatial::TransformT<FloatType,RotationClass>
      (RotationClass(c,s),
       Vector3d::Zero());	    
    std::cout  << "Xj: " << std::endl << Xj << std::endl;
    
    TransformT<FloatType,RotationMatrixIdentity > Xt;
    Xt = Spatial::TransformT<FloatType,RotationMatrixIdentity >
      (RotationMatrixIdentity(),Vector3d(0.0,0.0,0.0));
    
    TransformT<FloatType, typename rm_mul_op<FloatType, RotationClass, RotationMatrixIdentity >::rm > sXp;
    sXp = Xj *Xt;
    std::cout  << sXp << std::endl;

    Force f;
    Xj.apply(f);
    Xj.applyInv(f);
    Xt.apply(f);
    Xt.applyInv(f);
    sXp.apply(f);
    sXp.applyInv(f);

    Motion m;
    Xj.apply(m);
    Xj.applyInv(m);
    Xt.apply(m);
    Xt.applyInv(m);
    sXp.apply(m);
    sXp.applyInv(m);

    Inertia I;
    Xj.apply(I);
    Xj.applyInv(I);
    Xt.apply(I);
    Xt.applyInv(I);
    sXp.apply(I);
    sXp.applyInv(I);

    typename Vector3dTpl<FloatType>::Type v;
    Xj.apply(v);
    Xj.applyInv(v);
    Xt.apply(v);
    Xt.applyInv(v);
    sXp.apply(v);
    sXp.applyInv(v);;

    Vector3d v2;
    Xj.apply(v2);
    Xj.applyInv(v2);
    Xt.apply(v2);
    Xt.applyInv(v2);
    sXp.apply(v2);
    sXp.applyInv(v2);;

  }
};

BOOST_AUTO_TEST_CASE(test_transform)
{
  typedef double FloatType;
  TestTransform<FloatType, Spatial::RotationMatrixAboutXTpl<FloatType> > testx;
  testx.run();
  TestTransform<FloatType, Spatial::RotationMatrixAboutYTpl<FloatType> > testy;
  testy.run();
  TestTransform<FloatType, Spatial::RotationMatrixAboutZTpl<FloatType> > testz;
  testz.run();
}
