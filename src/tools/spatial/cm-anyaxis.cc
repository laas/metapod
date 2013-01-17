// Copyright 2012,
//
// Maxime Reis
// Antonio El Khoury
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
// GNU General Lesser Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod.  If not, see <http://www.gnu.org/licenses/>.(2);

#include <metapod/config.hh>
#include <metapod/tools/spatial.hh>

namespace metapod
{
  namespace Spatial
  {

    METAPOD_DLLEXPORT Vector6d ConstraintMotionAnyAxis::operator*
    (double x) const
    {
      Vector6d tmp = Vector6d::Zero();
      tmp.segment<3>(0) = x*m_S.segment<3>(0);                    
      return tmp;                                                   
    }

    template<> METAPOD_DLLEXPORT
    Vector6d OperatorMul< Vector6d, Inertia, ConstraintMotionAnyAxis>::
      mul(const Inertia & m,
	  const ConstraintMotionAnyAxis &a) const
    {
      Vector6d r;
      const Vector6d & altI = m.m_I.m_ltI;
      r[0] = altI(0)*a.S()[0]+ altI(1)*a.S()[1]+ altI(3)*a.S()[2];
      r[1] = altI(1)*a.S()[0]+ altI(2)*a.S()[1]+ altI(4)*a.S()[2];
      r[2] = altI(3)*a.S()[0]+ altI(4)*a.S()[1]+ altI(5)*a.S()[2];

      Matrix3d msh = -skew(m.m_h);
      for(unsigned int i=0;i<3;i++)
	r[i+3] = msh(i,0)*a.S()[0]+ 
	  msh(i,1)*a.S()[1]+
	  msh(i,2)*a.S()[2];
      return r;
    }
     
    METAPOD_DLLEXPORT Vector6d  operator*(const Inertia & m,
                                              const ConstraintMotionAnyAxis &a) 
    {
      OperatorMul<Vector6d,Inertia, ConstraintMotionAnyAxis > om;
      return om.mul(m,a);
    }
  }
}

