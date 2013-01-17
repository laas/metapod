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
    
    template<>
    METAPOD_DLLEXPORT Transform 
    OperatorMul<Transform, 
                TransformX, 
                Transform>::
    mul( const TransformX &aTX,
         const Transform &aT) const
    {
      return Transform(aTX.E()*aT.E(),
                       (Vector3d)(aT.r() + 
                                  aT.E().transpose()*aTX.r()));
    }
    
    METAPOD_DLLEXPORT TransformT<RotationMatrix> 
    operator*(const TransformT<RotationMatrixAboutX> &aTX,
                                const TransformT<RotationMatrix> &aT)
    {
      OperatorMul<Transform,TransformX, Transform> om;
      return om.mul(aTX,aT);
    }

    template <>
    METAPOD_DLLEXPORT Vector6d 
    TransformT<RotationMatrixAboutX>::mulMatrixTransposeBy(Vector6d &aF) const
    {
      Vector6d M;
      M[0] = aF(0);
      M[1] = m_E.m_c*aF(1) - m_E.m_s*aF(2);
      M[2] = m_E.m_s*aF(1) + m_E.m_c*aF(2);
  
      M[0] += (-m_E.m_c*m_r(2) + m_E.m_s*m_r(1)) * aF(4) + 
        (m_E.m_s*m_r(2) + m_E.m_c*m_r(1)) * aF(5) ;
  
      M[1] += m_r(2) * aF(3) 
        - m_E.m_s*m_r(0) * aF(4) 
        - m_E.m_c*m_r(0) * aF(5) ;

      M[2] += -m_r(1)* aF(3) + 
        m_E.m_c*m_r(0) * aF(4) 
        -m_E.m_s*m_r(0) * aF(5);
  
      M[3] = aF(3) ;
      M[4] = m_E.m_c*aF(4) - m_E.m_s*aF(5);
      M[5] = m_E.m_s*aF(4) + m_E.m_c*aF(5);
  
      return M;
    }

    template <> METAPOD_DLLEXPORT
    Vector6d TransformT<RotationMatrix>::mulMatrixTransposeBy(Vector6d &aF) const 
    {
      Vector6d M;
  
      M[0] = m_E(0,0)*aF(0) + m_E(1,0)*aF(1) + m_E(2,0)*aF(2);
      M[1] = m_E(0,1)*aF(0) + m_E(1,1)*aF(1) + m_E(2,1)*aF(2);
      M[2] = m_E(0,2)*aF(0) + m_E(1,2)*aF(1) + m_E(2,2)*aF(2);
	  
      M[0] += (-m_E(0,1)*m_r(2) + m_E(0,2)*m_r(1))* aF(3) + 
        (-m_E(1,1)*m_r(2) + m_E(1,2)*m_r(1)) * aF(4) + 
        (-m_E(2,1)*m_r(2) + m_E(2,2)*m_r(1)) * aF(5) ;

      M[1] += ( m_E(0,0)*m_r(2) - m_E(0,2)*m_r(0))* aF(3) + 
        ( m_E(1,0)*m_r(2) - m_E(1,2)*m_r(0)) * aF(4) + 
        ( m_E(2,0)*m_r(2) - m_E(2,2)*m_r(0)) * aF(5) ;

      M[2] += (-m_E(0,0)*m_r(1) + m_E(0,1)*m_r(0))* aF(3) + 
        (-m_E(1,0)*m_r(1) + m_E(1,1)*m_r(0)) * aF(4) + 
        (-m_E(2,0)*m_r(1) + m_E(2,1)*m_r(0)) * aF(5);
	  
      M[3] = m_E(0,0)*aF(3) + m_E(1,0)*aF(4) + m_E(2,0)*aF(5);
      M[4] = m_E(0,1)*aF(3) + m_E(1,1)*aF(4) + m_E(2,1)*aF(5);
      M[5] = m_E(0,2)*aF(3) + m_E(1,2)*aF(4) + m_E(2,2)*aF(5);
	  
      return M;
    }

  }
}
