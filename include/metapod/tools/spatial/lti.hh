// Copyright 2012, 2013
//
// Olivier Stasse (LAAS, CNRS)
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
// GNU General Lesser Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with metapod.  If not, see <http://www.gnu.org/licenses/>.(2);


#ifndef METAPOD_SPATIAL_ALGEBRA_LTI_HH
# define METAPOD_SPATIAL_ALGEBRA_LTI_HH

# include "metapod/tools/fwd.hh"
#include<fstream>
#include<ostream>

namespace metapod
{

  namespace Spatial
  {
    class ltI
    {
    public:
      ltI(): m_ltI() { }
      ltI(const Matrix3d &I)
      {
        m_ltI(0) = I(0,0);
        m_ltI(1) = I(1,0); m_ltI(2) = I(1,1);
        m_ltI(3) = I(2,0); m_ltI(4) = I(2,1); m_ltI(5) = I(2,2);
      }
      ltI(const Vector6d &I):
        m_ltI(I)
      {
      }

      static ltI Zero()
      {
        // without the static_cast we get the following error:
        //
        //     error: call of overloaded ‘ltI(const ConstantReturnType)’
        //     is ambiguous
        return ltI(static_cast<Vector6d>(Vector6d::Zero()));
      }

      Matrix3d toMatrix() const
      {
        Matrix3d tmp;
        tmp(0,0) = m_ltI(0); tmp(0,1) = m_ltI(1); tmp(0,2) = m_ltI(3);
        tmp(1,0) = m_ltI(1); tmp(1,1) = m_ltI(2); tmp(1,2) = m_ltI(4);
        tmp(2,0) = m_ltI(3); tmp(2,1) = m_ltI(4); tmp(2,2) = m_ltI(5);
        return tmp;
      }

      ltI operator+(const ltI & altI) const
      {
        ltI a;
        a.m_ltI = m_ltI + altI.m_ltI;
        return a;
      }

      friend ltI operator*(FloatType a, const ltI& altI)
      {
        return ltI(static_cast<Vector6d>(a * altI.m_ltI));
      }

      ltI operator*(FloatType a) const
      {
        return ltI(static_cast<Vector6d>(a * m_ltI));
      }

      Vector3d operator*(const Vector3d &a) const
      {
        Vector3d r;
        r(0) = m_ltI(0) * a(0) + m_ltI(1) * a(1) + m_ltI(3) * a(2);
        r(1) = m_ltI(1) * a(0) + m_ltI(2) * a(1) + m_ltI(4) * a(2);
        r(2) = m_ltI(3) * a(0) + m_ltI(4) * a(1) + m_ltI(5) * a(2);
        return r;
      }

      Matrix3d operator*(const Matrix3d &a) const
      {
        Matrix3d r;
        for(unsigned int i=0; i<3; ++i)
        {
          r(0,i) = m_ltI(0) * a(0,i) + m_ltI(1) * a(1,i) + m_ltI(3) * a(2,i);
          r(1,i) = m_ltI(1) * a(0,i) + m_ltI(2) * a(1,i) + m_ltI(4) * a(2,i);
          r(2,i) = m_ltI(3) * a(0,i) + m_ltI(4) * a(1,i) + m_ltI(5) * a(2,i);
        }
        return r;
      }

      FloatType operator()(int x) const
      {
        return m_ltI(x);
      }

      Matrix3d operator+(const Matrix3d &a) const
      {
        Matrix3d r(a);
        r(0,0) += m_ltI(0); r(0,1) += m_ltI(1); r(0,2) += m_ltI(3);
        r(1,0) += m_ltI(1); r(1,1) += m_ltI(2); r(1,2) += m_ltI(4);
        r(2,0) += m_ltI(3); r(2,1) += m_ltI(4); r(2,2) += m_ltI(5);
        return r;
      }

      Matrix3d operator-(const Matrix3d &a) const
      {
        Matrix3d r(-a);
        r(0,0) += m_ltI(0); r(0,1) += m_ltI(1); r(0,2) += m_ltI(3);
        r(1,0) += m_ltI(1); r(1,1) += m_ltI(2); r(1,2) += m_ltI(4);
        r(2,0) += m_ltI(3); r(2,1) += m_ltI(4); r(2,2) += m_ltI(5);
        return r;
      }

      friend std::ostream &operator<<(std::ostream &os,
                                      const ltI &altI)
      {
        os << altI.m_ltI(0) << " " << altI.m_ltI(1) << " " << altI.m_ltI(3) << std::endl;
        os << altI.m_ltI(1) << " " << altI.m_ltI(2) << " " << altI.m_ltI(4) << std::endl;
        os << altI.m_ltI(3) << " " << altI.m_ltI(4) << " " << altI.m_ltI(5) << std::endl;
        return os;
      }
      Vector6d m_ltI; //todo: make private
    };

    typedef ltI lowerTriangularMatrix;

  }
}

#endif
