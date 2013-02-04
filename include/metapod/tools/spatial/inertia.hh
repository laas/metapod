// Copyright 2012,
//
// Maxime Reis (JRL/LAAS, CNRS/AIST)
// Antonio El Khoury (JRL/LAAS, CNRS/AIST)
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

#ifndef METAPOD_SPATIAL_ALGEBRA_INERTIA_HH
# define METAPOD_SPATIAL_ALGEBRA_INERTIA_HH

# include "metapod/tools/fwd.hh"
# include "metapod/tools/spatial/lti.hh"

namespace metapod
{
  namespace Spatial
  {
    class Inertia
    {
      public:
        // Constructors
        Inertia() : m_m(), m_h(), m_I() {}
        Inertia(FloatType m, const Vector3d & h, const Matrix3d & I)
          : m_m(m), m_h(h), m_I(I) {}
        Inertia(FloatType m, const Vector3d & h, const lowerTriangularMatrix & I)
          : m_m(m), m_h(h), m_I(I) {}

        // Initializers
        static const Inertia Zero() { return Inertia(0., Vector3d::Zero(), lowerTriangularMatrix::Zero()); }

        // Getters
        FloatType m() const { return m_m; }
        const Vector3d & h() const { return m_h; }
        const lowerTriangularMatrix & I() const { return m_I; }

        Matrix6d toMatrix() const
        {
          Matrix6d M;
          M.block<3,3>(0,0) = m_I.toMatrix();
          M.block<3,3>(0,3) = skew(m_h);
          M.block<3,3>(3,0) = -skew(m_h);
          M.block<3,3>(3,3) = m_m*Matrix3d::Identity();
          return M;
        }

        // Arithmetic operators

        Inertia operator+(const Inertia & other) const
        {
          return Inertia(m_m+other.m(), m_h+other.h(), m_I+other.I());
        }


        friend Inertia operator*(FloatType a, const Inertia & I)
        {
          return Inertia(I.m()*a, I.h()*a, I.I()*a);
        }

        // Print operator
        friend std::ostream & operator << (std::ostream & os, const Inertia & I)
        {
          os
            << "m =\n" << I.m() << std::endl
            << "h =\n" << I.h() << std::endl
            << "I =\n" << I.I() << std::endl;
          return os;
        }
      
      template<class T, class U, class S>
      friend class OperatorMul;


      template<class T, class U, class S>
      friend T operator*(const U &u,const S & s) 
      { return OperatorMul<T,U,S>::mul(u,s); }

      
     
      private:
        // Private members
        FloatType m_m;
        Vector3d m_h;
        lowerTriangularMatrix m_I;
    };

    template<> inline
    Inertia OperatorMul<Inertia,Inertia,FloatType>::mul(const Inertia & m,
							const FloatType &a) const
    {
      return Inertia(m.m_m*a, m.m_h*a, m.m_I*a);
    }
	
    inline Inertia operator*(const Inertia & m,
		      const FloatType &a) 
    {
      OperatorMul<Inertia,Inertia,FloatType> om;
      return om.mul(m,a);
    }
	
    /* Operator Force = Inertia * Motion */
    template<> inline
    Force OperatorMul<Force,Inertia,Motion>::mul(const Inertia & m,
						 const Motion &mv) const
    {
	  return Force(m.m_I*mv.w() + m.m_h.cross(mv.v()),
		       m.m_m*mv.v() - m.m_h.cross(mv.w()));
    }
    
    inline Force operator*(const Inertia &m,
                           const Motion & mv) 
    {
      OperatorMul<Force,Inertia,Motion> om;
      return om.mul(m,mv);
    }
    
  } // end of namespace Spatial

} // end of namespace metapod

# endif
