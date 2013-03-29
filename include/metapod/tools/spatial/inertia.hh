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
        Inertia(FloatType m, const Vector3d &h, const Matrix3d &I)
            : m_m(m),
              m_h(h),
              m_I(I) {}
        Inertia(FloatType m, const Vector3d &h, const lowerTriangularMatrix &I)
            : m_m(m),
              m_h(h),
              m_I(I) {}

        // Initializers
        static const Inertia Zero() {
          return Inertia(0., Vector3d::Zero(), lowerTriangularMatrix::Zero());
        }

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
        Inertia operator+(const Inertia &other) const {
          return Inertia(m_m+other.m_m, m_h+other.m_h, m_I+other.m_I);
        }

        Inertia operator*(const FloatType &a) const {
          return Inertia(m_m*a, m_h*a, m_I*a);
        }

        Force operator*(const Motion &mv) const {
          return Force(m_I*mv.w() + m_h.cross(mv.v()),
                       m_m*mv.v() - m_h.cross(mv.w()));
        }

      private:
        // Private members
        FloatType m_m;
        Vector3d m_h;
        lowerTriangularMatrix m_I;
    };

    inline std::ostream & operator<< (std::ostream &os, const Inertia &I) {
      os << "m =\n" << I.m() << "\n"
         << "h =\n" << I.h() << "\n"
         << "I =\n" << I.I();
      return os;
    }

    inline Inertia operator*(FloatType a, const Inertia &I)
    {
      return I * a;
    }

    
  } // end of namespace Spatial

} // end of namespace metapod

# endif
