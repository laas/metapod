// Copyright 2012, 2013
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

#ifndef METAPOD_SPATIAL_ALGEBRA_FORCE_HH
# define METAPOD_SPATIAL_ALGEBRA_FORCE_HH

namespace metapod
{
  namespace Spatial
  {
    class Force
    {
      public:
        // Constructors
        Force() : m_n(), m_f() {}
        Force(const Vector3d & n, const Vector3d & f) : m_n(n), m_f(f) {}
        Force(const Vector6d & v) : m_n(v.segment<3>(0)),
                                    m_f(v.segment<3>(3)) {}

        // initializers
        static const Force Zero() { return Force(Vector3d::Zero(), Vector3d::Zero()); }

        // Getters
        const Vector3d & n() const { return m_n; }
        const Vector3d & f() const { return m_f; }

        Vector6d toVector() const
        {
          Vector6d v;
          for(unsigned int i=0; i<3; i++)
            v[i] = m_n[i];
          for(unsigned int i=0; i<3; i++)
            v[i+3] = m_f[i];
          return v;
        }

        // Arithmetic operators
        Force & operator=(const Vector6d & v)
        {
          m_n = v.segment<3>(0);
          m_f = v.segment<3>(3);
          return *this;
        }

        Force operator+(const Force & fv) const
        {
          return Force(m_n+fv.n(), m_f+fv.f());
        }

        Force operator-() const
        {
          return Force(-m_n, -m_f);
        }

        Force operator-(const Force & fv) const
        {
          return Force(m_n-fv.n(), m_f-fv.f());
        }

        Force operator*(FloatType a) const
        {
          return Force(m_n*a, m_f*a);
        }

        friend Force operator*(FloatType a, const Force & fv)
        {
          return Force(fv.n()*a, fv.f()*a);
        }

        // Print operator
        friend std::ostream & operator << (std::ostream & os, const Force & fv)
        {
          os
            << "n =\n" << fv.n() << std::endl
            << "f =\n" << fv.f() << std::endl;
          return os;
        }

      private:
        // Private members
        Vector3d m_n;
        Vector3d m_f;
    };
  } // end of namespace Spatial

} // end of namespace metapod

# endif

