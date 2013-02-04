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


#ifndef METAPOD_SPATIAL_ALGEBRA_MOTION_HH
# define METAPOD_SPATIAL_ALGEBRA_MOTION_HH

# include "metapod/tools/spatial/force.hh"

namespace metapod
{

  namespace Spatial
  {

    class Motion
    {
      public:
        // Constructors
        Motion() : m_w(), m_v() {}
        Motion(const Vector3d & w, const Vector3d & v) : m_w(w), m_v(v) {}
        Motion(const Vector6d & v) : m_w(v.segment<3>(0)),
                                     m_v(v.segment<3>(3)) {}

        static const Motion Zero()
        {
          return Motion (Vector3d::Zero(), Vector3d::Zero());
        }

        // Getters
        const Vector3d & w() const { return m_w; }
        const Vector3d & v() const { return m_v; }

        // Setters
        void w(const Vector3d & v) { m_w = v; }
        void v(const Vector3d & v) { m_v = v; }

        // Arithmetic operators
        Motion & operator=(const Vector6d & v)
        {
          m_w = v.segment<3>(0);
          m_v = v.segment<3>(3);
          return *this;
        }

        Motion operator-() const
        {
          return Motion(-m_w, -m_v);
        }

        Motion operator+(const Motion & mv) const
        {
          return Motion(m_w+mv.w(), m_v+mv.v());
        }

        Motion operator*(FloatType a) const
        {
          return Motion(m_w*a, m_v*a);
        }

        Motion operator^(const Motion & mv) const
        {
          return Motion(m_w.cross(mv.w()),
                        m_w.cross(mv.v()) + m_v.cross(mv.w()));
        }

        Force operator^(const Force & fv) const
        {
          return Force(m_w.cross(fv.n()) + m_v.cross(fv.f()),
                       m_w.cross(fv.f()));
        }

        friend Motion operator*(FloatType a, const Motion & mv)
        {
          return Motion(mv.w()*a, mv.v()*a);
        }

        // Print operator
        friend std::ostream & operator << (std::ostream & os, const Motion & mv)
        {
          os
            << "w =\n" << mv.w() << std::endl
            << "v =\n" << mv.v() << std::endl;
          return os;
        }

      private:
        // Private members
        Vector3d m_w;
        Vector3d m_v;
    };

  } // end of namespace Spatial

} // end of namespace metapod

# endif
