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
    template <class FloatType>
    class MotionTpl
    {
      EIGEN_METAPOD_TYPEDEFS;
      EIGEN_METAPOD_SPATIAL_FORCE_TYPEDEF;
      public:
        // Constructors
        MotionTpl() : m_w(), m_v() {}
        MotionTpl(const Vector3d & w, const Vector3d & v) : m_w(w), m_v(v) {}
        MotionTpl(const Vector6d & v) : 
          m_w(v.template segment<3>(0)),
          m_v(v.template segment<3>(3)) {}

        static const MotionTpl Zero()
        {
          return MotionTpl(Vector3d::Zero(), 
                        Vector3d::Zero());
        }

        // Getters
        const Vector3d & w() const { return m_w; }
        const Vector3d & v() const { return m_v; }

        // Setters
        void w(const Vector3d & v) { m_w = v; }
        void v(const Vector3d & v) { m_v = v; }

        // Arithmetic operators
        MotionTpl & operator=(const Vector6d & v)
        {
          m_w = v.segment<3>(0);
          m_v = v.segment<3>(3);
          return *this;
        }

        MotionTpl operator-() const
        {
          return MotionTpl(-m_w, -m_v);
        }

        MotionTpl operator+(const MotionTpl & mv) const
        {
          return MotionTpl(m_w+mv.w(), m_v+mv.v());
        }

        MotionTpl operator*(FloatType a) const
        {
          return MotionTpl(m_w*a, m_v*a);
        }

        MotionTpl operator^(const MotionTpl & mv) const
        {
          return MotionTpl(m_w.cross(mv.w()),
                        m_w.cross(mv.v()) + m_v.cross(mv.w()));
        }

      Force operator^(const Force & fv) const
         {
           return Force(m_w.cross(fv.n()) + m_v.cross(fv.f()),
                                   m_w.cross(fv.f()));
         }

         friend MotionTpl operator*(FloatType a, const MotionTpl & mv)
         {
           return MotionTpl(mv.w()*a, mv.v()*a);
         }

         // Print operator
         friend std::ostream & operator << (std::ostream & os, const MotionTpl & mv)
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
 #define EIGEN_METAPOD_SPATIAL_MOTION_TYPEDEF \
  typedef Spatial::MotionTpl<FloatType> Motion

} // end of namespace metapod

# endif
