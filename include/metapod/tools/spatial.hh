// Copyright 2012,
//
// Maxime Reis
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
// along with metapod.  If not, see <http://www.gnu.org/licenses/>.

/*
 * Implementation of a spatial algebra.
 */

#ifndef METAPOD_NEW_SPATIAL_ALGEBRA_HH
# define METAPOD_NEW_SPATIAL_ALGEBRA_HH

# include "metapod/tools/fwd.hh"
# include "metapod/tools/smallmatrixmacros.hh"

namespace metapod
{

  namespace Spatial
  {

    // Tool methods
    inline matrix3d skew(const vector3d & v)
    {
      matrix3d m;
      m <<   0 , -v(2),  v(1),
           v(2),    0 , -v(0),
          -v(1),  v(0),    0 ;
      return m;
    }

    class Force
    {
      public:
        // Constructors
        Force() {}
        Force(vector3d n, vector3d f)
        {
          m_n = n;
          m_f = f;
        }
        Force(vector6d v)
        {
          m_n = v.segment<3>(0);
          m_f = v.segment<3>(3);
        }

        // Getters
        const vector3d & n() const { return m_n; }
        const vector3d & f() const { return m_f; }

        // Arithmetic operators
        Force operator=(vector6d & v) { return Force(v); }
        Force operator+(Force & fv) { return Force(m_n+fv.n(), m_f+fv.f()); }
        Force operator+(const Force & fv) { return Force(m_n+fv.n(), m_f+fv.f()); }
        Force operator-(Force & fv) { return Force(m_n-fv.n(), m_f-fv.f()); }
        Force operator*(FloatType a) { return Force(m_n*a, m_f*a); }

        friend Force operator*(FloatType a, Force & fv) { return Force(fv.n()*a, fv.f()*a); }

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
        vector3d m_n;
        vector3d m_f;
    };

    class Motion
    {
      public:
        // Constructors
        Motion() {}
        Motion(vector3d w, vector3d v)
        {
          m_w = w;
          m_v = v;
        }
        Motion(vector6d v)
        {
          m_w = v.segment<3>(0);
          m_v = v.segment<3>(3);
        }

        // Getters
        const vector3d & w() const { return m_w; } 
        const vector3d & v() const { return m_v; } 

        // Setters
        void w(const vector3d & v) { m_w = v; }

        // Arithmetic operators
        Motion operator=(vector6d & v) { return Motion(v); }
        Motion operator+(Motion & mv) { return Motion(m_w+mv.w(), m_v+mv.v()); }
        Motion operator+(const Motion & mv) { return Motion(m_w+mv.w(), m_v+mv.v()); }
        Motion operator*(FloatType a) { return Motion(m_w*a, m_v*a); }

        Motion operator^(Motion & mv)
        {
          return Motion(m_w.cross(mv.w()),
                        m_w.cross(mv.v()) + m_v.cross(mv.w()));
        }

        Force operator^(Force & fv)
        {
          return Force(m_w.cross(fv.n()) + m_v.cross(fv.f()),
                       m_w.cross(fv.f()));
        }

        Force operator^(const Force & fv)
        {
          return Force(m_w.cross(fv.n()) + m_v.cross(fv.f()),
                       m_w.cross(fv.f()));
        }

        friend Motion operator*(FloatType a, Motion & mv)
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
        vector3d m_w;
        vector3d m_v;
    };

    class Inertia
    {
      public:
        // Constructors
        Inertia() {}
        Inertia(FloatType m, vector3d h, matrix3d I)
        {
          m_m = m;
          m_h = h;
          m_I = I;
          m_hx = skew(h);
        }

        // Getters
        FloatType m() const { return m_m; }
        const vector3d & h() const { return m_h; }
        const matrix3d & I() const { return m_I; }
        const matrix3d & hx() const { return m_hx; }

        // Arithmetic operators
        Inertia operator*(FloatType a) { return Inertia(m_m*a, m_h*a, m_I*a); }
        Inertia operator+(Inertia & I) { return Inertia(m_m+I.m(),
                                                        m_h+I.h(),
                                                        m_I+I.I()); }
        Force operator*(Motion & mv) { return Force(m_I*mv.w() + m_hx*mv.v(),
                                                    m_m*mv.v() - m_hx*mv.w()); }

        friend Inertia operator*(FloatType a, Inertia & I)
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

      private:
        // Private members
        FloatType m_m;
        vector3d m_h;
        matrix3d m_I;
        matrix3d m_hx;
    };

    class Transform
    {
      public:
        // Constructors
        Transform() {}
        Transform(matrix3d E, vector3d r)
        {
          m_E = E;
          m_r = r;
          m_ET = E.transpose();
          m_rx = skew(r);
        }

        // Getters
        const vector3d & r() const { return m_r; }
        const matrix3d & E() const { return m_E; }
        const matrix3d & rx() const { return m_rx; }
        const matrix3d & ET() const { return m_ET; }

        // Transformations
        Motion apply(const Motion & mv)
        {
          return Motion(m_E * mv.w(), m_E * (mv.v() - m_rx * mv.w()));
        }

        Force apply(const Force & fv)
        {
          return Force(m_E*(fv.n() - m_rx*fv.f()), m_E*fv.f());
        }

        Inertia apply(const Inertia & I)
        {
          vector3d tmp = I.h() - I.m()*m_r;
          return Inertia(I.m(),
                         m_E*tmp,
                         m_E*(I.I() + m_rx*I.hx() + skew(tmp)*m_rx)*m_ET);
        }

        Motion applyInv(const Motion & mv)
        {
          vector3d ET_w = m_ET*mv.w();
          return Motion(ET_w, m_ET*mv.v() + m_rx*ET_w);
        }

        Force applyInv(const Force & fv)
        {
          vector3d ET_f = m_ET*fv.f();
          return Force(m_ET*fv.n() + m_rx*ET_f, ET_f);
        }

        Inertia applyInv(const Inertia & I)
        {
          vector3d tmp1 = m_ET*I.h();
          vector3d tmp2 = tmp1 + I.m()*m_r;
          return Inertia(I.m(),
                         tmp2,
                         m_ET*I.I()*m_E - m_rx*skew(tmp1) - skew(tmp2)*m_rx);
        }

        Transform inverse() { return Transform(m_ET, -m_E*m_r); }

        // Arithmetic operators
        Transform operator*(Transform & X)
        {
          return Transform(m_E*X.E(), X.r() + X.ET()*m_r);
        }

        Transform operator*(const Transform & X)
        {
          return Transform(m_E*X.E(), X.r() + X.ET()*m_r);
        }

        Motion operator*(Motion & mv) { return apply(mv); }
        Force operator*(Force & fv) { return apply(fv); }
        Inertia operator*(Inertia & I) { return apply(I); }

        // Print operator
        friend std::ostream & operator << (std::ostream & os,
                                           const Transform & X)
        {
          os
            << "  E =\n" << X.E() << std::endl
            << "  r =\n" << X.r().transpose() << std::endl;
          return os;
        }

      private:
        // Private members
        vector3d m_r;
        matrix3d m_E;
        matrix3d m_rx;
        matrix3d m_ET;
    };

  } // end of namespace Spatial

} // end of namespace metapod

# endif
