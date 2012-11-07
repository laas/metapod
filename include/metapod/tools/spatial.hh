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

/*
 * Implementation of a spatial algebra.
 * It follows R. Featherstone guidelines, and implements :
 * - Spatial Motion vectors
 * - Spatial Force vectors
 * - Spatial Rigid Body Inertia matrices
 * - Spatial Transforms
 */

#ifndef METAPOD_SPATIAL_ALGEBRA_HH
# define METAPOD_SPATIAL_ALGEBRA_HH

# include "metapod/tools/fwd.hh"

namespace metapod
{

  namespace Spatial
  {

    // Tool methods
    inline matrix3d skew(const vector3d & v)
    {
      matrix3d m;
      m(0,0) = 0;    m(0,1) = -v(2); m(0,2) = v(1);
      m(1,0) = v(2); m(1,1) = 0    ; m(1,2) = -v(0);
      m(2,0) = -v(1);m(2,1)=  v(0) ; m(2,2) =  0 ;
      return m;
    }

    class Force
    {
      public:
        // Constructors
        Force() : m_n(), m_f() {}
        Force(const vector3d & n, const vector3d & f) : m_n(n), m_f(f) {}
        Force(const vector6d & v) : m_n(v.segment<3>(0)),
                                    m_f(v.segment<3>(3)) {}

        // Getters
        const vector3d & n() const { return m_n; }
        const vector3d & f() const { return m_f; }

        vector6d toVector() const
        {
          vector6d v;
          for(unsigned int i=0; i<3; i++)
            v[i] = m_n[i];
          for(unsigned int i=0; i<3; i++)
            v[i+3] = m_f[i];
          return v;
        }

        // Arithmetic operators
        Force & operator=(const vector6d & v)
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
        vector3d m_n;
        vector3d m_f;
    };

    class Motion
    {
      public:
        // Constructors
        Motion() : m_w(), m_v() {}
        Motion(const vector3d & w, const vector3d & v) : m_w(w), m_v(v) {}
        Motion(const vector6d & v) : m_w(v.segment<3>(0)),
                                     m_v(v.segment<3>(3)) {}

        // Getters
        const vector3d & w() const { return m_w; }
        const vector3d & v() const { return m_v; }

        // Setters
        void w(const vector3d & v) { m_w = v; }
        void v(const vector3d & v) { m_v = v; }

        // Arithmetic operators
        Motion & operator=(const vector6d & v)
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
        vector3d m_w;
        vector3d m_v;
    };

    class Inertia
    {
      public:
        // Constructors
        Inertia() : m_m(), m_h(), m_I() {}
        Inertia(FloatType m, const vector3d & h, const matrix3d & I)
          : m_m(m), m_h(h), m_I(I) {}

        // Getters
        FloatType m() const { return m_m; }
        const vector3d & h() const { return m_h; }
        const matrix3d & I() const { return m_I; }

        matrix6d toMatrix() const
        {
          matrix6d M;
          M.block<3,3>(0,0) = m_I;
          M.block<3,3>(0,3) = skew(m_h);
          M.block<3,3>(3,0) = -skew(m_h);
          M.block<3,3>(3,3) = m_m*matrix3d::Identity();
          return M;
        }

        // Arithmetic operators
        Inertia operator*(FloatType a) const
        {
          return Inertia(m_m*a, m_h*a, m_I*a);
        }

        Inertia operator+(const Inertia & I) const
        {
          return Inertia(m_m+I.m(), m_h+I.h(), m_I+I.I());
        }

        Force operator*(const Motion & mv) const
        {
          return Force(m_I*mv.w() + m_h.cross(mv.v()),
                       m_m*mv.v() - m_h.cross(mv.w()));
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

      private:
        // Private members
        FloatType m_m;
        vector3d m_h;
        matrix3d m_I;
    };

    class Transform
    {
      public:
        // Constructors
        Transform() : m_E(), m_r() {}
        Transform(const matrix3d & E, const vector3d & r) : m_E(E), m_r(r) {}

        // Getters
        const vector3d & r() const { return m_r; }
        const matrix3d & E() const { return m_E; }

        // Transformations
        Motion apply(const Motion & mv) const
        {
          return Motion(m_E * mv.w(),
                        m_E * (mv.v() - m_r.cross(mv.w())));
        }

        Force apply(const Force & fv) const
        {
          return Force(m_E*(fv.n() - m_r.cross(fv.f())), m_E*fv.f());
        }

        Inertia apply(const Inertia & I) const
        {
          vector3d tmp = I.h() - I.m()*m_r;
          return Inertia(I.m(),
                         m_E*tmp,
                         m_E*(I.I()
                         + skew(m_r)*skew(I.h())
                         + skew(tmp)*skew(m_r))*m_E.transpose());
        }

        vector3d apply(const vector3d& p) const
        {
          return m_E*(p - m_r);
        }

        matrix6d toMatrix() const
        {
          matrix6d M;
          M.block<3,3>(0,0) = m_E;
          M.block<3,3>(0,3) = matrix3d::Zero();
          M.block<3,3>(3,0) = -m_E * skew(m_r);
          M.block<3,3>(3,3) = m_E;
          return M;
        }

        matrix6d toMatrixTranspose() const
        {
          matrix6d M;
          M.block<3,3>(0,0) = m_E.transpose();
          M.block<3,3>(3,0) = matrix3d::Zero();
          M.block<3,3>(0,3) = (-m_E * skew(m_r)).transpose();
          M.block<3,3>(3,3) = m_E.transpose();
          return M;
        }

        matrix6d toMatrixTransposeInverse() const
        {
          matrix6d M;
          M.block<3,3>(0,0) = m_E;
          M.block<3,3>(0,3) = -m_E * skew(m_r);
          M.block<3,3>(3,0) = matrix3d::Zero();
          M.block<3,3>(3,3) = m_E;
          return M;
        }

        Motion applyInv(const Motion & mv) const
        {
          vector3d ET_w = m_E.transpose()*mv.w();
          return Motion(ET_w, m_E.transpose()*mv.v() + m_r.cross(ET_w));
        }

        Force applyInv(const Force & fv) const
        {
          vector3d ET_f = m_E.transpose()*fv.f();
          return Force(m_E.transpose()*fv.n() + m_r.cross(ET_f), ET_f);
        }

        Inertia applyInv(const Inertia & I) const
        {
          vector3d tmp1 = m_E.transpose()*I.h();
          vector3d tmp2 = tmp1 + I.m()*m_r;
          return Inertia(I.m(),
                         tmp2,
                         m_E.transpose()*I.I()*m_E
                         - skew(m_r)*skew(tmp1)
                         - skew(tmp2)*skew(m_r));
        }

        vector3d applyInv(const vector3d& p) const
        {
          return m_E.transpose()*p + m_r;
        }

        Transform inverse() const
        {
          return Transform(m_E.transpose(), -m_E*m_r);
        }

        // Arithmetic operators
        Transform operator*(FloatType a) const
        {
          return Transform(a*m_E, a*m_r);
        }

        Transform operator*(const Transform & X) const
        {
          return Transform(m_E*X.E(), X.r() + X.E().transpose()*m_r);
        }

        // Specialization of transform multiplication.Compute
        // transform pXA from BXA, with p vector expressed in B
        // coordinates: pXA = pXB*BXA
        Transform toPointFrame(const vector3d& p) const
        {
          return Transform (m_E, m_r + m_E.transpose()*p);
        }

        Motion operator*(const Motion & mv) const
        {
          return Motion(m_E * mv.w(), m_E * (mv.v() - m_r.cross(mv.w())));
        }

        Force operator*(const Force & fv) const
        {
          vector3d lf = fv.f();
          return Force(m_E*(fv.n() - m_r.cross(lf)), m_E*lf);
        }

        Inertia operator*(const Inertia & I) const
        {
          vector3d tmp = I.h() - I.m()*m_r;
          return Inertia(I.m(),
                         m_E*tmp,
                         m_E*(I.I()
                         + skew(m_r)*skew(I.h())
                         + skew(tmp)*skew(m_r))*m_E.transpose());
        }

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
        matrix3d m_E;
        vector3d m_r;
    };

    inline Motion sum(const Motion & mv1,
               const Motion & mv2,
               const Motion & mv3,
               const Motion & mv4)
    {
      return Motion(mv1.w() + mv2.w() + mv3.w() + mv4.w(),
                    mv1.v() + mv2.v() + mv3.v() + mv4.v());
    }

    inline Motion sum(const Motion & mv1,
               const Motion & mv2,
               const Motion & mv3)
    {
      return Motion(mv1.w() + mv2.w() + mv3.w(),
                    mv1.v() + mv2.v() + mv3.v());
    }

    inline Force sum(const Force & fv1,
              const Force & fv2,
              const Force & fv3)
    {
      return Force(fv1.n() + fv2.n() + fv3.n(),
                   fv1.f() + fv2.f() + fv3.f());
    }

  } // end of namespace Spatial

} // end of namespace metapod

# endif
