// Copyright 2010, 2011, 2012,
//
// Olivier Stasse,
// Layale Saab,
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

#ifndef metapod_SPATIAL_ALGEBRA_HH
# define metapod_SPATIAL_ALGEBRA_HH

# include "metapod/tools/fwd.hh"
# include "metapod/tools/smallmatrixmacros.hh"

namespace metapod
{

  namespace Spatial
  {
  
      class Velocity;
      class Acceleration;
      class cAcceleration;
      class Inertia;
      class Momentum;
      class Motion;
      class Force;
      class PluckerTransform;
      class PluckerTransformTranspose;
  
      class Velocity
      {
      public:
        Velocity();
        Velocity(vector3d lv0,vector3d lw);
        Velocity(vector6d v);
        Velocity operator+(Velocity &a);
        Velocity operator+(vector6d &a);
        Velocity operator-(Velocity &a);
        Velocity operator*(double ad);
        Velocity& operator=(const vector6d &a);
        vector6d  operator^(vector6d &a);
        Force operator^(Momentum &a);
  
        friend Velocity operator*(double ad, Velocity &a);
        friend Velocity operator+(vector6d & ,Velocity &);
  
        inline const vector3d & w() const     { return m_w;};
        inline const vector3d & v0() const      { return m_v0;}
  
        void v0(const vector3d &lv0)      { m_v0 = lv0;}
        void w(const vector3d& lw)      { m_w = lw;};
  
      friend std::ostream & operator<< (std::ostream & os, const Velocity & v)
      {
        os << "  lv0"   << std::endl
           << v.m_v0[0] << std::endl
           << v.m_v0[1] << std::endl
           << v.m_v0[2] << std::endl
           << "  lw"    << std::endl
           << v.m_w[0]  << std::endl
           << v.m_w[1]  << std::endl
           << v.m_w[2]  << std::endl;
        return os;
      }
  
      private:
        vector3d m_v0,m_w;
      };
  
      Velocity operator*(double ad, Velocity &a);
      Velocity operator+(vector6d &a, Velocity &b);
  
      class Acceleration
      {
      public:
        Acceleration();
        Acceleration(vector3d ldv0, vector3d ldw);
        Acceleration operator+(Acceleration &a);
        Acceleration operator-(Acceleration &a);
        Acceleration operator+(vector6d &a);
        Acceleration& operator=(vector6d &a);
  
        friend Acceleration operator+(vector6d & ,Acceleration &);
        vector3d dv0()
        { return m_dv0;}
        void dv0(const vector3d &lv0)
        { m_dv0 = lv0;}
  
        vector3d dw()
        { return m_dw;};
        void dw(const vector3d &ldw)
        { m_dw = ldw; };
  
      friend std::ostream & operator<< (std::ostream & os, const Acceleration & a)
      {
        os << "  ldv0"   << std::endl
           << a.m_dv0[0] << std::endl
           << a.m_dv0[1] << std::endl
           << a.m_dv0[2] << std::endl
           << "  ldw"    << std::endl
           << a.m_dw[0]  << std::endl
           << a.m_dw[1]  << std::endl
           << a.m_dw[2]  << std::endl;
        return os;
      }
  
      private:
        vector3d m_dv0, m_dw;
      };
  
      Acceleration operator+(vector6d &, Acceleration &);
  
      class cAcceleration
      {
      public:
        cAcceleration();
        cAcceleration(vector3d ldv0, vector3d ldw);
        cAcceleration operator+(cAcceleration &a);
        cAcceleration operator-(cAcceleration &a);
  
        vector3d dv0()
        { return m_dv0;}
        vector3d dw()
        { return m_dw;};
  
      private:
        vector3d m_dv0,m_dw;
      };
  
      class Force
      {
      public:
        Force();
        Force(vector3d lf, vector3d ln0);
        Force operator+(Force &a);
        Force operator-(Force &a);
        Force& operator=(vector6d &a);
  
        Force operator*(double ad);
        const vector3d& f() const      {return m_f;};
        const vector3d& n0() const     {return m_n0;};
        void f(vector3d &lf)      {m_f = lf;}
        void n0(vector3d &ln0)
        {m_n0 = ln0;}
  
      friend std::ostream & operator<< (std::ostream & os, const Force & f)
      {
        os << "  lf"   << std::endl
           << f.m_f[0] << std::endl
           << f.m_f[1] << std::endl
           << f.m_f[2] << std::endl
           << "  ln0"    << std::endl
           << f.m_n0[0]  << std::endl
           << f.m_n0[1]  << std::endl
           << f.m_n0[2]  << std::endl;
        return os;
      }
  
  
      private:
  
        vector3d m_f, m_n0;
      };
  
      class Motion
      {
      public:
        Motion();
        Motion(vector3d lp, vector3d ltheta);
        Motion operator+(Motion &a);
        Motion operator-(Motion &a);
  
        vector3d p()
        { return m_p; }
        vector3d theta()
        { return m_theta; }
  
      private:
        vector3d m_p, m_theta;
      };
  
      class Momentum
      {
      public:
        Momentum();
        Momentum(vector3d v, vector3d w);
  
        vector3d v()
        { return m_v;}
        vector3d w()
        { return m_w;}
        void v(vector3d &lv)
        { m_v = lv;}
        void w(vector3d &lw)
        { m_w = lw;}
  
      private:
        vector3d m_v, m_w;
      };
  
      class Inertia
      {
      public:
        Inertia();
        Inertia(matrix3d lI,   vector3d lh, double lm);
        void addInertia(Inertia &c,
                        Inertia &a,
                        Inertia &b) const;
  
        Inertia operator+(Inertia &a);
        Momentum operator*(Velocity &);
        Force operator*(Acceleration &);
  
        matrix3d  I()
        { return m_I;};
        vector3d  h()
        { return m_h;};
        double m()
        { return m_m;};
  
      friend std::ostream & operator<< (std::ostream & os, const Inertia & I)
      {
        os << "  lI"    << std::endl
           << I.m_I(0,0) << " " << I.m_I(0,1) << " " << I.m_I(0,2) << std::endl
           << I.m_I(1,0) << " " << I.m_I(1,1) << " " << I.m_I(1,2) << std::endl
           << I.m_I(2,0) << " " << I.m_I(2,1) << " " << I.m_I(2,2) << std::endl
           << "  lh"    << std::endl
           << I.m_h[0]  << std::endl
           << I.m_h[1]  << std::endl
           << I.m_h[2]  << std::endl
           << "  lm"    << std::endl
           << I.m_m << std::endl;
        return os;
      }
  
       private:
        matrix3d m_I;
        vector3d m_h;
        double m_m;
      };
  
      class PluckerTransform
      {
      public:
        PluckerTransform();
        PluckerTransform(matrix3d lR, vector3d lp) ;
        PluckerTransform operator*(PluckerTransform &a);
        Velocity operator*(Velocity &a);
        Acceleration operator*(Acceleration &a);
        Force operator*(Force &f);
        void inverse(PluckerTransform &a);
  
        friend class PluckerTransformTranspose;
        PluckerTransform& operator=( const PluckerTransform &a);
        PluckerTransform(PluckerTransformTranspose &X);
  
        const matrix3d & R() const
        {return m_R;}
        const vector3d & p() const
        {return m_p;}
  
      friend std::ostream & operator<< (std::ostream & os, const PluckerTransform & pt)
      {
        os << "  R =\n" << pt.m_R(0,0) << " " << pt.m_R(0,1) << " " << pt.m_R(0,2) << std::endl
                        << pt.m_R(1,0) << " " << pt.m_R(1,1) << " " << pt.m_R(1,2) << std::endl
                        << pt.m_R(2,0) << " " << pt.m_R(2,1) << " " << pt.m_R(2,2) << std::endl
           << "  p =\n" << pt.m_p[0] << " " << pt.m_p[1] << " " << pt.m_p[2] << std::endl;
        return os;
      }
  
      private:
        matrix3d m_R;
        vector3d m_p;
  
      };
  
      class PluckerTransformTranspose
      {
      public:
        PluckerTransformTranspose();
        PluckerTransformTranspose(matrix3d lR, vector3d lp) ;
        PluckerTransformTranspose(PluckerTransform &X);
  
        friend class PluckerTransform;
  
        Velocity operator*(Velocity &a);
        Force operator*(Force &f);
  
        matrix3d R()
        {return m_R;}
        vector3d p()
        {return m_p;}
  
      private:
        matrix3d m_R;
        vector3d m_p;
  
      };

      Velocity::Velocity()
      {
        S3_VECTOR_FILL(m_v0,0.0);
        S3_VECTOR_FILL(m_w,0.0);
      }
  
      Velocity::Velocity(vector3d lv0,
  		       vector3d lw)
        : m_v0(lv0), m_w(lw)
      {
      }
  
      Velocity::Velocity(vector6d v)
      {
        for(unsigned int i=0; i<3; i++)
        {
          m_v0(i) = v(i);
          m_w(i) = v(i+3);
        }
      }
  
      Velocity Velocity::operator+(Velocity &a)
      {
        return Velocity(m_v0 + a.m_v0, m_w + a.m_w);
      }
  
      Velocity Velocity::operator-(Velocity &a)
      {
        return Velocity(m_v0 - a.m_v0, m_w - a.m_w);
      }
  
      Velocity Velocity::operator+(vector6d &a)
      {
        Velocity av;
        if (a.size()==6)
  	{
  	  for(unsigned int i=0;i<3;i++)
  	    av.m_v0(i) = m_v0(i) + a(i);
  
  	  for(unsigned int i=0;i<3;i++)
  	    av.m_w(i) = m_w(i) + a(i+3);
  	}
        return av;
      }
  
      //----------> Adding the operator = to associate 2 equivalent quantities by L.S
      Velocity& Velocity::operator=(const vector6d &a)
      {
        for(unsigned int i=0;i<3;i++)
        {
          m_v0(i) = a(i);
          m_w(i) = a(i+3);
        }
        return *this;
      }
  
      Velocity operator*(double ad, Velocity &a)
      {
        Velocity c;
        c = a * ad;
        return c;
      }
  
      Velocity operator+(vector6d &a, Velocity &b)
      {
        Velocity c;
        vector3d bv0 = b.v0();
        vector3d bw = b.w();
        vector3d cv0, cw;
        for(unsigned int i=0;i<3;i++)
        {
          cv0(i) = bv0(i) + a(i);
          cw(i) = bw(i) + a(i+3);
        }
        c.v0(cv0);
        c.w(cw);
        return c;
      }
  
      Acceleration::Acceleration()
      {
        S3_VECTOR_FILL(m_dv0,0.0);
        S3_VECTOR_FILL(m_dw,0.0);
      }
  
      Acceleration::Acceleration(vector3d ldv0,
  			       vector3d ldw)
        : m_dv0(ldv0), m_dw(ldw)
      {
      }
  
      Acceleration Acceleration::operator+(Acceleration &a)
      {
        return Acceleration(m_dv0 + a.m_dv0, m_dw + a.m_dw);
      }
  
      Acceleration Acceleration::operator-(Acceleration &a)
      {
        return Acceleration(m_dv0 - a.m_dv0, m_dw - a.m_dw);
      }
  
      Acceleration Acceleration::operator+(vector6d &a)
      {
        Acceleration av;
        if (a.size()==6)
  	{
  	  for(unsigned int i=0;i<3;i++)
  	    av.m_dv0(i) = m_dv0(i) + a(i);
  
  	  for(unsigned int i=0;i<3;i++)
  	    av.m_dw(i) = m_dw(i) + a(i+3);
  	}
        return av;
      }
  
      //----------> Adding the operator = to associate 2 equivalent quantities by L.S
      Acceleration& Acceleration::operator=(vector6d &a)
      {
        for(unsigned int i=0;i<3;i++)
        {
          m_dv0(i) = a(i);
          m_dw(i) = a(i+3);
        }
        return *this;
      }
  
      // Adding this function Acceleration = vector6d+Acceleration (defined in header but not developed in source code) by L.S
      Acceleration operator+(vector6d & a, Acceleration & b)
      {
        Acceleration c;
        vector3d bdv0 = b.dv0();
        vector3d bdw = b.dw();
        vector3d cdv0, cdw;
        for(unsigned int i=0;i<3;i++)
        {
          cdv0(i) = bdv0(i) + a(i);
          cdw(i) = bdw(i) + a(i+3);
        }
        c.dv0(cdv0);
        c.dw(cdw);
        return c;
      }
  
      cAcceleration::cAcceleration()
      {
        S3_VECTOR_FILL(m_dv0,0.0);
        S3_VECTOR_FILL(m_dw,0.0);
      }
  
      cAcceleration::cAcceleration(vector3d ldv0,
  				 vector3d ldw)
        : m_dv0(ldv0),m_dw(ldw)
      {
      }
  
      cAcceleration cAcceleration::operator+(cAcceleration &a)
      {
        return cAcceleration(m_dv0 + a.m_dv0, m_dw + a.m_dw);
      }
  
      cAcceleration cAcceleration::operator-(cAcceleration &a)
      {
        return cAcceleration(m_dv0 - a.m_dv0, m_dw - a.m_dw);
      }
  
      Force::Force()
      {
        S3_VECTOR_FILL(m_f,0.0);
        S3_VECTOR_FILL(m_n0,0.0);
      }
  
      Force::Force(vector3d lf, vector3d ln0)
        : m_f(lf), m_n0(ln0)
      {}
  
      Force Force::operator+(Force &a)
      {
        return Force(m_f + a.m_f, m_n0 + a.m_n0);
      }
      Force Force::operator-(Force &a)
      {
        return Force(m_f - a.m_f, m_n0 - a.m_n0);
      }
  
      //----------> Adding the operator = to associate 2 equivalent quantities by L.S
      Force& Force::operator=(vector6d &a)
      {
        for(unsigned int i=0;i<3;i++)
        {
          m_f(i) = a(i);
          m_n0(i) = a(i+3);
        }
        return *this;
      }
  
      Motion::Motion()
      {
        S3_VECTOR_FILL(m_p,0.0);
        S3_VECTOR_FILL(m_theta,0.0);
      }
  
      Motion::Motion(vector3d lp,
  		   vector3d ltheta):
        m_p(lp), m_theta(ltheta)
      {}
  
      Motion Motion::operator+(Motion &a)
      {
        return Motion(m_p + a.m_p, m_theta + a.m_theta);
      }
      Motion Motion::operator-(Motion &a)
      {
        return Motion(m_p - a.m_p, m_theta - a.m_theta);
      }
  
      Inertia::Inertia()
      {
        S3x3_MATRIX_SET_IDENTITY(m_I);
        S3_VECTOR_FILL(m_h,0.0);
        m_m = 0.0;
      }
  
      Inertia::Inertia(matrix3d lI,
  		     vector3d lh,
  		     double lm)
        : m_I(lI), m_h(lh),m_m(lm)
      {
      }
  
      void Inertia::addInertia(Inertia &c,
  			     Inertia &a,
  			     Inertia &b) const
      {
        c.m_m = a.m_m + b.m_m;
        c.m_h = a.m_h + b.m_h;
        c.m_I = a.m_I + b.m_I;
      }
  
      Inertia Inertia::operator+( Inertia &a)
      {
        Inertia c;
        c.m_m = a.m_m + m_m;
        c.m_h = a.m_h + m_h;
        c.m_I = a.m_I + m_I;
        return c;
      }
  
      Momentum::Momentum()
      {
        S3_VECTOR_FILL(m_v,0.0);
        S3_VECTOR_FILL(m_w,0.0);
      }
  
      /* Adding this constructor Momentum(vector3d , vector3d)(defined in header
       * but not developed in source code) by L.S. */
      Momentum::Momentum(vector3d m, vector3d w)
    : m_v(m), m_w(w)
      {
      }
  
      /* ----------> correct the formula by L.S(substraction in lf linear
       * expression instead of addition). */
      Momentum Inertia::operator*(Velocity &v)
      {
        Momentum c;
        vector3d NE_tmp,NE_tmp2;
        // Angular acceleration
        NE_tmp = m_I * v.w();
        S3_VECTOR_CROSS_PRODUCT(NE_tmp2, m_h, v.v0());
        vector3d lw = NE_tmp2 + NE_tmp;
        c.w(lw);
  
        // Linear acceleration
        NE_tmp = v.v0() * m_m;
        S3_VECTOR_CROSS_PRODUCT(NE_tmp2, m_h, v.w());
        vector3d lv = NE_tmp - NE_tmp2;
        c.v(lv);
  
        return c;
      }
  
      /* ----------> correct the formula by L.S(substraction in lf linear
       * expression instead of addition). */
      Force Inertia::operator*(Acceleration &a)
      {
        Force c;
        vector3d NE_tmp,NE_tmp2;
        // Angular acceleration
        NE_tmp = m_I * a.dw();
        S3_VECTOR_CROSS_PRODUCT(NE_tmp2, m_h, a.dv0());
        vector3d ln0 = NE_tmp2 + NE_tmp;
        c.n0(ln0);
  
        // Linear acceleration
        NE_tmp = a.dv0() * m_m;
        S3_VECTOR_CROSS_PRODUCT(NE_tmp2, m_h, a.dw());
        vector3d lf = NE_tmp - NE_tmp2;
        c.f(lf);
  
        return c;
      }
  
      Force Force::operator*(double ad)
      {
        Force c;
        c.m_f = m_f * ad;
        c.m_n0 = m_n0 * ad;
        return c;
      }
  
      Velocity Velocity::operator*(double ad)
      {
        Velocity c;
        c.m_v0 = m_v0 * ad;
        c.m_w = m_w * ad;
        return c;
      }
  
      vector6d Velocity::operator^(vector6d &a)
      {
        vector6d c;
  
        // w x m
        c(3) = -m_w(2)*a(4)    + m_w(1)*a(5);
        c(4) =  m_w(2)*a(3)    - m_w(0)*a(5);
        c(5) = -m_w(1)*a(3)    + m_w(0)*a(4);
        // v0 x m + w x m0
        c(0) = -m_v0(2)*a(4)   + m_v0(1)*a(5) - m_w(2)*a(1) + m_w(1)*a(2);
        c(1) =  m_v0(2)* a(3)  - m_v0(0)*a(5) + m_w(2)*a(0) - m_w(0)*a(2);
        c(2) = -m_v0(1)* a(3)  + m_v0(0)*a(4) - m_w(1)*a(0) + m_w(0)*a(1);
        return c;
      }
  
      Force Velocity::operator^(Momentum &a)
      {
        vector3d dn0,df;
        vector3d aw  = a.w();
        vector3d av0 = a.v();
  
        /* ----------> Formulation Rectification by L.S cf eq.(2.16) in Springer
                       Handbook Of Robotics. */
        // -S(w)'*m0
        df(0) = -m_w(2)*av0(1) + m_w(1)*av0(2);
        df(1) =  m_w(2)*av0(0) - m_w(0)*av0(2);
        df(2) = -m_w(1)*av0(0) + m_w(0)*av0(1);
  
        //-S(w)'*m -S(v0)'*m0
        dn0(0) = -m_w(2)*aw(1) + m_w(1)*aw(2) - m_v0(2)*av0(1) + m_v0(1)*av0(2);
        dn0(1) =  m_w(2)*aw(0) - m_w(0)*aw(2) + m_v0(2)*av0(0) - m_v0(0)*av0(2);
        dn0(2) = -m_w(1)*aw(0) + m_w(0)*aw(1) - m_v0(1)*av0(0) + m_v0(0)*av0(1);
  
        Spatial::Force c(df,dn0);
  
        return c;
      }
  
      /*! Plucker Transform */
      PluckerTransform::PluckerTransform()
      {}
  
      PluckerTransform::PluckerTransform(matrix3d lR,
  				       vector3d lp):
        m_R(lR), m_p(lp) {}
  
  
      PluckerTransform  PluckerTransform::operator*(PluckerTransform &a)
      {
        PluckerTransform c;
        // Rotation
        c.m_R = m_R*a.m_R;
        // position
        matrix3d aRT;
        aRT = a.m_R; aRT.transposeInPlace();
        c.m_p = a.m_p + aRT * m_p;
        return c;
      }
  
  
      Force  PluckerTransform::operator*( Force &f)
      {
        Force c;
        // Computes the angular velocity
        vector3d NE_tmp,NE_tmp2,NE_tmp3;
        S3_VECTOR_CROSS_PRODUCT(NE_tmp,m_p,f.f());
        NE_tmp2=f.n0()-NE_tmp;
  
        NE_tmp = m_R*NE_tmp2;
        c.n0(NE_tmp);
  
        // Computes the linear velocity
        NE_tmp3 = f.f();
        NE_tmp = m_R*NE_tmp3;
        c.f(NE_tmp);
        return c;
      }
  
      void PluckerTransform::inverse( PluckerTransform &a)
      {
        m_R = a.m_R; m_R.transposeInPlace();
        vector3d NE_tmp;
        NE_tmp = a.p()* -1.0;
        m_p = a.m_R*NE_tmp;
      }
  
      /* ----------> correcting the formulas by L.S (inverting angular and linear
                     expressions). */
      Velocity PluckerTransform::operator*( Velocity &v)
      {
        Velocity c;
        // Computes the linear velocity
        vector3d NE_tmp,NE_tmp2;
        S3_VECTOR_CROSS_PRODUCT(NE_tmp,m_p,v.w());
        NE_tmp2=v.v0()-NE_tmp;
        c.v0(m_R*NE_tmp2);
        // Computes the angular velocity
        c.w(m_R*v.w());
        return c;
      }
      /* ----------> correcting the formulas by L.S (inverting angular and linear
                     expressions). */
      Acceleration PluckerTransform::operator*( Acceleration &v)
      {
        Acceleration c;
        // Computes the linear velocity
        vector3d NE_tmp,NE_tmp2;
        S3_VECTOR_CROSS_PRODUCT(NE_tmp,m_p,v.dw());
        NE_tmp2=v.dv0()-NE_tmp;
        c.dv0(m_R*NE_tmp2);
  
        // Computes the angularvelocity
        c.dw(m_R*v.dw());
        return c;
      }
  
      /* ----------> Adding the operator = to associate 2 equivalent quantities
                     by L.S. */
      PluckerTransform& PluckerTransform::operator=( const PluckerTransform &a)
      {
        if (&a == this) return *this;
        m_R = a.m_R;
        m_p = a.m_p;
        return *this;
      }
  
      /* ----------> The Transpose of a Plucker Transform by L.S is not the
       * actual Transpose of a pluckertransform X but it allows the
       * implementation of the remaining operations on pluckertransforms since
       * there is a difference between motion pluckertransform X and force
       * pluckertransform XF; with XF = X^{-T}.
  
       * Note that the operator* working as * f_res = XT*f, takes X as input
       * (XT=X) but the computation inside the operator is done for the case of
       * XF^{-1}.f; XF^{-1}=X^{T} so this is truly expressing a Transpose and
       * that the operator* working as v_res =  XT*v, takes X as input (XT=X)
       * but the computation inside the operator is done for the case of
       * X^{-1}.v; X^{-1}=X.inverse so this is not expressing a Transpose
       * cf. Table 2.1 in Springer Handbook of Robotics.
       */
  
      PluckerTransform::PluckerTransform(PluckerTransformTranspose &X):
        m_R(X.m_R),m_p(X.m_p) {}
  
      PluckerTransformTranspose::PluckerTransformTranspose()
      {}
  
      PluckerTransformTranspose::PluckerTransformTranspose(matrix3d lR,
  							 vector3d lp):
        m_R(lR), m_p(lp) {}
  
      PluckerTransformTranspose::PluckerTransformTranspose(PluckerTransform &X):
        m_R(X.m_R),m_p(X.m_p) {}
  
      Force PluckerTransformTranspose::operator*( Force &f)
      {
        Force c;
        // Computes the angular velocity
        vector3d NE_tmp,NE_tmp1,NE_tmp2,NE_tmp3;
        matrix3d Rt;
        Rt = m_R; Rt.transposeInPlace();
        NE_tmp1 = Rt*f.f();
        S3_VECTOR_CROSS_PRODUCT(NE_tmp,m_p,NE_tmp1);
        NE_tmp3 = Rt*f.n0();
        NE_tmp2=NE_tmp3+NE_tmp;
        c.n0(NE_tmp2);
  
        // Computes the linear velocity
        NE_tmp = Rt*f.f();
        c.f(NE_tmp);
        return c;
      }
  
      Velocity PluckerTransformTranspose::operator*( Velocity &v)
      {
        Velocity c;
        // Computes the linear velocity
        vector3d NE_tmp,NE_tmp1,NE_tmp2;
        matrix3d Rt;
        Rt = m_R; Rt.transposeInPlace();
        NE_tmp1 = Rt*v.w();
        S3_VECTOR_CROSS_PRODUCT(NE_tmp,m_p,NE_tmp1);
        NE_tmp2 = Rt*v.v0();
        NE_tmp2 = NE_tmp2+NE_tmp;
  
        c.v0(NE_tmp2);
  
        // Computes the angular velocity
        c.w(Rt*v.w());
        return c;
      }

  } // end of namespace Spatial

} // end of namespace metapod.

#endif /* _SPATIAL_ALGEBRA_H_ */
