// Copyright 2012,
//
// Olivier STASSE
//
// LAAS, CNRS
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


#ifndef METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_HH
# define METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_HH

# include "metapod/tools/fwd.hh"

namespace metapod
{

  namespace Spatial
  {


    template <>
    vector6d ConstraintMotionOneAxis<AxisX>::operator*(const Spatial::Transform &X) const
    {
      vector6d tmp = vector6d::Zero();			    
      tmp.segment<3>(0) = X.E().col(AxisX);                             
      tmp.segment<3>(3) = X.E()*vector3d(0,-X.r()[2],X.r()[1]);     
      return tmp;                                                   
    }

    template<>
    vector6d ConstraintMotionOneAxis<AxisY>::operator*(const Spatial::Transform &X) const
    {
      vector6d tmp = vector6d::Zero();			    
      tmp.segment<3>(0) = X.E().col(AxisY);                             
      tmp.segment<3>(3) = X.E()*vector3d(X.r()[2],0,-X.r()[0]);     
      return tmp;                                                   
    }
    
    template<>
    vector6d ConstraintMotionOneAxis<AxisZ>::operator*(const Spatial::Transform &X) const
    {
      vector6d tmp = vector6d::Zero();			    
      tmp.segment<3>(0) = X.E().col(AxisZ);                             
      tmp.segment<3>(3) = X.E()*vector3d(-X.r()[1],X.r()[0],0);     
      return tmp;                                                   
      
    }

    
    /* Operator Inertia = Inertia * float */
    

    /* */
    template<>
    vector6d OperatorMul< vector6d, Inertia, ConstraintMotionOneAxisAxisX>::
      mul(const Inertia & m,
	  const ConstraintMotionOneAxisAxisX &a) const
    {
      vector6d r;
      for(unsigned int i=0;i<3;i++)
	r[i] = m.I().col(0)[i];
      matrix3d msh = -skew(m.h());
      for(unsigned int i=0;i<3;i++)
	r[i+3] = msh(0,i);
      return r;
    }
    
    vector6d operator*(const Inertia & m,
		       const ConstraintMotionOneAxisAxisX &a) 
    {
      vector6d r;
      r[0] = m.I()(0,0); r[1] = m.I()(1,0);r[2] = m.I()(2,0);      
      r[3] = 0.0; r[4] = -m.h()(2); r[5] = m.h()(1);
      return r;
    }
    
    template<>
    vector6d OperatorMul< vector6d, Inertia, ConstraintMotionOneAxisAxisY>::
      mul(const Inertia & m,
	  const ConstraintMotionOneAxisAxisY &a) const
    {
      vector6d r;
      for(unsigned int i=0;i<3;i++)
	r[i] = m.I().col(1)[i];
      matrix3d msh = -skew(m.h());
      for(unsigned int i=0;i<3;i++)
	  r[i+3] = msh(1,i);
      return r;
    }
    
    template<>
    vector6d OperatorMul< vector6d, Inertia, ConstraintMotionOneAxisAxisZ>::
      mul(const Inertia & m,
	  const ConstraintMotionOneAxisAxisZ &a) const
    {
      vector6d r;
      for(unsigned int i=0;i<3;i++)
	r[i] = m.I().col(2)[i];
      matrix3d msh = -skew(m.h());
	for(unsigned int i=0;i<3;i++)
	  r[i+3] = msh(2,i);
	return r;
    }

    // Class of motion constraint with a rotation around a general axis.
    class ConstraintMotionAnyAxis
    {
      public:
        // Constructors
      ConstraintMotionAnyAxis(double axisx, 
			      double axisy,
			      double axisz)
      { m_S(0) = axisx; m_S(1) = axisy; m_S(2) = axisz;};

      vector6d operator*(const Spatial::Transform &X) const;
      vector6d operator*(double d) const;

      private:
        vector6d m_S;

      public:
      const vector6d & S() const {return m_S;}
      vector6dt transpose() const {return m_S.transpose();}
    };

    vector6d ConstraintMotionAnyAxis::operator*
    (const Spatial::Transform &X) const
    {
      vector6d tmp = vector6d::Zero();
      tmp.segment<3>(0) = X.E()*m_S.segment<3>(0);                    
      tmp.segment<3>(3) = -X.E()*
	X.r().cross(vector3d(m_S(0),m_S(1),m_S(2))); 
      return tmp;                                                   
    }

    vector6d ConstraintMotionAnyAxis::operator*
    (double x) const
    {
      vector6d tmp = vector6d::Zero();
      tmp.segment<3>(0) = x*m_S.segment<3>(0);                    
      return tmp;                                                   
    }

    template<>
    vector6d OperatorMul< vector6d, Inertia, ConstraintMotionAnyAxis>::
      mul(const Inertia & m,
	  const ConstraintMotionAnyAxis &a) const
    {
      vector6d r;
      for(unsigned int i=0;i<3;i++)
	r[i] = m.I()(i,0)*a.S()[0]+ 
	  m.I()(i,1)*a.S()[1]+
	  m.I()(i,2)*a.S()[2];
      matrix3d msh = -skew(m.h());
      for(unsigned int i=0;i<3;i++)
	r[i+3] = msh(i,0)*a.S()[0]+ 
	  msh(i,1)*a.S()[1]+
	  msh(i,2)*a.S()[2];
      return r;
    }
    
    vector6d operator*(const Inertia & m,
		       const ConstraintMotionAnyAxis &a) 
    {
      OperatorMul<vector6d,Inertia, ConstraintMotionAnyAxis > om;
      return om.mul(m,a);
    }

    // Class of motion constraint with a free flyer.
    class ConstraintMotionFreeFlyer
    {
      public:
        // Constructors
      ConstraintMotionFreeFlyer()
      { m_S = matrix6d::Zero(); };

      matrix6d operator*(const Spatial::Transform &X) const;
      matrix6d operator*(double d) const;

      private:
        matrix6d m_S;

      public:
      void setlocalR(const matrix3d &localR)  
      {       m_S.block<3,3>(0,3) = m_S.block<3,3>(3,0) = localR; }
      const matrix6d & S() const {return m_S;}
      matrix6d transpose() const {return m_S.transpose();}
    };

    matrix6d ConstraintMotionFreeFlyer::operator*
    (const Spatial::Transform &X) const
    {
      matrix6d tmp = matrix6d::Zero();                              
      tmp.block<3,3>(0,3) = X.E() * m_S.block<3,3>(0,3);              
      tmp.block<3,3>(3,0) = X.E() * m_S.block<3,3>(3,0);              
      tmp.block<3,3>(3,3) = -X.E() * Spatial::skew(X.r()) * m_S.block<3,3>(0,3); 
      return tmp;                                                   
    }

    matrix6d ConstraintMotionFreeFlyer::operator*
    (double x) const
    {
      matrix6d tmp = matrix6d::Zero();
      tmp = x*m_S;                    
      return tmp;                                                   
    }

    template<>
    matrix6d OperatorMul< matrix6d, Inertia, ConstraintMotionFreeFlyer>::
      mul(const Inertia & m,
	  const ConstraintMotionFreeFlyer &a) const
    {
      matrix6d r;
      r=matrix6d::Zero();
      r.block<3,3>(0,0)=skew(m.h())*a.S().block<3,3>(3,0);
      r.block<3,3>(0,3)=m.I()*a.S().block<3,3>(0,3);
      r.block<3,3>(3,0)=m.m()*a.S().block<3,3>(3,0);
      r.block<3,3>(3,3)=-skew(m.h())*a.S().block<3,3>(0,3);
      return r;
    }
    
    matrix6d operator*(const Inertia & m,
		       const ConstraintMotionFreeFlyer &a) 
    {
      OperatorMul<matrix6d,Inertia, ConstraintMotionFreeFlyer > om;
      return om.mul(m,a);
    }

  } // End of spatial namespace
} // End of metapod namespace
#endif /* METAPOD_SPATIAL_ALGEBRA_CONSTRAINT_MOTION_HH */
