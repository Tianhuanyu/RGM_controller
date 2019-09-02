// Copyright  (C)  2007  Francois Cauwe <francois at cauwe dot org>
// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#include "chainfksolverpos_rgm.hpp"
#include <iostream>
#include "models.hpp"
#include <cstdlib>

namespace KDL {

    ChainFkSolverPos_rgm::ChainFkSolverPos_rgm(const Chain& _chain):
        chain(_chain),
        d1(0.0748),
        a2(-0.4016),
        a3(-0.3683),
        d4(0.1023),
        d5(0.1023),
        d6(0.1023),
        ZERO_THRESH(0.00000001),
        PI(M_PI)
    {
        
    }

    int ChainFkSolverPos_rgm::JntToCart(const JntArray& q_in, Frame& p_out, int seg_nr) {
        unsigned int segmentNr;
        if(seg_nr<0)
            segmentNr=chain.getNrOfSegments();
        else
            segmentNr = seg_nr;

        //TODO 正运动学

        double _q_in[] ={q_in.data(0),q_in.data(1),q_in.data(2),q_in.data(3),q_in.data(4),q_in.data(5)};
        double _Tout[16];
        //计算正运动学
        forward(_q_in,_Tout);
        
        p_out = TransToFrame(_Tout);
        return 0;
    }
    //这部分没有集成，不能使用
    int ChainFkSolverPos_rgm::JntToCart(const JntArray& q_in, std::vector<Frame>& p_out, int seg_nr)    {
        unsigned int segmentNr;
        if(seg_nr<0)
            segmentNr=chain.getNrOfSegments();
        else
            segmentNr = seg_nr;
        std::cout<<"Error! Not integrate!!"<<std::endl;
       return -1;
    }
     int ChainFkSolverPos_rgm::SIGN(double x) {
      return (x > 0) - (x < 0);
    }
    void ChainFkSolverPos_rgm::forward(const double* q, double* T) {
      double s1 = sin(*q), c1 = cos(*q); q++;
      double q23 = *q, q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
      double s3 = sin(*q), c3 = cos(*q); q23 += *q; q234 += *q; q++;
      double s4 = sin(*q), c4 = cos(*q); q234 += *q; q++;
      double s5 = sin(*q), c5 = cos(*q); q++;
      double s6 = sin(*q), c6 = cos(*q); 
      double s23 = sin(q23), c23 = cos(q23);
      double s234 = sin(q234), c234 = cos(q234);
      *T = c234*c1*s5 - c5*s1; T++;
      *T = c6*(s1*s5 + c234*c1*c5) - s234*c1*s6; T++;
      *T = -s6*(s1*s5 + c234*c1*c5) - s234*c1*c6; T++;
      *T = d6*c234*c1*s5 - a3*c23*c1 - a2*c1*c2 - d6*c5*s1 - d5*s234*c1 - d4*s1; T++;
      *T = c1*c5 + c234*s1*s5; T++;
      *T = -c6*(c1*s5 - c234*c5*s1) - s234*s1*s6; T++;
      *T = s6*(c1*s5 - c234*c5*s1) - s234*c6*s1; T++;
      *T = d6*(c1*c5 + c234*s1*s5) + d4*c1 - a3*c23*s1 - a2*c2*s1 - d5*s234*s1; T++;
      *T = -s234*s5; T++;
      *T = -c234*s6 - s234*c5*c6; T++;
      *T = s234*c5*s6 - c234*c6; T++;
      *T = d1 + a3*s23 + a2*s2 - d5*(c23*c4 - s23*s4) - d6*s5*(c23*s4 + s23*c4); T++;
      *T = 0.0; T++; *T = 0.0; T++; *T = 0.0; T++; *T = 1.0;
  }


    Frame ChainFkSolverPos_rgm::TransToFrame(double* T){
        Vector v = Vector(*(T+3),*(T+7),*(T+11));
        Rotation r = Rotation(*(T),*(T+1),*(T+2),*(T+4),*(T+5),*(T+6),*(T+8),*(T+9),*(T+10));
        return Frame(r,v);
    }

    ChainFkSolverPos_rgm::~ChainFkSolverPos_rgm()
    {
    }


}

KDL::Chain ur5 = KDL::universal_robot5();
static KDL::ChainFkSolverPos_rgm chainrgm_fk_instance = KDL::ChainFkSolverPos_rgm(ur5);
int chainFk_rgm(int32_t ap1,int32_t ap2,int32_t ap3,int32_t ap4,int32_t ap5,int32_t ap6){
    KDL::JntArray q(6);
    q(5) = (ap1  -AP1ZERO)/DRIVER_COUNT*2.0*M_PI ;
    q(4) = (ap2  -AP2ZERO)/DRIVER_COUNT*2.0*M_PI ;
    q(3) = (-ap3 +AP3ZERO)/DRIVER_COUNT*2.0*M_PI +M_PI;
    q(2) = (ap4  -AP4ZERO)/DRIVER_COUNT*2.0*M_PI ;
    q(1) = (-ap5 +AP5ZERO)/DRIVER_COUNT*2.0*M_PI +M_PI;
    q(0) = (ap6  -AP6ZERO)/DRIVER_COUNT*2.0*M_PI ;

    std::cout<<q(5)<<" "<<q(4)<<" "<<q(3)<<" "<<q(2)<<" "<<q(1)<<" "<<q(0)<<" "<<std::endl;

    KDL::Frame T;
    chainrgm_fk_instance.JntToCart(q,T,6);
    std::cout<<T.p.x()<<" "<<T.p.y()<<" "<<T.p.z()<< std::endl;
    double wx = 0;
    double wy = 0;
    double wz = 0;
    double wq = 0;
    T.M.GetQuaternion(wx,wy,wz,wq);

    //std::cout<<wx<<" "<<wy<<" "<<wz<<" "<<wq<< std::endl;


}
