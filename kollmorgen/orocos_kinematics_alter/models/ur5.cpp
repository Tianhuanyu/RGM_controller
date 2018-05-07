#include <chain.hpp>
#include "models.hpp"

namespace KDL{

    Chain universal_robot5(){
        Chain universal_robot5;
    /*DH for UR5:
    * a = [0.00000, -0.42500, -0.39225,  0.00000,  0.00000,  0.0000]
    * d = [0.089159,  0.00000,  0.00000,  0.10915,  0.09465,  0.0823]
    * alpha = [ 1.570796327, 0, 0, 1.570796327, -1.570796327, 0 ]
    * q_home_offset = [0, -1.570796327, 0, -1.570796327, 0, 0]
    * joint_direction = [-1, -1, 1, 1, 1, 1]
    * mass = [3.7000, 8.3930, 2.2750, 1.2190, 1.2190, 0.1879]
    * center_of_mass = [ [0, -0.02561, 0.00193], [0.2125, 0, 0.11336], [0.11993, 0.0, 0.0265], [0, -0.0018, 0.01634], [0, 0.0018,0.01634], [0, 0, -0.001159] ]
    */

        //std::cout<<"1111\n"<<std::endl;
        universal_robot5.addSegment(Segment());
        universal_robot5.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,M_PI_2,0.089159,0.0),
                                   RigidBodyInertia(0,Vector::Zero(),RotationalInertia(0,0.35,0,0,0,0))));
        universal_robot5.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(-0.42500,0.0,0.0,-M_PI_2),
                                   RigidBodyInertia(17.4,Vector(-.3638,.006,.2275),RotationalInertia(0.13,0.524,0.539,0,0,0))));
        //universal_robot5.addSegment(Segment());
        universal_robot5.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(-0.39225,0,0.0,0.0),
                                   RigidBodyInertia(4.8,Vector(-.0203,-.0141,.070),RotationalInertia(0.066,0.086,0.0125,0,0,0))));
        universal_robot5.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,M_PI_2,0.10915,-M_PI_2),
                                   RigidBodyInertia(0.82,Vector(0,.019,0),RotationalInertia(1.8e-3,1.3e-3,1.8e-3,0,0,0))));
        //universal_robot5.addSegment(Segment());
        //std::cout<<"2222\n"<<std::endl;
        //universal_robot5.addSegment(Segment());
        universal_robot5.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,-M_PI_2,0.09465,0.0),
                                   RigidBodyInertia(0.34,Vector::Zero(),RotationalInertia(.3e-3,.4e-3,.3e-3,0,0,0))));
        universal_robot5.addSegment(Segment(Joint(Joint::RotZ),
                                   Frame::DH(0.0,0.0,0.0823,0.0),
                                   RigidBodyInertia(0.09,Vector(0,0,.032),RotationalInertia(.15e-3,0.15e-3,.04e-3,0,0,0))));
        universal_robot5.addSegment(Segment());
        //std::cout<<"3333\n"<<std::endl;
        return universal_robot5;

    }


}