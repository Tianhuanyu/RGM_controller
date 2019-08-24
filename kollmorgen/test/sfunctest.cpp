#include "chainsfunc.hpp"
#include "models.hpp"
#include "frames.hpp"
#include "jntarray.hpp"
#include <vector>
#include <Eigen/Core>

using namespace KDL;
int main(){
    Chain ur5 = universal_robot5();

    JntArray q_init(6);
    q_init(0) = 0;
    q_init(1) = 0;
    q_init(2) = 0;
    q_init(3) = 0;
    q_init(4) = 0;
    q_init(5) = 0;

    JntArray q_init1(6);
    q_init1(0) = 0;
    q_init1(1) = 0;
    q_init1(2) = 0;
    q_init1(3) = 0;
    q_init1(4) = 0;
    q_init1(5) = 0;

    std::vector<double> times;
    times.push_back(0.0);
    times.push_back(1.0);
    times.push_back(2.0);
    times.push_back(3.0);

    std::vector<KDL::Frame> frames;
    frames.push_back(Frame(Vector(0.3,0.3,0.2)));
    frames.push_back(Frame(Vector(0.3,0.3,0.3)));
    frames.push_back(Frame(Vector(0.3,0.3,0.4)));
    frames.push_back(Frame(Vector(0.3,0.3,0.5)));

    std::vector<Eigen::Vector2d> vc;
    vc.push_back(Eigen::Vector2d(1,2));
    vc.push_back(Eigen::Vector2d(2,5));
    vc.push_back(Eigen::Vector2d(3,6));
    vc.push_back(Eigen::Vector2d(4,8));


    chainsfunc sfunc(ur5,frames,times,q_init);
    // ChainIkSolverPos_rgm ik = ChainIkSolverPos_rgm(ur5);
    // ik.CartToJnt(q_init,frames.at(1),q_init1);
    // std::cout<<"q_init1 = "<< q_init1(0)<<", "<<q_init1(1)<<std::endl;
    // Jointsfunc js(vc);
    
    double i = 1;
    double theta = 0.0;
    for(i=1;i<4.1;i=i+0.1){
        q_init1 = sfunc.calculate_cube_q(i);
        //theta = js.calculate_cube_theta(i);
        std::cout<<"output = "<<q_init1(0)<<" "<< q_init1(1)<<" "<< q_init1(2)\
        <<" "<< q_init1(3)<<" "<< q_init1(4)<<" "<< q_init1(5)<<"; i="<<i<<std::endl;
        //cout<<q_init1;

    }


}