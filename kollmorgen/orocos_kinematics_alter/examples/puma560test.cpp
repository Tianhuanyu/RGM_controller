#include <chain.hpp>
#include "models.hpp"
#include <frames_io.hpp>
#include <kinfam_io.hpp>

#include <chainfksolverpos_recursive.hpp>
#include <chainidsolver_recursive_newton_euler.hpp>
#include <chainiksolverpos_lma.hpp>
#include "jacobian.hpp"
#include "chainjnttojacsolver.hpp"

typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> MatrixXq;
Eigen::JacobiSVD<MatrixXq> svd(6,6,Eigen::ComputeThinU | Eigen::ComputeThinV);

using namespace KDL;

int main(int argc , char** argv){
    
    Chain p560=Puma560();
    //Chain p560;
//    p560.addSegment(Segment(Joint(Joint::RotX),Frame::Identity(),RigidBodyInertia(1.0,Vector(0.0,1.0,.0),RotationalInertia(1.0,2.0,3.0))));
//    p560.addSegment(Segment(Joint(Joint::RotY),Frame(Rotation::Identity(),Vector(0,2,0)),RigidBodyInertia(1.0,Vector(1.0,0.0,.0),RotationalInertia(1.0,2.0,3,4,5,6))));
//    p560.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation::Identity(),Vector(2,0,0)),RigidBodyInertia(1.0,Vector(0.0,0.0,1),RotationalInertia(1.0,2.0,3,4,5,6))));
    
    JntArray q(p560.getNrOfJoints());
    JntArray qdot(p560.getNrOfJoints());
    JntArray qdotdot(p560.getNrOfJoints());
    JntArray tau(p560.getNrOfJoints());
    Wrenches f(p560.getNrOfSegments());
    JntArray q_out(6);

    for(unsigned int i=0;i<p560.getNrOfJoints();i++){
      q(i)=0.0;
      qdot(i)=0.0;
      qdotdot(i)=0.0;
      
      //if(i<2)
      //{
	std::cout << "give q(" << i+1 << ")\n" << std::endl;
	std::cin >> q(i);
	// std::cout << "give qdot(" << i+1 << ")\n" << std::endl;
	// std::cin >> qdot(i);
	// std::cout << "give qdotdot(" << i << ")\n" << std::endl;
	// std::cin >> qdotdot(i);
      //}
        
    }
    // std::cout<<"tau0: "<<tau<<std::endl;
   // std::cout<<"f: "<<f<<std::endl;
    
    ChainFkSolverPos_recursive fksolver(p560);
    Frame T;
    Frame T1;
    ChainIkSolverPos_LMA iksolver_thy(p560);
    ChainJntToJacSolver jnt2jac_thy(p560);
    Jacobian jac(6);

    // ChainIdSolver_RNE idsolver(p560,Vector(0.0,0.0,-9.81));
    
    // #include <time.h>
    // time_t before,after;
    // time(&before);
    // unsigned int k=0;
    // for(k=0;k<1000;k++){
        fksolver.JntToCart(q,T);
        std::cout<<"T:"<<T<<std::endl;

        iksolver_thy.CartToJnt(q_out,T,q_out);
        std::cout<<q_out<<std::endl;

        fksolver.JntToCart(q_out,T1);
        std::cout<<"T1:"<<T1<<std::endl;

        jnt2jac_thy.JntToJac(q,jac,6);
        std::cout<<"jac:"<<jac<<std::endl;
        svd.compute(jac.data);
        std::cout << "Singular values : " << svd.singularValues().transpose()<<"\n"<<std::endl;

        jnt2jac_thy.JntToJac(q,jac,3);
        std::cout<<"jac:"<<jac<<std::endl;
        svd.compute(jac.data);
        std::cout << "Singular values : " << svd.singularValues().transpose()<<"\n"<<std::endl;

        jnt2jac_thy.JntToJac(q,jac);

        svd.compute(jac.data);

        std::cout<<"jac:"<<jac<<std::endl;
        std::cout<<"jac^-1:"<<jac.data.inverse()<<std::endl;
        std::cout << "Singular values : " << svd.singularValues().transpose()<<"\n"<<std::endl;
        
        
        std::cout<<"puma560:"<<p560.getNrOfSegments()<<std::endl;

    // }
    // time(&after);
    // std::cout<<"elapsed time for FK: "<<difftime(after,before)<<" seconds for "<<k<<" iterations"<<std::endl;
    // std::cout<<"time per iteration for FK: "<<difftime(after,before)/k<<" seconds."<<std::endl;
    //time(&before);
    //for(k=0;k<1e7;k++)
        // idsolver.CartToJnt(q,qdot,qdotdot,f,tau);
        //time(&after);
        //std::cout<<"elapsed time for ID: "<<difftime(after,before)<<" seconds for "<<k<<" iterations"<<std::endl;
        //std::cout<<"time per iteration for ID: "<<difftime(after,before)/k<<" seconds."<<std::endl;

    // std::cout<<"tau: "<<tau<<std::endl;


}
    
