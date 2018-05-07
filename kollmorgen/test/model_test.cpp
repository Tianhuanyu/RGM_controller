#include <chain.hpp>
#include "models.hpp"
#include <frames_io.hpp>
#include <kinfam_io.hpp>
#include <cmath>

#include <chainfksolverpos_recursive.hpp>
#include <chainidsolver_recursive_newton_euler.hpp>
#include <chainiksolverpos_lma.hpp>
#include "jacobian.hpp"
#include "chainjnttojacsolver.hpp"
#include "chainiksolvervel_wdls.hpp"
#include "RGMcontrol.hpp"
#include "RGMtrajControl.hpp"


typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> MatrixXq;
Eigen::JacobiSVD<MatrixXq> svd(6,6,Eigen::ComputeThinU | Eigen::ComputeThinV);


using namespace KDL;

int main(){

    Chain ur5 = universal_robot5();
    //Chain ur5;
//    ur5.addSegment(Segment(Joint(Joint::RotX),Frame::Identity(),RigidBodyInertia(1.0,Vector(0.0,1.0,.0),RotationalInertia(1.0,2.0,3.0))));
//    ur5.addSegment(Segment(Joint(Joint::RotY),Frame(Rotation::Identity(),Vector(0,2,0)),RigidBodyInertia(1.0,Vector(1.0,0.0,.0),RotationalInertia(1.0,2.0,3,4,5,6))));
//    ur5.addSegment(Segment(Joint(Joint::RotZ),Frame(Rotation::Identity(),Vector(2,0,0)),RigidBodyInertia(1.0,Vector(0.0,0.0,1),RotationalInertia(1.0,2.0,3,4,5,6))));
    
    std::cout << "ur5 joint number = " << ur5.getNrOfJoints() << "\n" << std::endl;
    JntArray q(ur5.getNrOfJoints());
    JntArray qdot(ur5.getNrOfJoints());
    JntArray qdotdot(ur5.getNrOfJoints());
    JntArray tau(ur5.getNrOfJoints());
    Wrenches f(ur5.getNrOfSegments());
    JntArray q_out(6);

    for(unsigned int i=0;i<ur5.getNrOfJoints();i++){
      q(i)=0.0;
      qdot(i)=0.0;
      qdotdot(i)=0.0;
      

	std::cout << "give q(" << i+1 << ")\n" << std::endl;
	std::cin >> q(i);

        
    }

    
    ChainFkSolverPos_recursive fksolver(ur5);
    Frame T;
    Frame T1;
    ChainIkSolverPos_LMA iksolver_thy(ur5);
    ChainJntToJacSolver jnt2jac_thy(ur5);
    Jacobian jac(6);

    double x,y,z,w;
    int i=0;
    for(i=0;i<7;i++){
        fksolver.JntToCart(q,T,i);
        //std::cout<<"T:"<<T<<std::endl;
        T.M.GetQuaternion(x,y,z,w);
        std::cout<<"Q : ["<<x<<"\t"
                          <<y<<"\t"
                          <<z<<"\t"
                          <<w<<"]\t\n"
                          <<std::endl;

        std::cout<<"Position :"
                <<T.p
                <<"\n"
                <<std::endl;

        std::cout<<"Joint::RotZ: "
                <<Joint::RotZ
                <<"\n"
                <<std::endl;
        }   

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
        MatrixXq U = MatrixXq::Identity(6,6);
        U = svd.matrixU();
        MatrixXq V = svd.matrixV();
        MatrixXq S = svd.singularValues();

        MatrixXq S_persudo = S; 
        int j = 0;
        for(j=0;j<6;j++){
            std::cout<<"singularValues = "<< svd.singularValues()(j)<<" "<<S(j)<<"\n"<<std::endl;
            S_persudo(j) =  S(j)/(pow(S(j),2)+pow(0.05,2));
            std::cout<<"singularValues = "<< S_persudo(j)<<"\n"<<std::endl;
        }
        Eigen::DiagonalMatrix<double,Eigen::Dynamic> S_diag(S);
        Eigen::DiagonalMatrix<double,Eigen::Dynamic> S_diag_persudo(S_persudo);


        std::cout<<"S = "<< MatrixXq(S_diag)<<"\n"<<std::endl;
        std::cout<<"U = "<< U<<"\n"<<std::endl;
        std::cout<<"V = "<< V<<"\n"<<std::endl;

        MatrixXq Jout = svd.matrixU()*MatrixXq(S_diag)*svd.matrixV().transpose();

        std::cout<<"jac out = "<< Jout<<"\n"<<std::endl;
        std::cout<<"jac in = "<< jac<<"\n"<<std::endl;


        std::cout<<"jac inverse = "<< jac.data.inverse()<<"\n"<<std::endl;
        std::cout<<"jac svd inverse = "<< 
        svd.matrixV()*MatrixXq(S_diag_persudo)*svd.matrixU().transpose()<<
        "\n"<<std::endl;



        ChainIkSolverVel_wdls iksolver_vel(ur5);

        JntArray q_out_vel(6);
        std::cout<<"before - q_out_vel = "<<q_out_vel<<"\n"<<std::endl;
        Twist delta_T = diff(T1,T);

        iksolver_vel.CartToJnt(q,delta_T,q_out_vel);
        std::cout<<"after - q_out_vel = "<<q_out_vel<<"\n"<<std::endl;


        LinkQueue_pvt pQueue;

        RGMtrajCtrl pos_test(pQueue,111);

        int p[6];
        int rec = 0;

        rec = pos_test.pos_trans(q,p[0],p[1],p[2],p[3],p[4],p[5]);
        rec = pos_test.pos_trans(p[0],p[1],p[2],p[3],p[4],p[5],q_out_vel);

        std::cout<<"q ="<<q<<"\n"<<std::endl;

        std::cout<<"p = ["
        <<p[0]<<", "
        <<p[1]<<", "
        <<p[2]<<", "
        <<p[3]<<", "
        <<p[4]<<", "
        <<p[5]<<"]"<<"\n"<<std::endl;

        std::cout<<"q_out_vel = "<< q_out_vel<<"\n"<<std::endl;



        //svd.singularValues().asDiagonal().matrix().asDiagonal()


        // std::cout<<"jac:"<<jac<<std::endl;
        // std::cout<<"jac^-1:"<<jac.data.inverse()<<std::endl;
        // std::cout << "Singular values : " << svd.singularValues().transpose()<<"\n"<<std::endl;
        
        
        // std::cout<<"puma560:"<<ur5.getNrOfSegments()<<std::endl;

    




}