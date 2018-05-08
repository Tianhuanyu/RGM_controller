/*ik_algothrim
*
*state change
*
*	1. get 
*			-kinematics actual pose  
*			-target pose
*	2. compare
*		 	-subtract FRt - FRa
*
*	3. compute jacobi
*			-svd resolve
*
*output method
*
*4. output
*/

#include "RGMikControl.hpp"
#include <cmath>
#include "frames.hpp"

namespace KDL
{

        RGMikCtrl::RGMikCtrl(const Chain& _chain,double _eps,double _eps_joints,double _alpha ):
        nj(_chain.getNrOfJoints()),
        ns(_chain.getNrOfSegments()),
        jac(nj),
        jnt2jac(_chain),
        fksolver(_chain),
        svd(6, nj,Eigen::ComputeThinU | Eigen::ComputeThinV),
        output_vel(6),
        q_actual(6),
        X_actual(Rotation::Identity(),Vector(0,0,1)),
        X_target(Rotation::Identity(),Vector(0,0,1)),
        //jac_inv_persudo(6,6),
        //jac_singular_persudo(1,6),
        RGMiksolver(_chain)
        {
            //jac_inv_persudo = MatrixXq::Identity(6,6);
            //jac_singular_persudo = MatrixXq::Zero(1,6);
        }
        RGMikCtrl::~RGMikCtrl(){

        }

        
    int RGMikCtrl::get_pos(const int &p1,const int &p2,const int &p3,const int &p4,const int &p5,const int &p6,const int &flag)
    {
        //wait for complete
        RGMctrl::pos_trans(p1,p2,p3,p4,p5,p6,q_actual);
        //state change step 1
        fksolver.JntToCart(q_actual,X_actual,6);

        return 0;

        //state change  step 3
        // jnt2jac(actual_position,jac,6);

        // svd.compute(jac.data);
        // int i = 0;
        // for(i=0;i<6;i++){
            
        //     jac_singular_persudo(i)=svd.singularValues()(i)/\
        //         (pow(svd.singularValues()(i),2)+pow(0.03,2));
        // }

        // jac_inv_persudo = svd.matrixV()*\
        //                 MatrixXq(S_diag_persudo)*\
        //                 svd.matrixU().transpose();
    }

    int RGMikCtrl::calcuVelocity(int& v1,int& v2,int& v3,int& v4,int& v5,int& v6){

        int ret = 0;

        Twist twist_diff = diff(X_target,X_actual);

        twist_diff = twist_diff*0.05;
        
        RGMiksolver.CartToJnt(q_actual,twist_diff,output_vel);

        // output
        ret = RGMctrl::vel_trans(output_vel,v1,v2,v3,v4,v5,v6);

        return ret;

    }


    int RGMikCtrl::get_target(const FRAME& fr,const int& flag){

        Vector vec = Vector(fr.point[0],fr.point[1],fr.point[2]);
        Rotation rot = Rotation::Quaternion(fr.orientation[0],fr.orientation[1],
                                    fr.orientation[2],fr.orientation[3]);

         
        X_target = Frame(Rotation::Quaternion(fr.orientation[0],fr.orientation[1],
                                fr.orientation[2],fr.orientation[3]),
        Vector(fr.point[0],fr.point[1],fr.point[2]));

        return 0;      

    }

}

void rgm_Ctrl_init_wrap(int* handle){
    KDL::RGMikCtrl *p = NULL;
    p = new KDL::RGMikCtrl(ur5);

    RGMikCtrl_Vector.push_back(p);

    *handle = 0;

    return;
}

int get_pos_ik_wrap(int handle,int p1, int p2,int p3,
                    int p4,int p5,int p6,FRAME fr){
    
    int ret = 0;

    ret = RGMikCtrl_Vector[handle]->get_target(fr,0);

    ret = ret||( RGMikCtrl_Vector[handle]->get_pos(p1,p2,p3,p4,p5,p6,0) );

    return ret;

}

int calcuVelocity_wrap_ctrl(int handle,int* v1,int* v2,int* v3,int* v4,int* v5,int* v6)
{
    return RGMikCtrl_Vector[handle]->calcuVelocity(*v1,*v2,*v3,*v4,*v5,*v6);
}

void rgm_Ctrl_dele_wrap(int* handle){
    RGMikCtrl_Vector.clear();
    *handle = -1;
    return ;
}
