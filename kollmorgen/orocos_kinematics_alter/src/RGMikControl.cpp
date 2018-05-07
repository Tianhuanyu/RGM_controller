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

namespace KDL
{
    RGMikCtrl(
            const KDL::Chain& _chain,
            double _eps =1e-5,
            double _eps_joints = 1e-5
            double _alpha = 0.01 
        ):
        nj(_chain.getNrOfJoints()),
        ns(_chain.getNrOfSegments())
        jac(nj),
        jnt2jac(_chain),
        fksolver(_chain),
        svd(6, nj,Eigen::ComputeThinU | Eigen::ComputeThinV),
        output_vel(6),
        q_actual(6),
        X_actual(Rotation::Identity(),Vector(0,0,1)),
        X_target(Rotation::Identity(),Vector(0,0,1)),
        jac_inv_persudo(6,6),
        jac_singular_persudo(1,6),
        RGMiksolver(_chain)
        {
            jac_inv_persudo = MatrixXq::Identity(6,6);
            jac_singular_persudo = MatrixXq::Zero(1,6);
        };

        
    virtual int get_pos(const int &p1,const int &p2,const int &p3,const int &p4,const int &p5,const int &p6,const int &flag);
    {
        //wait for complete
        RGMctrl::pos_trans(p1,p2,p3,p4,p5,p6,q_actual);
        //state change step 1
        fksolver.JntToCart(q_actual,X_actual,6);



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

    virtual int calcuVelocity(int& v1,int& v2,int& v3,int& v4,int& v5,int& v6){

        int ret = 0;

        Twist twist_diff = diff(X_target,X_actual);

        twist_diff = twist_diff*0.05;
        
        RGMiksolver.JntToCart(q_actual,twist_diff,output_vel);

        // output
        ret = RGMctrl::vel_trans(output_vel,v1,v2,v3,v4,v5,v6)

        return ret;

    }


    int get_target(const FRAME& fr,const int& flag){

        Vector vec = Vector(fr.position[0],fr.position[1],fr.position[2]);
        Rotation rot = Quaternion(fr.orientation[0],fr.orientation[1],
                                    fr.orientation[2],fr.orientation[3]);

         
        X_target.Frame(Quaternion(fr.orientation[0],fr.orientation[1],
                                    fr.orientation[2],fr.orientation[3]),
        Vector(fr.position[0],fr.position[1],fr.position[2]));

        return 0;      

    }

}