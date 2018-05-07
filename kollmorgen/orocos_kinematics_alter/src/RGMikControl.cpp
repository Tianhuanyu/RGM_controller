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
        jac_singular_persudo(1,6)
        {
            jac_inv_persudo = MatrixXq::Identity(6,6);
            jac_singular_persudo = MatrixXq::Zero(1,6);
        };

        
    virtual int get_pos(const int &p1,const int &p2,const int &p3,const int &p4,const int &p5,const int &p6,const int &flag);
    {
        //wait for complete
        trans_p()
        //state change step 1
        fksolver.JntToCart(q_actual,X_actual,6);








        //state change  step 3
        jnt2jac(actual_position,jac,6);

        svd.compute(jac.data);
        int i = 0;
        for(i=0;i<6;i++){
            
            jac_singular_persudo(i)=svd.singularValues()(i)/\
                (pow(svd.singularValues()(i),2)+pow(0.03,2));
        }

        jac_inv_persudo = svd.matrixV()*\
                        MatrixXq(S_diag_persudo)*\
                        svd.matrixU().transpose();
    }

    virtual int calcuVelocity(int* v1,int* v2,int* v3,int* v4,int* v5,int* v6){





    }







}