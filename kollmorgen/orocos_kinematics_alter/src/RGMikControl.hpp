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

#ifndef KDL_RGM_IK_CTRL
#define KDL_RGM_IK_CTRL

#ifdef __cplusplus

#include "chain.hpp"
#include "frames.hpp"
#include "jntarray.hpp"
#include "RGMcontrol.hpp"
#include <Eigen/Dense>
#include "models.hpp"
#include <vector>
#include "RGMcontrol.hpp"
#include "chainiksolvervel_wdls.hpp"
#include "chainfksolverpos_recursive.hpp"


typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> MatrixXq;

struct FRAME{
    float point[3];
    float orientation[4];
};


namespace KDL
{
    
    class RGMikCtrl :public RGMctrl
    {
        public:
            explicit RGMikCtrl(const Chain& _chain,double _eps =1e-5,
        double _eps_joints = 1e-5,double _alpha = 0.01);
        ~RGMikCtrl();
        
        virtual int get_pos(const int &p1,const int &p2,const int &p3,const int &p4,const int &p5,const int &p6,const int &flag);
            
        virtual int get_vel(const int &v1,const int &v2,const int &v3,const int &v4,const int &v5,const int &v6,const int &flag){return 0;};
    
        virtual int get_tor(const int &t1,const int &t2,const int &t3,const int &t4,const int &t5,const int &t6,const int &flag){return 0;};
    
        virtual int get_target(const FRAME &fr,const int& flag);
    
    
        /*output*/
        virtual int calcuPosition(int* p1,int* p2,int* p3,int* p4,int* p5,int* p6){return 0;};

        virtual int calcuVelocity(int& v1,int& v2,int& v3,int& v4,int& v5,int& v6);

        virtual int calcuTorque(int* t1,int* t2,int* t3,int* t4,int* t5,int* t6){return 0;};

        
        
        
        
        /**
         * \brief for internal use only.
         *
         * contains the last value for the Jacobian after an execution of compute_jacobian.
         */
        Jacobian jac;

        ChainJntToJacSolver jnt2jac;

        Eigen::JacobiSVD<MatrixXq> svd;
        
        JntArray output_vel;

        JntArray q_actual;

        Frame X_target;

        Frame X_actual;

        ChainIkSolverVel_wdls RGMiksolver;

        private:

        unsigned int nj;

        unsigned int ns;
        
        MatrixXq jac_inv_persudo;

        //MatrixXq jac_singular_persudo;

        ChainFkSolverPos_recursive fksolver;
  
    };

}

static std::vector<KDL::RGMikCtrl *> RGMikCtrl_Vector;
static KDL::Chain ur5 = KDL::universal_robot5();
#endif

#ifdef __cplusplus
extern "C"{
#endif

void rgm_Ctrl_init_wrap(int* handle);

int get_pos_ik_wrap(int handle,int p1,int p2,int p3,int p4,int p5,int p6,FRAME fr);

int calcuVelocity_wrap_ctrl(int handle,int* v1,int* v2,int* v3,int* v4,int* v5,int* v6);

void rgm_Ctrl_dele_wrap(int* handle);

#ifdef __cplusplus
}
#endif

#endif