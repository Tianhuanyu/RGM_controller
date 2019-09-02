//thy  2019 8.19 


#ifdef __cplusplus

#ifndef KDLCHAINFKSOLVERPOS_RGM_HPP
#define KDLCHAINFKSOLVERPOS_RGM_HPP

#include "chainfksolver.hpp"
#include "RGMtrajControl.hpp"
#include "math.h"


namespace KDL {

    /**
     * Implementation of a recursive forward position kinematics
     * algorithm to calculate the position transformation from joint
     * space to Cartesian space of a general kinematic chain (KDL::Chain).
     *
     * @ingroup KinematicFamily
     */

    class ChainFkSolverPos_rgm 
    {
    public:
        //这个地方的chain没有实际作用
        ChainFkSolverPos_rgm(const Chain& chain);
        ~ChainFkSolverPos_rgm();

        //输入的JntArray 和Frame 在体系框架内
        int JntToCart(const JntArray& q_in, Frame& p_out, int segmentNr=-1);
        //这种方式不能使用，没有集成！！！！
        int JntToCart(const JntArray& q_in, std::vector<Frame>& p_out, int segmentNr=-1);

    private:
        const Chain chain;
        const double d1;
        const double a2;
        const double a3;
        const double d4;
        const double d5;
        const double d6;
        const double ZERO_THRESH;
        const double PI;

        int SIGN(double x);
        void forward(const double* q, double* T);
        Frame TransToFrame(double* T);

    };

}
#endif

//typedef KDL::ChainFkSolverPos_rgm chainrgm_fk;

#ifdef __cplusplus
extern "C"{
#endif
int chainFk_rgm(int32_t ap1,int32_t ap2,int32_t ap3,int32_t ap4,int32_t ap5,int32_t ap6);
// #include "pvt_src.h"

#ifdef __cplusplus
}
#endif




#endif
