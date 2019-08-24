#ifndef KDL_CHAINIKSOLVERPOS_RGM_HPP
#define KDL_CHAINIKSOLVERPOS_RGM_HPP



#include "chainiksolver.hpp"
#include "chain.hpp"
#include <Eigen/Dense>
#include <math.h>

namespace KDL
{

class ChainIkSolverPos_rgm : public KDL::ChainIkSolverPos
{

    public:
        ChainIkSolverPos_rgm(const KDL::Chain& _chain);
    
        virtual int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& T_base_goal, KDL::JntArray& q_out);

        virtual ~ChainIkSolverPos_rgm();
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

        Frame TransToFrame(double* T);
        int SIGN(double x);
        int inverse(const double* T, double* q_sols, double q6_des);

};

}; // namespace KDL







#endif
