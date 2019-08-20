//thy  2019 8.19 

#ifndef KDLCHAINFKSOLVERPOS_RGM_HPP
#define KDLCHAINFKSOLVERPOS_RGM_HPP

#include "chainfksolver.hpp"

namespace KDL {

    /**
     * Implementation of a recursive forward position kinematics
     * algorithm to calculate the position transformation from joint
     * space to Cartesian space of a general kinematic chain (KDL::Chain).
     *
     * @ingroup KinematicFamily
     */
    class ChainFkSolverPos_rgm : public ChainFkSolverPos
    {
    public:
        //这个地方的chain没有实际作用
        ChainFkSolverPos_rgm(const Chain& chain);
        ~ChainFkSolverPos_rgm();

        //输入的JntArray 和Frame 在体系框架内
        virtual int JntToCart(const JntArray& q_in, Frame& p_out, int segmentNr=-1);
        virtual int JntToCart(const JntArray& q_in, std::vector<Frame>& p_out, int segmentNr=-1);

    private:
        const Chain chain;
    };

}

#endif
