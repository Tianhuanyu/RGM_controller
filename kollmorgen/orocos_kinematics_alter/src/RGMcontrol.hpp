#ifndef KDL_RGM_CTRL
#define KDL_RGM_CTRL

#include "chain.hpp"
#include "frames.hpp"
#include "solveri.hpp"
#include "jntarray.hpp"
//#include "intarry.hpp"


namespace KDL
{
    typedef std::vector<Twist> Twists;

    /**
     * \brief:this class make a control flow
     * and introduce an alogrithm
     * 
     * /
     */
    const double RGM_LPR = 524288.0;
    const double position_calibrations[6] = {RGM_LPR, RGM_LPR-43690, -142000+RGM_LPR, 217700, RGM_LPR-36410, 0}; // to_driver_count = phy_count + cali

    class RGMctrl : public SolverI
    {
        public:
            
            /**
             * @param pn input joint position
             * @param q output q vector 
             * 
             */
            virtual int get_pos(const int &p1,const int &p2,const int &p3,
                                const int &p4,const int &p5,const int &p6,const int &flag) = 0;
            
            /**
             * @param vn input joint velocity
             * @param qd output qdot vector 
             * 
             */
            virtual int get_vel(const int &v1,const int &v2,const int &v3,
                                const int &v4,const int &v5,const int &v6,const int &flag) =0;

            /**
             * @param tn input joint torque
             * @param tau_c output tau(current) vector 
             * 
             */
            virtual int get_tor(const int &t1,const int &t2,const int &t3,
                                const int &t4,const int &t5,const int &t6,const int &flag) =0;

            // /**
            //  * @param &q input joint tor
            //  * @param tau_c output tau(current) vector 
            //  * 
            //  */
             virtual int calcuPosition(int* p1,int* p2,int* p3,
                                        int* p4,int* p5,int* p6) = 0;

            /**
             * @param &q input joint tor
             * @param tau_c output tau(current) vector 
             * 
             */
            virtual int calcuVelocity(int* v1,int* v2,int* v3,
                                    int* v4,int* v5,int* v6) = 0;

            /**
             * @param &q input joint tor
             * @param tau_c output tau(current) vector 
             * 
             */
            virtual int calcuTorque(int* t1,int* t2,int* t3,
                                    int* t4,int* t5,int* t6) = 0;


            inline int pos_trans(const int& p1,const int& p2,const int& p3,
                                const int& p4,const int& p5,const int& p6,JntArray& q_out)const
                                
                {
                    if(q_out.rows()!=6)
                        return(E_SIZE_MISMATCH);

                    q_out(0) = fmod(((p1-position_calibrations[0])/RGM_LPR*(2*M_PI)+M_PI),(2*M_PI))-M_PI;
                    q_out(1) = fmod(((p2-position_calibrations[1])/RGM_LPR*(2*M_PI)+M_PI),(2*M_PI))-M_PI;
                    q_out(2) = fmod(((p3-position_calibrations[2])/RGM_LPR*(2*M_PI)+M_PI),(2*M_PI))-M_PI;
                    q_out(3) = fmod(((p4-position_calibrations[3])/RGM_LPR*(2*M_PI)+M_PI),(2*M_PI))-M_PI;
                    q_out(4) = fmod(((p5-position_calibrations[4])/RGM_LPR*(2*M_PI)+M_PI),(2*M_PI))-M_PI;
                    q_out(5) = fmod(((p6-position_calibrations[5])/RGM_LPR*(2*M_PI)+M_PI),(2*M_PI))-M_PI;

                    return 0;

                }
                                

            
           inline int pos_trans(const JntArray& q_in,int& p1,int& p2,int& p3,
                                                int& p4,int& p5,int& p6)  const
                {
                    if(q_in.rows()!=6)
                        return(E_SIZE_MISMATCH);

                    p1 = int( RGM_LPR*q_in(0)/(2*M_PI) + fmod(position_calibrations[0],(RGM_LPR)) );
                    p2 = int( RGM_LPR*q_in(1)/(2*M_PI) + fmod(position_calibrations[1],(RGM_LPR)) );
                    p3 = int( RGM_LPR*q_in(2)/(2*M_PI) + fmod(position_calibrations[2],(RGM_LPR)) );
                    p4 = int( RGM_LPR*q_in(3)/(2*M_PI) + fmod(position_calibrations[3],(RGM_LPR)) );
                    p5 = int( RGM_LPR*q_in(4)/(2*M_PI) + fmod(position_calibrations[4],(RGM_LPR)) );
                    p6 = int( RGM_LPR*q_in(5)/(2*M_PI) + fmod(position_calibrations[5],(RGM_LPR)) );
                   
                   return 0;
                        
                }


        
    };


};
#endif