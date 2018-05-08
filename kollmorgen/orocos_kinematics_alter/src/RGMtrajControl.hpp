#ifndef KDL_RGM_TRAJ_CTRL
#define KDL_RGM_TRAJ_CTRL


#ifdef __cplusplus
extern "C" {
    #include <stdint.h>
        }
struct Node_pvt{
    
    int32_t Position[6];
    int32_t Velocity[6];
    int16_t  Time;
    int16_t  Count;

    struct Node_pvt *next;
};
typedef Node_pvt* Queue_pvt;

struct LinkQueue_pvt{
    Queue_pvt front;
    Queue_pvt rear;
};


#include "chain.hpp"
#include <Eigen/Dense>
#include "RGMcontrol.hpp"
#include "velocityprofile_rect.hpp"
//#include <stdio.h>

namespace KDL{

    //queue_pointer* queue;
    /*
    *\brief introduce a velocity profile for RGM6
    * trajactory control include pt\pvt\interpolation method
    */

    class RGMtrajCtrl : public KDL::RGMctrl
    {
    public:
    RGMtrajCtrl(LinkQueue_pvt& pQueue,int _max_vel);
    /*
    *\* output data
    * 
    */
    JntArray Position;
    JntArray Velocity;
    JntArray Torque;
    JntArray pRef;
   
    /*
    *  \brief: step for finish a little 
    * segment from phy
    * 
    */

    int max_step;
    /*
    * \brief : step for now(inherent time)
    */
    int step;

    int init;
    //VelocityProfile_Dirac trajSolver profile;

    void SetTrajProfile();


    //velocity get method
    JntArray RGM_vel(int step);
    JntArray RGM_pos(int step);

    //dataflow control function
    /*state change*/
    virtual int get_pos(const int &p1,const int &p2,const int &p3,const int &p4,const int &p5,const int &p6,const int &flag);
            
    virtual int get_vel(const int &v1,const int &v2,const int &v3,const int &v4,const int &v5,const int &v6,const int &flag){return 0;};
    
    virtual int get_tor(const int &t1,const int &t2,const int &t3,const int &t4,const int &t5,const int &t6,const int &flag){return 0;};
    
    
    
    
    /*output*/
    virtual int calcuPosition(int* p1,int* p2,int* p3,int* p4,int* p5,int* p6){return 0;};

    virtual int calcuVelocity(int& v1,int& v2,int& v3,int& v4,int& v5,int& v6);

    virtual int calcuTorque(int* t1,int* t2,int* t3,int* t4,int* t5,int* t6){return 0;};
    //a traj solver need tobe added here
    VelocityProfile_Rectangular TrajSolver1;
    VelocityProfile_Rectangular TrajSolver2;
    VelocityProfile_Rectangular TrajSolver3;
    VelocityProfile_Rectangular TrajSolver4;
    VelocityProfile_Rectangular TrajSolver5;
    VelocityProfile_Rectangular TrajSolver6;

    virtual ~RGMtrajCtrl();
    private:
    /*
    *\brief :segment pointer for phy's queue
    * 
    */
    Node_pvt* pNode_now;
    /*
    * \brief : offset at this time
    * offset = last disire - last actual positon
    * offset need to be added on pos2
    */
    JntArray offset;
    /*
    *\ brief : last actual 
    * 
    */
    JntArray lastActualPosition;
    /*
    *\brief :target position in this step
    * 
    */
    JntArray TarPosi_thisStep;
    /**
     *\brief :Position error
     *        position error feedback parameter in calVelocity
    */
    JntArray Error_p; 
    };
};


#include <vector>
typedef KDL::RGMtrajCtrl RGMtrajCtrl;

static std::vector<RGMtrajCtrl *> RGMtrajCtrl_Vector;


#endif



#ifdef __cplusplus
extern "C"{
#endif


/*
    *\brief : init a traj class
    * 
    */
void trajCtrlinit_wrap(int* handle,LinkQueue_pvt pQueue,int _max_vel);

/*
    *\brief : get actual position for reference
    * input : ActualPosition 1 2 3 4 5 6(Position())
    * output : offset()
    * return : -1 => finished
    *           0 => change trajProfile
    */

int get_pos_wrap(int handle,int p1,int p2,int p3,int p4,int p5,int p6,int flag);

   /*
    *\ brief: calculate velocity 
    *  input:static step in class
    *  output: TargetVelocity1 2 3 4 5 6
    *  return: 0 => step change (segment not change)
    *          1 => step = 0  segment change
    *          -1 => finished
    * 
    */

int calcuVelocity_wrap(int handle,int* v1,int* v2,int* v3,int* v4,int* v5,int* v6);

/*
    *\brief : delet a traj class
    * 
    */

void trajCtrlDele_wrap(int* handle);

#ifdef __cplusplus
}
#endif

#endif