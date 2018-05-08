#include "RGMtrajControl.hpp"

namespace KDL {
 RGMtrajCtrl::RGMtrajCtrl(LinkQueue_pvt& pQueue,int _max_vel):
    Position(6),
    Velocity(6),
    Torque(6),
    step(0),
    max_step(-1),
    offset(6),
    pRef(6),
    init(0),
    lastActualPosition(6),
    TarPosi_thisStep(6),
    TrajSolver1(_max_vel),
    TrajSolver2(_max_vel),
    TrajSolver3(_max_vel),
    TrajSolver4(_max_vel),
    TrajSolver5(_max_vel),
    TrajSolver6(_max_vel),
    Error_p(6)   
    {
        pNode_now = pQueue.front;
        //TrajSolver1(_max_vel);
        //JntArray pRef(6);
    };

    void RGMtrajCtrl::SetTrajProfile(){
        int num = 0;
        for(num=0;num<6;num++){
            Position(num) = pNode_now->Position[num];
        }

        if(max_step%5 == 0)
            max_step = (pNode_now->Time)/5;
        else
            max_step = (pNode_now->Time)/5 + 1; 
        

        TrajSolver1.SetProfileDuration(offset(0),Position(0),max_step*5);
        TrajSolver2.SetProfileDuration(offset(1),Position(1),max_step*5);
        TrajSolver3.SetProfileDuration(offset(2),Position(2),max_step*5);
        TrajSolver4.SetProfileDuration(offset(3),Position(3),max_step*5);
        TrajSolver5.SetProfileDuration(offset(4),Position(4),max_step*5);
        TrajSolver6.SetProfileDuration(offset(5),Position(5),max_step*5);

        std::cout<<"offset: "<<offset(0)<<" Position:"<<Position(0)<<"\n"<<std::endl;
    }
    JntArray RGMtrajCtrl::RGM_pos(int step){
        double vp_time = step*5;
        JntArray pos_rec(6);
        //vel_rec(6);
        
        pos_rec(0) = TrajSolver1.Pos(vp_time);
        pos_rec(1) = TrajSolver2.Pos(vp_time);
        pos_rec(2) = TrajSolver3.Pos(vp_time);
        pos_rec(3) = TrajSolver4.Pos(vp_time);
        pos_rec(4) = TrajSolver5.Pos(vp_time);
        pos_rec(5) = TrajSolver6.Pos(vp_time);

        return pos_rec;

    }

    JntArray RGMtrajCtrl::RGM_vel(int step){
        // parameter time in velocity profile class
        double vp_time = step*5;
        JntArray vel_rec(6);
        //vel_rec(6);
        
        vel_rec(0) = TrajSolver1.Vel(vp_time);
        vel_rec(1) = TrajSolver2.Vel(vp_time);
        vel_rec(2) = TrajSolver3.Vel(vp_time);
        vel_rec(3) = TrajSolver4.Vel(vp_time);
        vel_rec(4) = TrajSolver5.Vel(vp_time);
        vel_rec(5) = TrajSolver6.Vel(vp_time);

        return vel_rec;        
    }
    /*
    *\brief : get actual position for reference
    * input : ActualPosition 1 2 3 4 5 6(Position())
    * output : offset()
    * return : -1 => finished
    *           0 => change trajProfile
    */

    // int RGMtrajCtrl::get_pos(int p1,int p2,int p3,int p4,int p5,int p6){
        
    //     // if(init == 0)
    //     // {
    //     //     pRef(0) = p1;
    //     //     pRef(1) = p2;
    //     //     pRef(2) = p3;
    //     //     pRef(3) = p4;
    //     //     pRef(4) = p5;
    //     //     pRef(5) = p6;
    //     //     init =1;
    //     // }
        
    //     // offset(0) = Position(0) - p1 +pRef(0);
    //     // offset(1) = Position(1) - p2 +pRef(1);
    //     // offset(2) = Position(2) - p3 +pRef(2);
    //     // offset(3) = Position(3) - p4 +pRef(3);
    //     // offset(4) = Position(4) - p5 +pRef(4);
    //     // offset(5) = Position(5) - p6 +pRef(5);

    //     // pRef(0) = p1;
    //     // pRef(1) = p2;
    //     // pRef(2) = p3;
    //     // pRef(3) = p4;
    //     // pRef(4) = p5;
    //     // pRef(5) = p6;

    //      offset(0) = 0;
    //     offset(1) = 0;
    //     offset(2) = 0;
    //     offset(3) = 0;
    //     offset(4) = 0;
    //     offset(5) = 0;

    //     //std::cout<<"offset:"<<offset(4)<<"\n"<<std::endl;
    //     if(pNode_now != NULL){
    //         SetTrajProfile();
    //         return 0;
    //     }
    //     else{
    //         //std::cout<<"error no data\n"<<std::endl;
    //         return -1;}

    // }
    /*
    *\ brief: state change
    *  input: ActualPosition，flag returned by
    * output: Status (e.g. error in position ) ep
    * 
    * return : 0 : get_pos or change segement
    *          -1: finished!
    * 
    * */

    int RGMtrajCtrl::get_pos(const int &p1,const int &p2,const int &p3,const int &p4,const int &p5,const int &p6,const int &flag){
        //initial jntarry
        SetToZero(Error_p);
        JntArray target_pos(6);
        
        if(init == 0)
        {
            pRef(0) = p1;
            pRef(1) = p2;
            pRef(2) = p3;
            pRef(3) = p4;
            pRef(4) = p5;
            pRef(5) = p6;
            init =1;

            pNode_now = pNode_now->next;
            if(pNode_now != NULL)
                SetTrajProfile();
        }
        else{
                if(flag == 0){
                    target_pos = RGM_pos(step);

                    Error_p(0) = p1 -pRef(0) -target_pos(0);
                    Error_p(1) = p2 -pRef(1) -target_pos(1);
                    Error_p(2) = p3 -pRef(2) -target_pos(2);
                    Error_p(3) = p4 -pRef(3) -target_pos(3);
                    Error_p(4) = p5 -pRef(4) -target_pos(4);
                    Error_p(5) = p6 -pRef(5) -target_pos(5);

                    return 0;

                }
                else{
                    /*change reference position to actual position*/
                    
                    offset(0) = -Position(0) + p1 -pRef(0);
                    offset(1) = -Position(1) + p2 -pRef(1);
                    offset(2) = -Position(2) + p3 -pRef(2);
                    offset(3) = -Position(3) + p4 -pRef(3);
                    offset(4) = -Position(4) + p5 -pRef(4);
                    offset(5) = -Position(5) + p6 -pRef(5);

                    pRef(0) = p1;
                    pRef(1) = p2;
                    pRef(2) = p3;
                    pRef(3) = p4;
                    pRef(4) = p5;
                    pRef(5) = p6;

                        if(pNode_now != NULL){
                                SetTrajProfile();
                            return 0;
                        }
                        else{
                            std::cout<<"error no data\n"<<std::endl;
                            return -1;
                            }

                }

        }
        

    }

    /*
    *\ brief: calculate velocity 
    *  input:static step in class
    *  output: TargetVelocity1 2 3 4 5 6
    *  return: 0 => step change (segment not change)
    *          1 => step = 0  segment change
    *          -1 => finished
    * 
    */

    int RGMtrajCtrl::calcuVelocity(int& v1,int& v2,int& v3,int& v4,int& v5,int& v6) {
        
        const int kp = 0;
        if(step < max_step){
            JntArray Vel_fb(6);
            int i = 0;
            
            Vel_fb.data = RGM_vel(step).data + kp*Error_p.data;
            //Vel_fb.data = RGM_vel(step).data;
            
            step++;

            v1 = (0x15)*(0x3E8)*(int32_t)Vel_fb(0);
            v2 = (0x15)*(0x3E8)*(int32_t)Vel_fb(1);
            v3 = (0x15)*(0x3E8)*(int32_t)Vel_fb(2);
            v4 = (0x15)*(0x3E8)*(int32_t)Vel_fb(3);
            v5 = (0x15)*(0x3E8)*(int32_t)Vel_fb(4);
            v6 = (0x15)*(0x3E8)*(int32_t)Vel_fb(5);

            

            return 0; 
        }
        else{

            step = 0;
            if(pNode_now == NULL){
                    std::cout<<"There is no data"<<"\n"<<std::endl;
                    
                    v1 = 0;
                    v2 = 0;
                    v3 = 0;
                    v4 = 0;
                    v5 = 0;
                    v6 = 0;
                    
                    return -1;
                }
            else{
                    std::cout<<"pNode_now->Count = "<<pNode_now->Count<<"\n"<<std::endl;
                    
                    std::cout<<"v1,v2,v3 step, max_step= "<<v1<<" "<<v2
                    <<" "<<v3<<" "<<step<<" "<<max_step<<"\n"<<std::endl;
                    pNode_now = pNode_now->next;
                    return 1;
                    //RGMtrajCtrl::SetTrajProfile();
                }
        }


    }
    
    RGMtrajCtrl::~RGMtrajCtrl(){}






}

/*
    *\brief : init a traj class
    * 
    */

void trajCtrlinit_wrap(int* handle,LinkQueue_pvt pQueue,int _max_vel){
    //KDL::RGMtrajCtrl trajSolver(pQueue,_max_vel);
    //return p->RGMtrajCtrl(pQueue,_max_vel);
    RGMtrajCtrl * p = NULL;
    p = new RGMtrajCtrl(pQueue,_max_vel);
    
    RGMtrajCtrl_Vector.push_back(p);
    //RGMtrajCtrl_Vector[0]->RGMtrajCtrl(pQueue,_max_vel);

    *handle = 0;

    return;
}

int get_pos_wrap(int handle,int p1,int p2,int p3,int p4,int p5,int p6,int flag){
    // int ret = -1;
    // ret = trajSolver.get_pos(p1,p2,p3,p4,p5,p6);
    // return ret;
    //return p->get_pos(p1,p2,p3,p4,p5,p6);
    return RGMtrajCtrl_Vector[handle]->get_pos(p1,p2,p3,p4,p5,p6,flag);
}

int calcuVelocity_wrap(int handle,int* v1,int* v2,int* v3,int* v4,int* v5,int* v6){
    //int ret = -1;
    // ret = trajSolver.calcuVelocity(v1,v2,v3,v4,v5,v6);
    // return ret;
    return RGMtrajCtrl_Vector[handle]->calcuVelocity(*v1,*v2,*v3,*v4,*v5,*v6);
}


/*
    *\brief : delet a traj class
    * 
    */
void trajCtrlDele_wrap(int* handle){
    //RGMtrajCtrl_Vector[*handle]->~RGMtrajCtrl();
    RGMtrajCtrl_Vector.clear();
    *handle = -1;
}




