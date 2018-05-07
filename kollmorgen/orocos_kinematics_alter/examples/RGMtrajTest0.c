
#include "RGMtrajControl.hpp"
#include <stddef.h>
#include <stdio.h>


int main(int argc , char** argv){

    LinkQueue_pvt pQueue;
    
    Node_pvt* qNode = NULL;
    int ret = 0;
    int ret1 = -1;
    int vel[6] = {0,0,0,0,5,0};
    int handle = -1;
    //KDL::RGMtrajCtrl* p = NULL;

    trajCtrlinit_wrap(&handle,pQueue,3000);
    //RGMtrajCtrl trajSolver(pQueue,3000);
    ret = calcuVelocity_wrap(handle,vel,vel+1,vel+2,vel+3,vel+4,vel+5);

    ret1 = get_pos_wrap(handle,1,1,1,1,10,1);
    trajCtrlDele_wrap(&handle);
    //ret1 = get_pos_wrap(0,1,1,1,1,10,1);

    printf("ret,ret1 = %d,%d",ret,ret1);
    //std::cout<<ret<<"\n"<<ret1<<"\n"<<std::endl;



    

}