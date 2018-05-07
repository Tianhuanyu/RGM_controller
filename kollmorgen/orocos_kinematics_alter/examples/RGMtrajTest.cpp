#include "RGMtrajControl.hpp"

#ifdef __cplusplus
extern "C"{
#endif
#include <math.h>
#ifdef __cplusplus
}
#endif


using namespace KDL;
int main(int argc , char** argv){

    queue_pointer pQueue;
    
    Node* qNode = NULL;
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

    std::cout<<ret<<"\n"<<ret1<<"\n"<<std::endl;



    

}

