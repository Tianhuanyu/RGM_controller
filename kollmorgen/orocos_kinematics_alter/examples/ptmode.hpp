#ifndef __PT_MODE_H

#define __PT_MODE_H


#ifdef __cplusplus

extern "C" {

#endif


#include "rgm6.h"
#define SPEED_LIMIT 300000
#include <stdbool.h>
#include <errno.h>
#include <unistd.h>
#include <pthread.h>
#include "rgm_src.h"
#include "pvt_src.h"


/*brief : pt alogrithm */
/*input : 1.pvt_struct past  2.pvt_struct now*/
/*output: velocity[6] */
void PT_mode(Queue_pvt now,int* velocity,int length);
// void inti()
// {
//     control aaaa;
// }
// void add()
// {
//     aaaaa.add();
// }
// double getVel(int getVelpostion)
// {
//     return aaaa.getVel(postion);
// }

// #ifdef __cplusplus
// }
// #endif


// class control{

// };

#endif