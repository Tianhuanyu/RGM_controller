#ifndef _PVT_SRC_H
#define _PVT_SRC_H


#ifdef __cplusplus
extern "C" {
#endif


#include "rgm6.h"
#include <stdbool.h>
//#include <linux/spinlock.h>
// #include <linux/jiffies.h>
// #include <linux/timer.h>
// #include <linux/errno.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include "rgm_src.h"

#define AP1ZERO 76744
#define AP2ZERO 265504
#define AP3ZERO 251134
#define AP4ZERO 270630
#define AP5ZERO 256535.0
#define AP6ZERO 92958.0
#define DRIVER_COUNT 524288.00

typedef struct Nodepvt{
    
    INTEGER32 Position[6];
    INTEGER32 Velocity[6];
    FRAME tcp_frame;
    TWIST tcp_twist;
    CONTROL_MODE control_mode;
    INTEGER16  Time;
    INTEGER16  Count;

    struct Nodepvt *next;
}Node_pvt, *Queue_pvt;

//queue of struct & nest
typedef struct{
    Queue_pvt front;
    Queue_pvt rear;
}LinkQueue_pvt;

//#include "ptmode.hpp"
#include "RGMtrajControl.hpp"
// #include "chainfksolverpos_rgm.hpp"

void Enter_pvtqueue_Mutex(void);

void Leave_pvtqueue_Mutex(void);

void init_pvtqueue(LinkQueue_pvt *queue);

void destory_pvt_queue(LinkQueue_pvt *queue);

bool is_empty(LinkQueue_pvt queue);

int insert_pvt_queue(LinkQueue_pvt *queue,int* count,char* sub_receivebuf);

//int read_pvt_queue(LinkQueue_pvt *queue,int* step);
void deletepvtQueue(LinkQueue_pvt *queue);

void delete_pvt_queue(LinkQueue_pvt *queue);

//void tcp_pvt_queue(void);

void canopen_pvt_queue(void);

extern LinkQueue_pvt pvt_command_queue;

//brief: number in pvt queue
extern int pvt_queue_num;
//brief: the counts of pvt point
extern int pvt_count;
//brief: pvt points needed to write in buffer
extern int num_add_pvt;
//brief: end point num of pvt
extern int pvt_max_count;

extern int pvt_end;


#ifdef __cplusplus
}
#endif



#endif