
#ifndef RGM_SRC
#define RGM_SRC

#ifdef __cplusplus
extern "C" {
#endif

#define RGM_NUM 6
#include "rgm6.h"
#include <stdbool.h>
//#include <linux/spinlock.h>
// #include <linux/jiffies.h>
// #include <linux/timer.h>
// #include <linux/errno.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include "rgmtcp.h"
#include "Master.h"
#include "pvt_src.h"

#define TARGET 1
#define ACTUAL 0

#define POSITION 1
#define VELOCITY 2
#define TORQUE 3
#define NO_ACTION 0

#define COMMAND 0
#define MOTION_PREPARE 1
#define MOTION_PLAN 2

typedef struct rgm_motor{

    INTEGER32* target_position;
    INTEGER32* target_velocity;
    INTEGER16* target_torque;

    INTEGER32* actual_position;
    INTEGER32* actual_velocity;
    INTEGER16* actual_torque;

    INTEGER8* operation_mode;

}RGM;
typedef struct Node{
    
    INTEGER32 Position[RGM_NUM];
    INTEGER32 Velocity[RGM_NUM];
    INTEGER16 Torque[RGM_NUM];
    INTEGER8 OperationMode[RGM_NUM];

    struct Node *next;
}Node, *Queue;
//queue of struct & nest
typedef struct{
    Queue front;
    Queue rear;
}LinkQueue;
char* substring(char* ch,int pos,int length);
//Initialization
void initQueue(LinkQueue *queue);
//Is the queue empty?
bool isEmpty(LinkQueue queue);
//insert queue (FIFO)
void insertQueue6(LinkQueue *queue);
//READ QUEUE (FIFO)
void read_queue(LinkQueue *queue);
//delete Queue
void deleteQueue(LinkQueue *queue);
//delete ALL
void destory_queue(LinkQueue *queue);
//debug
void QueuePrint(LinkQueue *queue);
void Enter_queue_Mutex(void);
void Leave_queue_Mutex(void);

void canopen_queue(void);
void command_tcp_queue(char* recbuf);

void tcp_read(LinkQueue *queue,char* recbuf);

void tcp_write(LinkQueue *queue);


extern LinkQueue target_queue, actual_queue;
extern pthread_mutex_t mutex;
extern int newstate;
//depend on mode of control 1=> pvt command
//0=>command
extern int control_mode;
//extern char delims1[];
//extern char delims2[];
#ifdef __cplusplus
}
#endif

#endif
