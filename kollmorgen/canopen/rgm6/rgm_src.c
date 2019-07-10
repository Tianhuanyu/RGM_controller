#include "rgm_src.h"
#include "rgmtcp.h"


LinkQueue target_queue, actual_queue;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;

int control_mode = 0;
// const char delims1[] = {";"};
// const char delims2[] = {","};
int newstate = 0;

char* substring(char* ch,int pos,int length)  
{  
    char* pch=ch;
    //int length = sizeof(ch);

    char* subch=(char*)calloc(sizeof(char),length+1);  

    int i;  

    pch=pch+pos;  

    for(i=0;i<length;i++)  
    {  
        subch[i]=*(pch++);    
    }  
    subch[length]='\0';//加上字符串结束符。  
    return subch;       //返回分配的字符数组地址。  
}

void Enter_queue_Mutex(void)
{
	if(pthread_mutex_lock(&mutex) != 0)
        printf("Mutex lock error");
}

void Leave_queue_Mutex(void)
{
	if(pthread_mutex_unlock(&mutex) != 0)
        printf("Mutex unlock error");
}
//Initialization
void initQueue(LinkQueue *queue)
{
    queue->front = queue->rear = (Queue)calloc(1,sizeof(Node));

    if(NULL == queue->front){
        //exit(-1);// wait for done
        printf("NO memory to malloc \n");
        exit(-1);
    }

    queue->front->next = NULL;
}
//Is the queue empty?
bool isEmpty(LinkQueue queue)
{
    return queue.rear == queue.front ? true:false;
}
//insert queue (FIFO)
/*isTarget = 1*/
void insertQueue6(LinkQueue *queue)
{
    Queue q = (Queue)calloc(1,sizeof(Node));

    if (NULL==q){
        //exit(-1);
        printf("\n\nNo memoray to save\n\n");
        return;
    }
    
    if(isinit > 0){
    
    q->Position[0] = ActualPosition1;
    q->Position[1] = ActualPosition2;
    q->Position[2] = ActualPosition3;
    q->Position[3] = ActualPosition4;
    q->Position[4] = ActualPosition5;
    q->Position[5] = ActualPosition6;
   
    q->Velocity[0] = ActualVelocity1;
    q->Velocity[1] = ActualVelocity2;
    q->Velocity[2] = ActualVelocity3;
    q->Velocity[3] = ActualVelocity4;
    q->Velocity[4] = ActualVelocity5;
    q->Velocity[5] = ActualVelocity6;

    q->Torque[0] = ActualTorque1;
    q->Torque[1] = ActualTorque2;
    q->Torque[2] = ActualTorque3;
    q->Torque[3] = ActualTorque4;
    q->Torque[4] = ActualTorque5;
    q->Torque[5] = ActualTorque6;

    q->OperationMode[0] = OperationMode1;
    q->OperationMode[1] = OperationMode2;
    q->OperationMode[2] = OperationMode3;
    q->OperationMode[3] = OperationMode4;
    q->OperationMode[4] = OperationMode5;
    q->OperationMode[5] = OperationMode6;
    //printf("************************************\n**********INIT***********\n*******************\n");

    }
    q->next = NULL;

    //change the queue end
    queue ->rear ->next = q;
    queue -> rear = q;

  
}
void QueuePrint(LinkQueue *queue)
{
    int flag = isEmpty(*queue);
    printf("Here %d  %d\n",queue->rear->Position ,flag);
}
//delete Queue
void deleteQueue(LinkQueue *queue)
{
    Queue q = NULL;
    if(!isEmpty(*queue)){
        q = queue->front->next;
        queue->front->next = q->next;

        if (queue->rear == q){
            queue->rear = queue->front;
        }
        free(q);
    }
}
//delete ALL
void destoryQueue(LinkQueue *queue)
{
    while (queue->front != NULL){
        queue->rear = queue->front->next;
        free(queue->front);
        queue->front = queue->rear;
    }

    puts("Success in destory!");
}
// read queue and Add new target node
void read_queue(LinkQueue *queue){
    /*add judgement*/
    
    if(!isEmpty(*queue)){

        //队列队头的模式 进行判断模式 如果队头不为0 则需要进行模式切换
        if(queue->front->next->OperationMode[0] != 0)
        {
            //brief : select operation mode in queue
            switch(queue->front->next->OperationMode[0]){
                // mode changed
                case PROFILE_POSITION_MODE:
                    //printf("TARGET SET FINISHED!!\n");
                    //printf("TargetPosition1 before= %d",TargetPosition1);
                    TargetPosition1 = ActualPosition1;
                    TargetPosition2 = ActualPosition2;
                    TargetPosition3 = ActualPosition3;
                    TargetPosition4 = ActualPosition4;
                    TargetPosition5 = ActualPosition5;
                    TargetPosition6 = ActualPosition6;
                    //printf("TargetPosition1 after= %d",TargetPosition1);
                    break;
                
                case PROFILE_VELOCITY_MODE:
                    // printf("PROFILE_VELOCITY_MODE - status changed");
                    // printf("TargetPosition1 before= %d",queue->front->next->Velocity[0]);
                    TargetVelocity1 = 0;
                    TargetVelocity2 = 0;
                    TargetVelocity3 = 0;
                    TargetVelocity4 = 0;
                    TargetVelocity5 = 0;
                    TargetVelocity6 = 0;
                   
                    break;
                
                case PROFILE_TORQUE_MODE:
                    printf("NOT support yet!!");
                    break;

                case INTERPOLATED_POSITION_MODE: 
                    printf("NOT support!!!\n");
                    break;
                }
            
            OperationMode1 = queue->front->next->OperationMode[0];
            OperationMode2 = queue->front->next->OperationMode[1];
            OperationMode3 = queue->front->next->OperationMode[2];
            OperationMode4 = queue->front->next->OperationMode[3];
            OperationMode5 = queue->front->next->OperationMode[4];
            OperationMode6 = queue->front->next->OperationMode[5];
            //printf("\nTargetPosition1 before= %d",TargetPosition1);
        /*Master's operation_mode decide on slave 1*/

        // if the state is changed? finish change
          
            printf("\nMaster: Have changed!!\n");
          
        }
        else{
            // 模式为0 则进行数值切换，也就是数值指令
            printf("TargetPosition1 after= %d\n",TargetPosition1);
            switch (OperationMode1)
            {
                case PROFILE_POSITION_MODE:
                    //printf("123TargetPosition1 after= %d\n",TargetPosition1);
                    if(queue->front->next->Position[0] != 0)
                        
                    {
                        //not if
                        TargetPosition1 = queue->front->next->Position[0];
                        TargetPosition2 = queue->front->next->Position[1];
                        TargetPosition3 = queue->front->next->Position[2];
                        TargetPosition4 = queue->front->next->Position[3];
                        TargetPosition5 = queue->front->next->Position[4];
                        TargetPosition6 = queue->front->next->Position[5];
                        printf("\nATTENTION\nOperation mode,TargetPosition1,queue->front->next->Position[0]= %x %x %x\n",queue->front->next->OperationMode[0],TargetPosition1,queue->front->next->Position[0]);
                    }
                    break;
                
                case PROFILE_VELOCITY_MODE :

                    TargetVelocity1 = queue->front->next->Velocity[0];
                    TargetVelocity2 = queue->front->next->Velocity[1];
                    TargetVelocity3 = queue->front->next->Velocity[2];
                    TargetVelocity4 = queue->front->next->Velocity[3];
                    TargetVelocity5 = queue->front->next->Velocity[4];
                    TargetVelocity6 = queue->front->next->Velocity[5];
                    break;

                case PROFILE_TORQUE_MODE :

                    TargetTorque1 = queue->front->next->Torque[0];
                    TargetTorque2 = queue->front->next->Torque[1];
                    TargetTorque3 = queue->front->next->Torque[2];
                    TargetTorque4 = queue->front->next->Torque[3];
                    TargetTorque5 = queue->front->next->Torque[4];
                    TargetTorque6 = queue->front->next->Torque[5];
                    break;

            }
        }   
        
    }
 
}

//canopen queue process
//在程序中的canopen队列处理函数，负责把AP AV发送到上位机
void canopen_queue(void){
    int numt = 0;
    Enter_queue_Mutex();
    
    // actual Position  机械臂传感数据从CANOPEN 上获取
    insertQueue6(&actual_queue);  
    // Target Position  机械臂传感数据保存在队列里的数据在此处执行
    read_queue(&target_queue);
    deleteQueue(&target_queue);

    tcp_write(&actual_queue);  
    deleteQueue(&actual_queue);
    
  
    
    Leave_queue_Mutex();
    
    
    if((numt=send(sock_fd,sendbuf,strlen(sendbuf),MSG_NOSIGNAL)) < 0){
                printf("ERROR:Fail to send string\n");
                close(sock_fd);
                //exit(1);
                tcp_connected = 0;
                return;
                }
}







