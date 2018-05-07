#include "pvt_src.h"

// a var
LinkQueue_pvt pvt_command_queue;

pthread_mutex_t mutex_pvt = PTHREAD_MUTEX_INITIALIZER;

 //const char delims0[] = "\n";
 const char delims01[] = ";";
 const char delims02[] = ",";
 int position_sum = 0;

 //const char end_request[] = "PVT_FINISHED\n";
int pvt_queue_num = 0;
int pvt_max_count = 0;
int pvt_end = 0;
int pvt_count = 0;
int num_add_pvt = 15;

INTEGER32 v_temp[6] = {0,0,0,0,0,0};
Node_pvt pvt_memory = {
	{0,0,0,0,0,0},
    {0,0,0,0,0,0},
    0,
    0,
    NULL
};

void Enter_pvtqueue_Mutex(void)
{
	if(pthread_mutex_lock(&mutex_pvt) != 0)
        printf("Mutex_pvt lock error");
}

void Leave_pvtqueue_Mutex(void)
{
	if(pthread_mutex_unlock(&mutex_pvt) != 0)
        printf("Mutex_pvt unlock error");
}

//Initialization
void init_pvtqueue(LinkQueue_pvt *queue)
{
    queue->front = queue->rear = (Queue_pvt)calloc(1,sizeof(Node_pvt));

    if(NULL == queue->front){
        //exit(-1);// wait for done
        printf("NO memory to malloc \n");
        exit(-1);
    }

    queue->front->next = NULL;
    printf("Success init pvt_queue\n");
    return;
}

//Is the queue empty?
bool is_empty(LinkQueue_pvt queue)
{
    return queue.rear == queue.front ? true:false;
}

void destory_pvt_queue(LinkQueue_pvt *queue)
{
    while (queue->front != NULL){
        queue->rear = queue->front->next;
        free(queue->front);
        queue->front = queue->rear;
    }

    puts("Success in destory!");
}

//read (sub)receive buff and insert pvt queue 
int insert_pvt_queue(LinkQueue_pvt *queue,int* count,char* sub_receivebuf)
{
    // const sub_receivebuf_long
    char* result = NULL;
    char* sub_result = NULL;
    int num = 0;
    char sub_buff_p[100];
    char sub_buff_v[100];
    char sub_buff_tm[100];
    char sub_buff[100];
    char *key,*keyp,*keyv,*keyt,*keys;
    
    /*insert a queue*/
    Queue_pvt q = (Queue_pvt)calloc(1,sizeof(Node_pvt));

    if (NULL==q){
        //exit(-1);
        printf("No memoray to save");
        return -1;
    }
    
    //eprintf("run to here1 %s\n",sub_receivebuf);
    strcpy(sub_buff,sub_receivebuf);
    result = strtok_r(sub_buff,delims01,&key);
    while(result != NULL){
        //printf("result = %d\n",*result);
        switch(*result){
            //num = 0;
            case 80:
                //position
                //if(*(result+1) == 91)
                strcpy(sub_buff_p,result);
                printf("sub_buff_p = %s\n",sub_buff_p);
                break;
            case 86:
                //velocity
                //if(*(result+1) == 91)
                strcpy(sub_buff_v,result);           
                //printf("sub_buff_v = %s\n",sub_buff_v);
                break;
            case 84:
                //time
                //if(*(result+1) == 91)
                    strcpy(sub_buff_tm,result);
                break;
            default :
                free(q);
                //printf("ERROR MESSAGE!!!\n");
                return 1;
            
        }
        result = strtok_r(NULL,delims01,&key);
    }
    
    //eprintf("run to here0\n");
        //position
        num = 0;
        sub_result = strtok_r(sub_buff_p,delims02,&keyp);
        while((sub_result != NULL)&&(num<6)){
            if(num ==0){
                //printf("sub_result1 = %s\n",sub_result);
                sub_result = substring(sub_result,2,9);
            }

            q->Position[num] = (INTEGER32)strtol(sub_result,NULL,16);
            printf("Position[%d] = %x\n",num,q->Position[num]); 
            sub_result = strtok_r(NULL,delims02,&keyp);
            num = num+1 ;
        }
        //velocity
        // num = 0;
        // sub_result = strtok(sub_buff_v,delims02);
        // while((sub_result != NULL)&&(num<6)){
        //     if(num ==0){
        //         sub_result = substring(sub_result,2,9);
        //     }
        //     printf("\nsub_result_velocity = %c\n",sub_result);
        //     q->Velocity[num] = (INTEGER16)strtol(sub_result,NULL,16); 
        //     sub_result = strtok(NULL,delims02);
        //     num = num+1 ;
        // }
        //time
        num = 0;
        sub_result = strtok_r(sub_buff_tm,delims02,&keyt);
        while((sub_result != NULL)&&(num<6)){
            if(num ==0){
                sub_result = substring(sub_result,2,4);
            }

            q->Time = (INTEGER16)strtol(sub_result,NULL,16); 
            sub_result = strtok_r(NULL,delims02,&keyp);
            num = num+1 ;
        }
        q->Count = (INTEGER16)(*count);

    //change the queue end
    q->next = NULL;

    queue ->rear ->next = q;
    queue -> rear = q;
    //*count = *count+1;
    return 0;
    
}

// /*change_1995*/
// int read_pvt_queue(LinkQueue_pvt *queue,int* step){

//     int numt = 0;
//     int max_step = 0;

//      //renew targetvelocity
// 		if(!is_empty(*queue)){
//             if((queue->front->next->Time) > 5)
//                 max_step = (queue->front->next->Time)/5;
//             else
//                 max_step = 1;
			
//             if(queue->front->next->Count == 0){
//                 position_sum = ActualPosition2;
//             }



// 			PT_mode((queue->front->next), //point to pvt_command_now
// 					v_temp,
// 					6);
            
            
//             TargetVelocity1 = v_temp[0];
//             TargetVelocity2 = v_temp[1];
//             TargetVelocity3 = v_temp[2];
//             TargetVelocity4 = v_temp[3];
//             TargetVelocity5 = v_temp[4];
//             TargetVelocity6 = v_temp[5];
            


//             if((max_step-1) <= *(step)){
			
//                 delete_pvt_queue(queue);
//                 *step = 0;
//                     if(is_empty(*queue)){

//                             control_mode = COMMAND;

//                             TargetVelocity1 = 0;
//                             TargetVelocity2 = 0;
//                             TargetVelocity3 = 0;
//                             TargetVelocity4 = 0;
//                             TargetVelocity5 = 0;
//                             TargetVelocity6 = 0;
//                             position_sum = ActualPosition2 - position_sum;

//                             printf("position_sum = %x\n",position_sum);
//                             //sleep(0.2);
//                             if((numt=send(sock_fd,end_request,strlen(end_request),MSG_NOSIGNAL)) < 0){
//                                 printf("ERROR:Fail to send string\n");
//                                 close(sock_fd);
//                                 //exit(1);
//                                 tcp_connected = 0;
//                                 //return 0;
//                             }
                        
//                             return 0;
//                         }
//                     else{
//                         printf("Count = %d\nTime = %d\nposition = %d\n",queue->front->next->Count,(queue->front->next->Time),(queue->front->next->Position[1]));
            
//                         return 1;
//                     }
            
//             }
//             else{
//                 *(step) = *(step) +1;
//             }

            
// 		}
//         return -1;

// }

// //delete Queue
// void delete_pvt_queue(LinkQueue_pvt *queue)
// {
//     Queue_pvt q = NULL;
//     if(!is_empty(*queue)){
//         q = queue->front->next;
//         queue->front->next = q->next;

//         if (queue->rear == q){
//             queue->rear = queue->front;
//         }
//         free(q);
//     }
// }

//tcp queue process
// void tcp_pvt_queue(void){
    
//     //Enter_queue_Mutex();
//     //netdata=>myqueue target queue+1
//     printf("\n ************recieve from pc pvt************\n");
//     printf("pvt_count = %d\n",pvt_count);
//     //insert_pvt_queue(&pvt_command_queue,&pvt_count);
//     //printf("%s \n",receivebuf);
//     printf("\n ************recieve from pc pvt************\n");
    
//      //Leave_queue_Mutex();
// }

//canopen queue process
// void canopen_pvt_queue(void){
//     //control_mode = 1;
//     char sendbuf_pvt[]="pvt_request\n";
//     int i = 0;
//     int numres = 0;
//     if(PVTalert1 == 0xA){
//      if((numres=send(sock_fd,sendbuf_pvt,strlen(sendbuf_pvt),0)) < -1){
// 				printf("ERROR:Fail to send string\n");
// 				close(sock_fd);
// 				//exit(1);
//                  = 0;
//                 return;
//         		}
//     num_add_pvt = 5;
//     }
//     //slave pvt queue needed to be filled
//     //
//     if(num_add_pvt > 0){
       
//             Enter_queue_Mutex();    
//         //myqueue=>netdata actual queue-1
//             read_pvt_queue(&pvt_command_queue);
//             //printf("PVTbuff = %x \n",PVTbuff1);
//             delete_pvt_queue(&pvt_command_queue);
//             Leave_queue_Mutex();
//             //num_add_pvt--;
        
//     }
//     PVTalert1 = 0;				
// }