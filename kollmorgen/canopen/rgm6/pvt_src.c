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
    char sub_buff[100];
    // char sub_buff_v[100];
    // char sub_buff_tm[100];
    char sub_buff_r[100];
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
        switch(*result){
            //num = 0;
            case 80:
                //position
                num = 0;
                strcpy(sub_buff_r,result);
                printf("sub_buff = %s\n",sub_buff_r);

                sub_result = strtok_r(sub_buff_r,delims02,&keyp);
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


                break;
            case 86:
                //velocity
                strcpy(sub_buff_r,result);           
                //velocity
                // num = 0;
                // sub_result = strtok(sub_buff_r,delims02);
                // while((sub_result != NULL)&&(num<6)){
                //     if(num ==0){
                //         sub_result = substring(sub_result,2,9);
                //     }
                //     printf("\nsub_result_velocity = %c\n",sub_result);
                //     q->Velocity[num] = (INTEGER16)strtol(sub_result,NULL,16); 
                //     sub_result = strtok(NULL,delims02);
                //     num = num+1 ;
                // }
                break;
            case 84:
                //time
                //if(*(result+1) == 91)
                strcpy(sub_buff_r,result);

                //time
                num = 0;
                sub_result = strtok_r(sub_buff_r,delims02,&keyt);
                while((sub_result != NULL)&&(num<6)){
                    if(num ==0){
                        sub_result = substring(sub_result,2,4);
                    }

                    q->Time = (INTEGER16)strtol(sub_result,NULL,16); 
                    sub_result = strtok_r(NULL,delims02,&keyp);
                    num = num+1 ;
                }
                
            break;

            default :
                free(q);
                //printf("ERROR MESSAGE!!!\n");
                return 1;
            
        }
        result = strtok_r(NULL,delims01,&key);
    }
    
    //eprintf("run to here0\n");
    q->Count = (INTEGER16)(*count);

    //change the queue end
    q->next = NULL;

    queue ->rear ->next = q;
    queue -> rear = q;
    //*count = *count+1;
    return 0;
    
}
