#include "rgmtcp.h"
#include "TestMaster.h"
#include "pvt_src.h"
#include "rgm_src.h"
#include "fb_src.h"
#include "RGMikControl.hpp"

char receivebuf[BUFFER_SIZE];           // Receive buffer  
char sendbuf[BUFFER_SIZE];              // Send buffer

/*global data in tcp-pc2mc*/
int sock_fd;
int nsock_fd;
struct sockaddr_in server_addr;
int tcp_connected = 0;

const char delims0[] = "\n";
int handle = -1;

const char delims1[] = {";"};
const char delims2[] = {","};

int stop_process(int control_mode){

	int ret = 0;

	ControlWord = SWITCH_ON;
			//strcpy(inputbuf,"");
			if((control_mode == MOTION_PREPARE)||(control_mode == MOTION_PLAN)){
					
					control_mode = COMMAND;
					destory_pvt_queue(&pvt_command_queue);
					TargetVelocity1 = 0;
					TargetVelocity2 = 0;
					TargetVelocity3 = 0;
					TargetVelocity4 = 0;
					TargetVelocity5 = 0;
					TargetVelocity6 = 0;
				}

			if(control_mode == FEEDBACK_CONTROL){
					TargetVelocity1 = 0;
					TargetVelocity2 = 0;
					TargetVelocity3 = 0;
					TargetVelocity4 = 0;
					TargetVelocity5 = 0;
					TargetVelocity6 = 0;

					rgm_Ctrl_dele_wrap(&handle);
					free(pRGM);
					control_mode = COMMAND;
			}
			return 1;

}


//0 : no command
//1 : command
//2 : shutdown

int tcp_process_command(char* inputbuf){
	int ret = 0;
	// printf("\\*****\\\nRECEIVE:%s\n\\*****\\\n", receivebuf);
		
		if(!strncmp(inputbuf,"STOP",4)){ //receive command Start

	
			ret = stop_process(control_mode);
			strcpy(inputbuf,"");
			return ret;
        }

		if(!strncmp(inputbuf,"SHUTDOWN",8)){ //receive command Start
			ControlWord =SWITCH_ON;
            //ret_mode = SHUTDOWN_RGM;
			return 2; 
        }
		if(!strncmp(inputbuf,"START",5)){ //receive command Start
            
			ControlWord = SWITCH_ON;
			sleep(1);
			ControlWord = ENABLE_OPERATION;
			strcpy(inputbuf,"");
			printf("COMMAND START!!\n");
			return 1; 
        }//"PVT_START"
		if(!strncmp(inputbuf,"PVT_START",9)){ //receive command Start
            
			/*change_1995*/
			strcpy(inputbuf,"");
			position_sum = ActualPosition1;
			control_mode = MOTION_PLAN;
			ControlWord = ENABLE_OPERATION;
			trajCtrlinit_wrap(&handle,pvt_command_queue,300000);
			return 1; 
        }//"PVT_PREPARE000"
		if(!strncmp(inputbuf,"PVT_PREPARE",11)){
		
			control_mode = MOTION_PREPARE;
			pvt_queue_num = 0;
			init_pvtqueue(&pvt_command_queue);
			printf("PVT_PREPARE_GET\n");
                    
			return 1;
		}


}

// 处理TCP上的发送消息，TCP 线程循环函数
int read_buff(char* rbuff,int control_mode){
    //1.split rbuff with \n
    //2.rbuff write
    
    char* result = NULL;
	//const char* const nobuff = "";
    char temp_buff[100];
    int rec = 0;
    int n_flag = 0;
	char *key_n;

	if(*(rbuff+strlen(rbuff)-1) != 10)
		n_flag = 1;

    //split with
    result = strtok_r(rbuff,delims0,&key_n);
    while(result != NULL){
        //copy temp buff
        strcpy(temp_buff,result);
		printf("temp_buff = %s\n",temp_buff);
        result = strtok_r(NULL,delims0,&key_n);
		printf("result = %s n_flag = %d\n",result,n_flag);
        //result lead to a final
		if((result == NULL) && (n_flag==1)){
			break;
		}
        
		//TCP 处理数据
		rec=tcp_process_command(temp_buff);
		if(rec == 1){
			continue;
		}
		else if(rec == 2){
			//strcpy(rbuff,"");
			printf("SHUTDOWN 123456\n");
			return 2;
		}
		printf("result,control_mode,isEmpty= %d,%d.%d\n",rec,control_mode,isEmpty(target_queue));
		//process_data
		printf("\nrun to here0 mode = %d \n\n",control_mode);
		//切换控制方式
		switch(control_mode)
        {
            case MOTION_PREPARE:
                if (rec=insert_pvt_queue(&pvt_command_queue,&pvt_queue_num,temp_buff) < 0){
                    
                printf("error in insert_pvt_queue function\n");
                return -1; 
                }
                if( rec == 0 ){
                    pvt_queue_num++;
					strcpy(temp_buff,"");
                }
				break;
				
            //case MOTION_PLAN:
            case COMMAND:
				//printf("run to here01 rec = %d \n",rec);
                command_tcp_queue(temp_buff);
				strcpy(temp_buff,"");
				printf("run to here0 mode = %d \n",control_mode);
				break;

			case FEEDBACK_CONTROL:
				if(pRGM == NULL){
					printf("Error in inialization robot tcp target\n");
					return -1;
				}
				else{
					//rec = fb_tcp_queue(temp_buff);
					rec = 0;
					strcpy(temp_buff,"");
					if(rec != 0){
					printf("Error in robot tcp target 222\n");	
					}
				}
                
        }
        //Leave_pvtqueue_Mutex();
    }

	//d
	if(n_flag > 0)
		strcpy(rbuff,temp_buff);
	else
		strcpy(rbuff,"");
		
	return 0;
}


int initialisation_mc()
{
    if ((sock_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        printf("create socket error: %s(errno: %d)\n", strerror(errno),errno);  
        return -1;
    } 
    memset(&server_addr, 0, sizeof(server_addr));           // server address
    server_addr.sin_family = AF_INET;                       // Protocol Family 
    server_addr.sin_port = htons(PORT);                     // Port number 
    if( inet_pton(AF_INET, SERVER_ADDR, &server_addr.sin_addr) <= 0)
    {
        printf("inet_pton error for %s\n",SERVER_ADDR);
        return -2;
    }
    /* Try to connect the remote */
    if ( nsock_fd=connect(sock_fd, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) < 0)
    {
        // printf ("ERROR: Failed to connect to the host!\n");
        printf("connect error: %s(errno: %d)\n",strerror(errno),errno); 
        return -3;
    }
    else
    {
        printf ("OK: Have connected to the %s\n",SERVER_ADDR);
        tcp_connected = 1;
    }
    

    return 0;
}

void tcp_read(LinkQueue *queue,char* recbuf){
    char* result = NULL;
    char* sub_result = NULL;
    int num = 0;
    char sub_buff_r[100] = "";
    // char sub_buff_v[100] = "";
    // char sub_buff_tq[100] = "";
    // char sub_buff_s[100] = "";
    char sub_buff[100];
    char *key,*key_p,*key_v,*key_tq,*key_s;
    
    /*insert a queue*/
    Queue q = (Queue)calloc(1,sizeof(Node));
    q->Position[0] = ActualPosition1;
    q->Position[1] = ActualPosition2;
    q->Position[2] = ActualPosition3;
    q->Position[3] = ActualPosition4;
    q->Position[4] = ActualPosition5;
    q->Position[5] = ActualPosition6;


    if (NULL==q){
        //exit(-1);
        printf("No memoray to save");
        return;
    }
    strcpy(sub_buff,recbuf);
    result = strtok_r(sub_buff,delims1,&key);
    while(result != NULL){
        switch(*result){
            //num = 0;
            case 112:
                //position
                strcpy(sub_buff_r,result);
                //position
                num = 0;
                sub_result = strtok_r(sub_buff_r,delims2,&key_p);
                while((sub_result != NULL)&&(num<6)){
                    if(num ==0){

                        sub_result = substring(sub_result,2,9);

                    }

                    q->Position[num] = (INTEGER32)strtol(sub_result,NULL,16); 
                    sub_result = strtok_r(NULL,delims2,&key_p);
                    num = num+1 ;
                }
                break;
            case 118:
                //velocity
                strcpy(sub_buff_r,result);

                 num = 0;
                sub_result = strtok_r(sub_buff_r,delims2,&key_v);
                while((sub_result != NULL)&&(num<6)){
                    if(num ==0){
                        printf("sub_result1 = %s\n",sub_result);
                        sub_result = substring(sub_result,2,9);
                        printf("sub_result2 = %s\n",sub_result);
                    }

                    q->Velocity[num] = (INTEGER32)strtol(sub_result,NULL,16); 
                    sub_result = strtok_r(NULL,delims2,&key_v);
                    num = num+1 ;
                }           
                break;
            case 116:
                //torque
                strcpy(sub_buff_r,result);
                num = 0;
                sub_result = strtok_r(sub_buff_r,delims2,&key_tq);
                while((sub_result != NULL)&&(num<6)){
                    if(num ==0){
                        sub_result = substring(sub_result,2,4);
                    }

                    q->Torque[num] = (INTEGER16)strtol(sub_result,NULL,16); 
                    sub_result = strtok_r(NULL,delims2,&key_tq);
                    num = num+1 ;
                }
                break;
            case 115:
                strcpy(sub_buff_r,result);
                //newstate = 1;

                num = 0;

                printf("sub_buff_r = %s\n", sub_buff_r);
                sub_result = strtok_r(sub_buff_r,delims2,&key_s);
                printf("sub_result = %s\n", sub_result);
                while((sub_result != NULL)&&(num<6)){
                    if(num ==0){
                        sub_result = substring(sub_result,2,2);
                    }
                    else if(num == 5){
                        sub_result[2] = '\0';
                    }
                    printf("\nsub_result_s = %s\n",sub_result);
                    q->OperationMode[num] = (INTEGER8)strtol(sub_result,NULL,16); 
                    
                    sub_result = strtok_r(NULL,delims2,&key_s);
                    num = num+1 ;
                }
                break;
            default :
                free(q);
                printf("ERROR MESSAGE!!!\n");
                return;
            result = strtok_r(NULL,delims1,&key);
            
        }
      

    //change the queue end
    q->next = NULL;

    queue ->rear ->next = q;
    queue -> rear = q;
    return;

    }
}

void tcp_write(LinkQueue *queue){
    int counts = 0;
    //printf("run to here 00\n");
    if(!isEmpty(*queue)){
    //printf("\nsprintf :%08x  %08x \n",(queue->front->next->Velocity[1]),ActualVelocity2);

    counts=sprintf(sendbuf,"p[%08x,%08x,%08x,%08x,%08x,%08x];",\
                queue->front->next->Position[0],queue->front->next->Position[1],\
                queue->front->next->Position[2],queue->front->next->Position[3],\
                queue->front->next->Position[4],queue->front->next->Position[5]);
    
    counts+=sprintf(sendbuf+counts,"v[%08x,%08x,%08x,%08x,%08x,%08x];",\
                queue->front->next->Velocity[0],queue->front->next->Velocity[1],\
                queue->front->next->Velocity[2],queue->front->next->Velocity[3],\
                queue->front->next->Velocity[4],queue->front->next->Velocity[5]);
    
    counts+=sprintf(sendbuf+counts,"t[%04x,%04x,%04x,%04x,%04x,%04x];",\
                queue->front->next->Torque[0]& (0xFFFF),queue->front->next->Torque[1]&(0xFFFF),\
                queue->front->next->Torque[2]& (0xFFFF),queue->front->next->Torque[3]&(0xFFFF),\
                queue->front->next->Torque[4]& (0xFFFF),queue->front->next->Torque[5]&(0xFFFF));
    
    counts+=sprintf(sendbuf+counts,"s[%02x,%02x,%02x,%02x,%02x,%02x]\n",\
                queue->front->next->OperationMode[0]& (0xFF),queue->front->next->OperationMode[1]& (0xFF),\
                queue->front->next->OperationMode[2]& (0xFF),queue->front->next->OperationMode[3]& (0xFF),\
                queue->front->next->OperationMode[4]& (0xFF),queue->front->next->OperationMode[5]& (0xFF));
    }
    
}


//tcp queue process
//order mode
//在程序里的TCP线程的函数，负责队列的收集和发散
void command_tcp_queue(char* recbuf){
    //printf("run to here02\n");
    int numt = 0;    
    Enter_queue_Mutex();
    
   // 执行与上位机通信操作，读取指令信息
    tcp_read(&target_queue,recbuf);
    // 发送传感器信息
 
    Leave_queue_Mutex();

    // if((numt=send(sock_fd,sendbuf,strlen(sendbuf),MSG_NOSIGNAL)) < 0){
    //         printf("ERROR:Fail to send string\n");
    //         close(sock_fd);
    //         //exit(1);
    //         tcp_connected = 0;
    //         return;
    //         }


}

