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
		if(!strncmp(inputbuf,"FRAME_CONTROL",13)){
		
			pRGM = (RGM_ROBOT*) calloc (1,sizeof(RGM_ROBOT));

			if(pRGM == NULL){
				printf("No memory to calloc\n");
				return -1;
			}
			
			control_mode = FEEDBACK_CONTROL;
			pRGM->control_mode = FRAME_BASE_CONTROL;

			rgm_Ctrl_init_wrap(&handle);
        
			return 1;
		}

		//return 0;

}

// 处理TCP上的发送消息，进行接收
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
        
		//process command
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
				//printf("run to here0 mode = %d \n",control_mode);
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
