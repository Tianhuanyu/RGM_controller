#ifdef __cpluscplus
extern "C"{
#endif

#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <errno.h>


#include "rgm6.h"
#include "rgmtcp.h"
#include "rgm_src.h"
#include "kollmorgen.h"

#ifdef __cpluscplus
}
#endif

#define SLEEP(time) sleep(time)


#define cst_str4(c1, c2, c3, c4) ((((unsigned int)0 | \
                                    (char)c4 << 8) | \
                                   (char)c3) << 8 | \
                                  (char)c2) << 8 | \
                                 (char)c1
#define QUIT 1


int main(void)
{
    char command[200];
	char fedstr[8001];
	char* res;
	int retf = 0;
	int ret_mode = 0;// route control
	int sysret = 0;
	//int numt = 0;
	int rec_len = 0;
	int times = 0; // tcp reconnect frequncy flag
	char* chbuff = NULL;
	int i=0;

	/*Rgm6_Init() success return 0;fail return not 0*/
	if(retf=Rgm6_Init() < 0)
	printf("ERROR:Rgm6 fail to Initialization\n");
	/*as above*/
	if(retf=Rgm6_Start() != 0)
	printf("ERROR:Rgm6 fail to start timerloop\n");
	/*as above*/
	if(retf=initialisation_mc() != 0)
	printf("ERROR:Rgm6 fail to access to pc\n");

	initQueue(&target_queue);
	initQueue(&actual_queue);
	pthread_mutex_init(&mutex,NULL);
	ControlWord = SWITCH_ON;
    
	/* Enter in a loop to read stdin command until "quit" is called */
	while(ret_mode != SHUTDOWN_RGM)
	{
		// // wait on stdin for string command
		// res = fgets(command, sizeof(command), stdin);
		// ret = ProcessCommand(command);
		// fflush(stdout);
		//wait for receivebuf block
		strcpy(receivebuf,"");
		if((rec_len = recv(sock_fd, receivebuf, BUFFER_SIZE,0)) < 0)
        {  
            perror("recv error");
			//ControlWord = SWITCH_ON;
			tcp_connected = 0;   
        }
		else{
			receivebuf[rec_len] = '\0';
		}

		printf("receive buff:%s\n",receivebuf);
		//1.+ =
		//odd string + new string
		strcat(fedstr,receivebuf);

		//2. split function
		//3. read function
		retf=read_buff(fedstr,control_mode);
		if(retf == 2){
				ret_mode = SHUTDOWN_RGM;
				printf("SHUTDOWN--\n");
				break;
				}
		else if(retf == -1){
			printf("ERROR OCCURE IN PT INSERT FUNCITON\n");
		}

			


		if ((tcp_connected== 0)&&(times++ == 100)){
			times = 0;
			printf ("Try to reconnect to server\n");
			if(retf=initialisation_mc() != 0)
				printf("ERROR:Rgm6 fail to access to pc\n");

			strcpy(receivebuf,"");
		}		
	
	}
	/*as above;see Rgm6_Init()*/
	if(retf=Rgm6_Stop() != 0)
		printf("ERROR:Rgm6 fail to stop\n");

    return 0;
}