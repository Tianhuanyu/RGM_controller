
#ifndef _RGMTCP_H
#define _RGMTCP_H


#ifdef __cpluscplus
extern "C"{
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>  
#include <netdb.h>  
#include <arpa/inet.h> 
#include <pthread.h>
#include "rgm_src.h"

#define PORT            2040
#define BUFFER_SIZE     8001
#define SERVER_ADDR     "192.168.1.107"
#define GET_ARRAY_LEN(array,len) {len = (sizeof(array) / sizeof(array[0]));}

int stop_process(int control_mode);

int read_buff(char* rbuff,int control_mode);

int tcp_process_command(char* inputbuf);

int initialisation_mc();

void tcp_read(LinkQueue *queue,char* recbuf);

void tcp_write(LinkQueue *queue);

void command_tcp_queue(char* recbuf);


extern int tcp_connected;
extern int sock_fd;
extern int nsock_fd;
extern struct sockaddr_in server_addr;

extern char receivebuf[BUFFER_SIZE];           // Receive buffer  
extern char sendbuf[BUFFER_SIZE];              // Send buffer
extern int handle;

#ifdef __cpluscplus
}
#endif

#endif
