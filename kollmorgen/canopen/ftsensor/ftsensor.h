#ifndef __KERNEL__
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#else
#include <linux/types.h>
#endif

#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <applicfg.h>



typedef unsigned char byte;
typedef struct response_struct {
	UNS32 rdt_sequence;
	UNS32 ft_sequence;
	UNS32 status;
	INTEGER32 FTData[6];
} RESPONSE;

/*初始化互斥锁*/
void ftsensor_enter_mutex();
void ftsensor_leave_mutex();

/*初始化函数，用于初始化FT力传感器socket*/
int ftsensor_init(const char*hostname);

/*接收函数 接收UDP ft结构体*/
RESPONSE ftsensor_recv();

extern RESPONSE resp;				/* The structured response received from the Net F/T. */
char * AXES[] = { "Fx", "Fy", "Fz", "Tx", "Ty", "Tz" };	/* The names of the force and torque axes. */
