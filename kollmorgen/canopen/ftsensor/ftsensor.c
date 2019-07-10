#include "ftsensor.h"
#include "pthread.h"

#define PORT 49152 /* Port the Net F/T always uses */
#define COMMAND 2 /* Command code 2 starts streaming */
#define NUM_SAMPLES 1 /* Will send 1 sample before stopping */




int socketHandle;			/* Handle to UDP socket used to communicate with Net F/T. */
struct sockaddr_in addr;	/* Address of Net F/T. */
struct hostent *he;			/* Host entry for Net F/T. */
byte request[8];			/* The request data sent to the Net F/T. */
byte response[36];			/* The raw response data received from the Net F/T. */
int i;						/* Generic loop/array index. */
int err;					/* Error status of operations. */




pthread_mutex_t ftsensor_mutex = PTHREAD_MUTEX_INITIALIZER;
//线程互斥锁 初始化
void ftsensor_enter_mutex()
{
    pthread_mutex_lock(&ftsensor_mutex);
}

void ftsensor_leave_mutex()
{
    pthread_mutex_unlock(&ftsensor_mutex);
}

/*spin function */
void* ftsensor_spin(){
    while(1){
        //读取，并保存在全局变量resp中
        ftsensor_enter_mutex();
        resp = ftsensor_recv();
        ftsensor_leave_mutex();
    }
}



int ftsensor_init(const char*hostname)
{
    /*Open Socket here*/
    socketHandle = socket(AF_INET, SOCK_DGRAM, 0);
	if (socketHandle == -1) {
		exit(1);
	}

    *(UNS16*)&request[0] = htons(0x1234); /* standard header. */
	*(UNS16*)&request[2] = htons(COMMAND); /* per table 9.1 in Net F/T user manual. */
	*(UNS32*)&request[4] = htonl(NUM_SAMPLES); /* see section 9.1 in Net F/T user manual. */

    /*Send the request*/
    he = gethostbyname(hostname);
	memcpy(&addr.sin_addr, he->h_addr_list[0], he->h_length);
	addr.sin_family = AF_INET;
	addr.sin_port = htons(PORT);
	
	err = connect( socketHandle, (struct sockaddr *)&addr, sizeof(addr) );
	if (err == -1) {
		exit(2);
	}
	send( socketHandle, request, 8, 0 );

    //TODO 初始化线程
    pthread_t ftsensor_tid;
    pthread_create(ftsensor_tid,NULL,ftsensor_spin,NULL);
    pthread_join(ftsensor_tid,NULL);


}

/* Receiving the response. */
RESPONSE ftsensor_recv()
{
    	/* Receiving the response. */
    RESPONSE resp;				/* The structured response received from the Net F/T. */
	recv( socketHandle, response, 36, 0 );
	resp.rdt_sequence = ntohl(*(UNS32*)&response[0]);
	resp.ft_sequence = ntohl(*(UNS32*)&response[4]);
	resp.status = ntohl(*(UNS32*)&response[8]);
	for( i = 0; i < 6; i++ ) {
		resp.FTData[i] = ntohl(*(INTEGER32*)&response[12 + i * 4]);
	}

    return resp;
}
