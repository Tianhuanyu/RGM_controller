
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

#include "canfestival.h"
//#include <can_driver.h>
//#include <timers_driver.h>
#include "rgm6.h"
#include "Master.h"


s_BOARD MasterBoard = {"0", "1M"};

void InitNodes(CO_Data* d, UNS32 id)
{
	if(MasterBoard.baudrate){
		printf("************ InitNodes ******************\n");
		/* Defining the node Id */
		setNodeId(&TestMaster_Data, 0x01);
		/* init */
		setState(&TestMaster_Data, Initialisation);
	}
}

void Exit(CO_Data* d, UNS32 id)
{
	if(strcmp(MasterBoard.baudrate, "none")){
		printf("************ Exit ******************\n");
		masterSendNMTstateChange(&TestMaster_Data, 0x00, NMT_Reset_Node);    
       	//Stop master
		setState(&TestMaster_Data, Stopped);
	}
}

int Rgm6_Init()
{

	ControlWord = 0x04;
	OperationMode1 = 0x03;
	OperationMode2 = 0x03;
	OperationMode3 = 0x03;
	OperationMode4 = 0x03;
	OperationMode5 = 0x03;
	OperationMode6 = 0x03;

	TimerInit();	

	if(strcmp(MasterBoard.baudrate, "none"))
	{		
		TestMaster_Data.heartbeatError = TestMaster_heartbeatError;
		TestMaster_Data.initialisation = TestMaster_initialisation;
		TestMaster_Data.preOperational = TestMaster_preOperational;
		TestMaster_Data.operational = TestMaster_operational;
		TestMaster_Data.stopped = TestMaster_stopped;
		TestMaster_Data.post_sync = TestMaster_post_sync;
		TestMaster_Data.post_TPDO = TestMaster_post_TPDO;
		TestMaster_Data.post_emcy = TestMaster_post_emcy;
		TestMaster_Data.post_SlaveBootup=TestMaster_post_SlaveBootup;
		
		if(!canOpen(&MasterBoard,&TestMaster_Data))
		{
			eprintf("Cannot open Master Board (%s,%s)\n",MasterBoard.busname, MasterBoard.baudrate);
			return -2;
		}
	}	

	return 0;
}

int Rgm6_Start()
{
	StartTimerLoop(&InitNodes);
	return 0;
}

int Rgm6_Stop()
{
	ControlWord = SHUTDOWN;
	StopTimerLoop(&Exit);
    TimerCleanup();
    printf("Finished RGM6 kollmorgen process\n");
	
	
	return 0;
}


