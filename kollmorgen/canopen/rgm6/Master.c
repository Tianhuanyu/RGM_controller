 /*
This file is part of CanFestival, a library implementing CanOpen Stack. 

Copyright (C): Edouard TISSERANT and Francis DUPIN

See COPYING file for copyrights details.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

#include "Master.h"
#include "rgm6.h"
#include "rgmtcp.h"

extern s_BOARD MasterBoard;
int isinit = 0;
int count = 0;
int step_pvt = 0;
const char end_request[] = "PVT_FINISHED\n";

long long time_now = 0,time_last=0,time_use=0;
struct timeval tv;
/*****************************************************************************/
void TestMaster_heartbeatError(CO_Data* d, UNS8 heartbeatID)
{
	eprintf("TestMaster_heartbeatError %d\n", heartbeatID);
}

/*****************************************************************************/
INTEGER8 Operation_Mode = PROFILE_VELOCITY_MODE;
void TestMaster_initialisation(CO_Data* d)
{
	UNS32 RPDO_COBID[7] = {0x80000282, 0x80000292, 0x800002A2, 0x80000212, 0x80000222, 0x80000232, 0x80000242}; 
	UNS32 TPDO_COBID[7] = {0x80000402, 0x80000412, 0x80000422, 0x80000302, 0x80000452, 0x80000462, 0x80000472};
	UNS32 size = sizeof(UNS32); 
	// UNS32 SINC_cicle=0;
	// UNS8 data_type = 0;	
	eprintf("TestMaster_initialisation\n");

	// change tpdo state based on opearation_mode
	switch(Operation_Mode)
	{
		case PROFILE_POSITION_MODE :
			TPDO_COBID[0] &= ~0x80000000;
			TPDO_COBID[3] &= ~0x80000000;
			break;
		case PROFILE_VELOCITY_MODE :
			TPDO_COBID[0] &= ~0x80000000;
			TPDO_COBID[1] &= ~0x80000000;
			TPDO_COBID[3] &= ~0x80000000;
			//TPDO_COBID[4] &= ~0x80000000;
			break;
		case PROFILE_TORQUE_MODE :
			TPDO_COBID[2] &= ~0x80000000;
			TPDO_COBID[3] &= ~0x80000000;
			break;
		case CYCLIC_POSITION_MODE :
			TPDO_COBID[4] &= ~0x80000000;
			TPDO_COBID[3] &= ~0x80000000;
			break;
		case CYCLIC_VELOCITY_MODE :
			TPDO_COBID[5] &= ~0x80000000;
			TPDO_COBID[3] &= ~0x80000000;
			break;
		case CYCLIC_TORQUE_MODE :
			TPDO_COBID[6] &= ~0x80000000;
			TPDO_COBID[3] &= ~0x80000000;
			break;
		default :
			break;
	}
	for (int i = 0; i < 7; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			UNS32 TPDO_COBId = TPDO_COBID[i] + j;

			//brief: select enable joint with j
			//e.g: j!=0 ==> slave 2 is enabled
			// if (!(j < 1))
			// {
			// 	TPDO_COBId |= 0x80000000;
			// }			
			if(0x1800 + i*6+j == 0x1806){
				printf("\nTPDO_COBId = %x\n",TPDO_COBId);
			}
			writeLocalDict( &TestMaster_Data, /*CO_Data* d*/
			0x1800 + i*6+j, /*UNS16 index*/
			0x01, /*UNS8 subind*/ 
			&TPDO_COBId, /*void * pSourceData,*/ 
			&size, /* UNS8 * pExpectedSize*/
			RW);  /* UNS8 checkAccess */
			
		}
	}					
}

//brief:Step counts number of times 
//       ConfigureSlaveNode is called
//      index means node id
static int init_step[6] = {0, 0, 0, 0, 0, 0};
//brief: the index of TPDO/RPDO 	
static int init_index[6] = {0, 0, 0, 0, 0, 0};
static void ConfigureSlaveNode(CO_Data* d, UNS8 nodeId);		/*Froward declaration*/

static void CheckSDOAndContinue(CO_Data* d, UNS8 nodeId)
{
	UNS32 abortCode;	
	if(getWriteResultNetworkDict (d, nodeId, &abortCode) != SDO_FINISHED)
		eprintf("!!!!!!!! Master : Failed in initializing slave %2.2x, step %d, index %d AbortCode :%4.4x \n", nodeId, init_step[nodeId-2], init_index[nodeId-2], abortCode);
	/* Finalise last SDO transfer with this node */
	closeSDOtransfer(&TestMaster_Data, nodeId, SDO_CLIENT);
	ConfigureSlaveNode(d, nodeId);
}

/********************************************************
 * ConfigureSlaveNode is responsible to
 *  - setup slave TPDO and RPDO transmit type
 *  - switch to operational mode
 *  - send NMT to slave
 ********************************************************/ 
static void ConfigureSlaveNode(CO_Data* d, UNS8 nodeId)
{
	
	//brief: PDO_COBID index
	UNS32 TPDO_COBID[10] = {0x80000282, 0x80000292, 0x800002A2, 0x80000212, 0x80000222, 0x80000232, 0x80000242, 0x800002C2, 0x800002D2, 0x800002E2}; 
	UNS32 RPDO_COBID[10] = {0x80000402, 0x80000412, 0x80000422, 0x80000302, 0x80000452, 0x80000462, 0x80000472, 0x800004C2, 0x800004D2, 0x800004E2}; 
	// UNS16 TPDO_INDEX[10] = {0x1800, 0x1802, 0x1803, 0x1804, 0x1814, 0x1815, 0x1816, 0x1817, 0x1818, 0x1819};
	// UNS16 RPDO_INDEX[10] = {0x1400, 0x1401, 0x1402, 0x1403, 0x1404, 0x1414, 0x1415, 0x1416, 0x1417, 0x1419};
	UNS16 TPDO_INDEX[10] = {0x1802, 0x1803, 0x1804, 0x1800, 0x1814, 0x1815, 0x1816, 0x1817, 0x1818, 0x1819};
	UNS16 RPDO_INDEX[10] = {0x1402, 0x1403, 0x1404, 0x1401, 0x1414, 0x1415, 0x1416, 0x1400, 0x1417, 0x1419};
	UNS8 res;
	
	//brief : select operation mode
	switch(Operation_Mode)
	{
		case PROFILE_POSITION_MODE :
			RPDO_COBID[0] &= ~0x80000000;
			RPDO_COBID[1] &= ~0x80000000;
			RPDO_COBID[2] &= ~0x80000000;
			RPDO_COBID[3] &= ~0x80000000;
			RPDO_COBID[4] &= ~0x80000000;
			RPDO_COBID[5] &= ~0x80000000;
			break;
		case PROFILE_VELOCITY_MODE :
			RPDO_COBID[0] &= ~0x80000000;
			RPDO_COBID[1] &= ~0x80000000;
			RPDO_COBID[3] &= ~0x80000000;
			RPDO_COBID[4] &= ~0x80000000;
			break;
		case PROFILE_TORQUE_MODE :
			RPDO_COBID[2] &= ~0x80000000;
			RPDO_COBID[3] &= ~0x80000000;
			break;
		case CYCLIC_POSITION_MODE :
			RPDO_COBID[4] &= ~0x80000000;
			RPDO_COBID[3] &= ~0x80000000;
			break;
		case CYCLIC_VELOCITY_MODE :
			RPDO_COBID[5] &= ~0x80000000;
			RPDO_COBID[3] &= ~0x80000000;
			break;
		case CYCLIC_TORQUE_MODE :
			RPDO_COBID[6] &= ~0x80000000;
			RPDO_COBID[3] &= ~0x80000000;
			break;
		default :
			break;
	}
	
	//brief: plus one after index dict go next
	if (init_step[nodeId-2] == 7)
	{
		init_index[nodeId-2]++;
		if (init_index[nodeId-2] < 10)
		{
			init_step[nodeId-2] = 0;
		}		
	}
	UNS32 TPDO_COBid;
	UNS32 RPDO_COBid;
	UNS16 TPDO_Index;
	UNS16 RPDO_Index;
	// brief : change PDO COB-ID or index with init_index 
	if (!((init_step[nodeId-2] == 9)&&(init_index[nodeId-2] == 10)))
	{
		TPDO_COBid = TPDO_COBID[init_index[nodeId-2]] + nodeId - 0x02;
		RPDO_COBid = RPDO_COBID[init_index[nodeId-2]] + nodeId - 0x02;
		TPDO_Index = TPDO_INDEX[init_index[nodeId-2]];
		RPDO_Index = RPDO_INDEX[init_index[nodeId-2]];
 
		// eprintf("nodeId : %2.2x; init_step: %d; init_index: %d; TPDO_Index: %4.4x; RPDO_Index: %4.4x, TPDO_COBid: %8.8x, RPDO_Index: %8.8x\n", nodeId,
	 	// 			init_step[nodeId-2],
		// 			init_index[nodeId-2],
		// 			TPDO_Index,
		// 			RPDO_Index,
		// 			TPDO_COBid,
		// 			RPDO_COBid
		// 			);
	}

	switch(++init_step[nodeId-2])
	{
	case 1:
	{
		//TPDO_COBid |= 0x80000000;
		eprintf("Master : slave %2.2x TPDO %4.4x TPDO_COBid %x\n", nodeId, TPDO_Index, TPDO_COBid); 
		res = writeNetworkDictCallBack (d, /*CO_Data* d*/
				nodeId, /*UNS8 nodeId*/
				TPDO_Index, /*UNS16 index*/
				0x01, /*UNS8 subindex*/
				4, /*UNS8 count*/
				0, /*UNS8 dataType*/
				&TPDO_COBid,/*void *data*/
				CheckSDOAndContinue, /*SDOCallback_t Callback*/
            	0); /* use block mode */
	}
	break;
	case 2:
	{
		UNS8 Transmission_Type = 0x01;
		eprintf("Master : set slave %2.2x TPDO %4.4x transmit type %x\n", nodeId, TPDO_Index,Transmission_Type);
		res = writeNetworkDictCallBack (d, /*CO_Data* d*/
				nodeId, /*UNS8 nodeinit_stepId*/
				TPDO_Index, /*UNS16 index*/
				0x02, /*UNS8 subindex*/
				1, /*UNS8 count*/
				0, /*UNS8 dataType*/
				&Transmission_Type,/*void *data*/
				CheckSDOAndContinue, /*SDOCallback_t Callback*/
            	0); /* use block mode */
	}
	break;
	case 3: 
	{
		if (init_index[nodeId-2] <= 4)
		{
		TPDO_COBid &= ~0x80000000;
		}
		eprintf("Master : re-enable or re-disable slave %2.2x TPDO %4.4x TPDO_COBid %x\n", nodeId, TPDO_Index,TPDO_COBid);
		res = writeNetworkDictCallBack (d, /*CO_Data* d*/
				nodeId, /*UNS8 nodeId*/
				TPDO_Index, /*UNS16 index*/
				0x01, /*UNS8 subindex*/
				4, /*UNS8 count*/
				0, /*UNS8 dataType*/
				&TPDO_COBid,/*void *data*/
				CheckSDOAndContinue, /*SDOCallback_t Callback*/
                0); /* use block mode */
	}
	break;
	case 4:
	{
		RPDO_COBid |= 0x80000000;
		eprintf("Master : disable slave %2.2x RPDO %4.4x RPDO_COBid %x\n", nodeId, RPDO_Index,RPDO_COBid); 
		res = writeNetworkDictCallBack (d, /*CO_Data* d*/
				nodeId, /*UNS8 nodeId*/
				RPDO_Index, /*UNS16 index*/
				0x01, /*UNS8 subindex*/
				4, /*UNS8 count*/
				0, /*UNS8 dataType*/
				&RPDO_COBid,/*vo#include "Master.h"id *data*/
				CheckSDOAndContinue, /*SDOCallback_t Callback*/
            	0); /* use block mode */
	}
	break;
	case 5:
	{
		UNS8 Transmission_Type = 0x01;
		eprintf("Master : set slave %2.2x RPDO %4.4x receive  %d\n", nodeId, RPDO_Index,Transmission_Type);
		res = writeNetworkDictCallBack (d, /*CO_Data* d*/
				nodeId, /*UNS8 nodeId*/
				RPDO_Index, /*UNS16 index*/
				0x02, /*UNS8 subindex*/
				1, /*UNS8 count*/
				0, /*UNS8 dataType*/
				&Transmission_Type,/*void *data*/
				CheckSDOAndContinue, /*SDOCallback_t Callback*/
            	0); /* use block mode */
	}
	break;
	case 6: 
	{
		// RPDO_COBid &= ~0x80000000;
		eprintf("Master : re-enable or re-disable slave %2.2x RPDO %4.4x RPDO_COBid %x\n", nodeId, RPDO_Index,RPDO_COBid);
		res = writeNetworkDictCallBack (d, /*CO_Data* d*/
				nodeId, /*UNS8 nodeId*/
				RPDO_Index, /*UNS16 index*/
				0x01, /*UNS8 subindex*/
				4, /*UNS8 count*/
				0, /*UNS8 dataType*/
				&RPDO_COBid,/*void *data*/
				CheckSDOAndContinue, /*SDOCallback_t Callback*/
                0); /* use block mode */
	}			
	break;
	case 7:
	{
		UNS16 Heartbeat_Producer_Time = 0x0000; 
		eprintf("Master : set slave %2.2x heartbeat producer time \n", nodeId);
		res = writeNetworkDictCallBack (d, /*CO_Data* d*/
				nodeId, /*UNS8 nodeId*/
				0x1017, /*UNS16 index*/
				0x00, /*UNS8 subindex*init_index*/
				2, /*UNS8 count*/
				0, /*UNS8 dataType*/
				&Heartbeat_Producer_Time,/*void *data*/
				CheckSDOAndContinue, /*SDOCallback_t Callback*/
                0); /* use block mode */
	}
	break;

	break;
	case 8: /* START */
		if (nodeId == 0x07)
		{
			/* Put the master in operational mode  run out 6 RGM upset process*/
			setState(d, Operational);
			//  /* Ask slave node to go in operational mode */
			masterSendNMTstateChange (d, 0x00, NMT_Start_Node);
		}
		else
		{
			masterSendNMTstateChange(&TestMaster_Data, nodeId+0x01, NMT_Reset_Node);
			//printf("test nodeId + 0x01 = %d \n ",nodeId+0x01);
		}
		
		break;	
	}
}


void TestMaster_preOperational(CO_Data* d)
{
	eprintf("TestMaster_preOperational\n");
	masterSendNMTstateChange(&TestMaster_Data, 0x02, NMT_Reset_Node);
	//eprintf("TestMaster_preOperational 222\n");
}

void TestMaster_operational(CO_Data* d)
{
	eprintf("TestMaster_operational\n");
	// UNS32 Cycle_Period = 0x0000C350;
	UNS32 Cycle_Period = 0x0001356;
	UNS32 size = sizeof(UNS32); 
	writeLocalDict( &TestMaster_Data, /*CO_Data* d*/
			0x1006, /*UNS16 index*/
			0x00, /*UNS8 subind*/ 
			&Cycle_Period, /*void * pSourceData,*/ 
			&size, /* UNS8 * pExpectedSize*/
			RW);  /* UNS8 checkAccess */
	eprintf("TestMaster_Operational 222\n");
}

void TestMaster_stopped(CO_Data* d)
{
	eprintf("TestMaster_stopped\n");
}


int rec_output = 0;
int display = 0;
int rec_state_change = 0;
void TestMaster_post_sync(CO_Data* d)
{
	count++;
	int position_sum = 0;
	if ((display == 1)&&(count < 30))
	{
		//eprintf("Position3 : %8.8ld  Velocity3 : %8.8ld  ActualTorque3 : %8.8ld\n", ActualPosition3, ActualVelocity3, ActualTorque3);
	}
	//eprintf("Position3 : %8.8ld  Velocity3 : %8.8ld  ActualTorque3 : %8.8ld\n", ActualPosition3, ActualVelocity3, ActualTorque3);
	// eprintf("AP6 : %d    AP5 : %d    AP4 : %d    AP3 : %d    AP2 : %d    AP1 : %d\n" , ActualPostion6,ActualPostion5,ActualPostion4,ActualPostion3,ActualPostion2,ActualPostion1);
	switch(isinit){
		case 0 :
			if (
				 (ActualPosition1<0x100000)&&(ActualPosition1>0x0)&&
				 (ActualPosition2<0x100000)&&(ActualPosition2>0x0)&&
				(ActualPosition3<0x100000)&&(ActualPosition3>0x0)&&
				 (ActualPosition4<0x100000)&&(ActualPosition4>0x0)&&
				 (ActualPosition5<0x100000)&&(ActualPosition5>0x0)&&
				 //(ActualPosition6<0x100000)&&(ActualPosition6>0x0)&&
				(1)
			)
			{
				// EnterMutex();
				TargetPosition1 = ActualPosition1;
				TargetPosition2 = ActualPosition2;
				TargetPosition3 = ActualPosition3;
				TargetPosition4 = ActualPosition4;
				TargetPosition5 = ActualPosition5;
				TargetPosition6 = ActualPosition6;
				ControlWord = SHUTDOWN;
				isinit = 1;
				// LeaveMutex();
			printf("Master :INITIALIZATION STEP 1 Controlword = SHUTDOWN\n");
			}
			else
			printf("ERROR:WRONG WORK AREA!!!!\n");
			break;		
		case 1 :
			// EnterMutex();
			ControlWord = SWITCH_ON;
			isinit = 2;
			printf("Master :INITIALIZATION STEP 2 Controlword = SWITCH_ON\n");
			// LeaveMutex();
			break;
		case 2 :
			// EnterMutex();
			ControlWord = ENABLE_OPERATION;
			isinit = 3;
			// LeaveMutex();
			printf("Master :INITIALIZATION STEP 3 Controlword = ENABLE_OPERATION\n");
			break;
		default :
			break;
		}

	// REPORT TO PC
	
		canopen_queue();
		if(count == 100){
			count = 0;
			//printf("TargetPosition1,ActualTar = %x,%x\n",TargetPosition1,ActualPosition1);
		
			gettimeofday(&tv,NULL);
			time_last = time_now;
			time_now = 1000000*tv.tv_sec+tv.tv_usec;
			time_use=time_now-time_last;
			//printf("time_use = %lld\n",time_use);
			}
   			

	//manage pvt_command
	if(control_mode == MOTION_PLAN){
		Enter_pvtqueue_Mutex();
		rec_state_change = get_pos_wrap(handle,ActualPosition1,ActualPosition2,
										ActualPosition3,ActualPosition4,
										ActualPosition5,ActualPosition6,rec_output);
		
		/*calculate velocity*/
		rec_output = calcuVelocity_wrap(handle,&TargetVelocity1,&TargetVelocity2,
											&TargetVelocity3,&TargetVelocity4,
											&TargetVelocity5,&TargetVelocity6);
		switch(rec_output){
			case 0 :
				break;
			case 1 :

				/*read actual position to calculate fedposition*/
				break;
			case -1 :
				printf("PVT_QUEUE_FINISHED!!\n");

				position_sum = ActualPosition1 - position_sum;

				printf("position_sum = %x\n", position_sum);

		        /*pop out traj control method*/
				trajCtrlDele_wrap(&handle);


				/*send end request*/
				if((rec_output=send(sock_fd,end_request,strlen(end_request),MSG_NOSIGNAL)) < 0){
                                printf("ERROR:Fail to send string\n");
                                close(sock_fd);
                                //exit(1);
                                tcp_connected = 0;
                                //return 0;
                            }
				rec_output = 0;
				rec_state_change = 0;
				
				/*destroy pvt command message queue from phy*/
				destory_pvt_queue(&pvt_command_queue);
				control_mode = COMMAND;
				break;			
			}
		Leave_pvtqueue_Mutex();
		
		}


	
}

void TestMaster_post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg)
{
	eprintf("Master received EMCY message. Node: %2.2x  ErrorCode: %4.4x  ErrorRegister: %2.2x\n", nodeID, errCode, errReg);
}

char query_result = 0;
char waiting_answer = 0;

static void CheckSDO(CO_Data* d, UNS8 nodeId)
{
	UNS32 abortCode;	
	if(getWriteResultNetworkDict (d, nodeId, &abortCode) != SDO_FINISHED)
		eprintf("Master : Failed in changing Slave's transmit type AbortCode :%4.4x \n", abortCode);
	/* Finalise last SDO transfer with this node */
	closeSDOtransfer(&TestMaster_Data, nodeId, SDO_CLIENT);
}

static int MasterSyncCount = 0;
void TestMaster_post_TPDO(CO_Data* d)
{
	// eprintf("TestMaster_post_TPDO MasterSyncCount = %d \n", MasterSyncCount);
	// if(MasterSyncCount % 17 == 0){
	// 	eprintf("Master : Ask RTR PDO (0x1402)\n");
	// 	sendPDOrequest(&TestMaster_Data, 0x1402 );
	// 	sendPDOrequest(&TestMaster_Data, 0x1403 );
	// }
	// if(MasterSyncCount % 50 == 0){
	// 	UNS8 transmitiontype;
	// 	eprintf("Master : Change slave's transmit type to 0xFF\n");
	// 	transmitiontype = 0xFF;
	// 	writeNetworkDictCallBack (&TestMaster_Data, /*CO_Data* d*/
	// 				2, /*UNS8 nodeId*/
	// 				0x1802, /*UNS16 index*/
	// 				0x02, /*UNS16 index*/
	// 				1, /*UNS8 count*/
	// 				0, /*UNS8 dataType*/
	// 				&transmitiontype,/*void *data*/
	// 				CheckSDO, /*SDOCallback_t Callback*/
    //                 0); /* use block mode */
	// }   
	// if(MasterSyncCount % 50 == 25){
	// 	UNS8 transmitiontype = 0x00;
	// 	eprintf("Master : Change slave's transmit type to 0x00\n");
	// 	writeNetworkDictCallBack (&TestMaster_Data, /*CO_Data* d*/
	// 				2, /*UNS8 nodeId*/
	// 				0x1802, /*UNS16 index*/
	// 				0x02, /*UNS16 index*/
	// 				1, /*UNS8 count*/
	// 				0, /*UNS8 dataType*/
	// 				&transmitiontype,/*void *data*/
	// 				CheckSDO, /*SDOCallback_t Callback*/
    //                 0); /* use block mode */
	// }
	// MasterSyncCount++;
}

void TestMaster_post_SlaveBootup(CO_Data* d, UNS8 nodeid)
{
	printf("---------------------------\n");
	eprintf("TestMaster_post_SlaveBootup %x\n", nodeid);
	eprintf("Master : ConfigureSlaveNode %2.2x\n", nodeid);
	ConfigureSlaveNode(d, nodeid);
}
