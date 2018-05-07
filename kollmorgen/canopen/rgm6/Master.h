#ifndef MASTER_H
#define MASTER_H

#include "TestMaster.h"
#include "rgm_src.h"
#include <sys/time.h>


#define PROFILE_POSITION_MODE   0x01
#define PROFILE_VELOCITY_MODE   0x03
#define PROFILE_TORQUE_MODE     0x04
#define HOMING_MODE             0x06
#define INTERPOLATED_POSITION_MODE  0x07
#define CYCLIC_POSITION_MODE    0x08
#define CYCLIC_VELOCITY_MODE    0x09
#define CYCLIC_TORQUE_MODE      0x0A
#define CONFIG_0_MODE           0x9E
#define CONFIG_1_MODE           0xDE
#define NONE_MODE               0xFF

#define RESET_FAULT             0x80
#define DISABLE_VOLTAGE         0x04
#define SHUTDOWN                0x06
#define SWITCH_ON               0x07
#define ENABLE_OPERATION        0x0F
#define STOP                    0x02
#define START_HOMEING           0x1F
#define END_HOMING              0x0F

void TestMaster_heartbeatError(CO_Data* d, UNS8);

UNS8 TestMaster_canSend(Message *);
extern INTEGER8 Operation_Mode ;

void TestMaster_initialisation(CO_Data* d);
void TestMaster_preOperational(CO_Data* d);
void TestMaster_operational(CO_Data* d);
void TestMaster_stopped(CO_Data* d);

void TestMaster_post_sync(CO_Data* d);
void TestMaster_post_TPDO(CO_Data* d);
void TestMaster_post_emcy(CO_Data* d, UNS8 nodeID, UNS16 errCode, UNS8 errReg);
void TestMaster_post_SlaveBootup(CO_Data* d, UNS8 nodeid);
extern int isinit;
extern int step_pvt;
extern int position_sum;

#endif // MASTER_H

