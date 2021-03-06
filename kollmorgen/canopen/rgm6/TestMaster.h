
/* File generated by gen_cfile.py. Should not be modified. */

#ifndef TESTMASTER2_H
#define TESTMASTER2_H

#include "data.h"

/* Prototypes of function provided by object dictionnary */
UNS32 TestMaster_valueRangeTest (UNS8 typeValue, void * value);
const indextable * TestMaster_scanIndexOD (UNS16 wIndex, UNS32 * errorCode, ODCallback_t **callbacks);

/* Master node data struct */
extern CO_Data TestMaster_Data;
extern UNS16 ControlWord;		/* Mapped at index 0x2000, subindex 0x00*/
extern UNS16 StatusWord1;		/* Mapped at index 0x2001, subindex 0x00*/
extern UNS16 StatusWord2;		/* Mapped at index 0x2002, subindex 0x00*/
extern UNS16 StatusWord3;		/* Mapped at index 0x2003, subindex 0x00*/
extern UNS16 StatusWord4;		/* Mapped at index 0x2004, subindex 0x00*/
extern UNS16 StatusWord5;		/* Mapped at index 0x2005, subindex 0x00*/
extern UNS16 StatusWord6;		/* Mapped at index 0x2006, subindex 0x00*/
extern INTEGER32 TargetPosition1;		/* Mapped at index 0x2011, subindex 0x00*/
extern INTEGER32 TargetPosition2;		/* Mapped at index 0x2012, subindex 0x00*/
extern INTEGER32 TargetPosition3;		/* Mapped at index 0x2013, subindex 0x00*/
extern INTEGER32 TargetPosition4;		/* Mapped at index 0x2014, subindex 0x00*/
extern INTEGER32 TargetPosition5;		/* Mapped at index 0x2015, subindex 0x00*/
extern INTEGER32 TargetPosition6;		/* Mapped at index 0x2016, subindex 0x00*/
extern INTEGER32 ActualPosition1;		/* Mapped at index 0x2021, subindex 0x00*/
extern INTEGER32 ActualPosition2;		/* Mapped at index 0x2022, subindex 0x00*/
extern INTEGER32 ActualPosition3;		/* Mapped at index 0x2023, subindex 0x00*/
extern INTEGER32 ActualPosition4;		/* Mapped at index 0x2024, subindex 0x00*/
extern INTEGER32 ActualPosition5;		/* Mapped at index 0x2025, subindex 0x00*/
extern INTEGER32 ActualPosition6;		/* Mapped at index 0x2026, subindex 0x00*/
extern INTEGER32 TargetVelocity1;		/* Mapped at index 0x2031, subindex 0x00*/
extern INTEGER32 TargetVelocity2;		/* Mapped at index 0x2032, subindex 0x00*/
extern INTEGER32 TargetVelocity3;		/* Mapped at index 0x2033, subindex 0x00*/
extern INTEGER32 TargetVelocity4;		/* Mapped at index 0x2034, subindex 0x00*/
extern INTEGER32 TargetVelocity5;		/* Mapped at index 0x2035, subindex 0x00*/
extern INTEGER32 TargetVelocity6;		/* Mapped at index 0x2036, subindex 0x00*/
extern INTEGER32 ActualVelocity1;		/* Mapped at index 0x2041, subindex 0x00*/
extern INTEGER32 ActualVelocity2;		/* Mapped at index 0x2042, subindex 0x00*/
extern INTEGER32 ActualVelocity3;		/* Mapped at index 0x2043, subindex 0x00*/
extern INTEGER32 ActualVelocity4;		/* Mapped at index 0x2044, subindex 0x00*/
extern INTEGER32 ActualVelocity5;		/* Mapped at index 0x2045, subindex 0x00*/
extern INTEGER32 ActualVelocity6;		/* Mapped at index 0x2046, subindex 0x00*/
extern INTEGER16 TargetTorque1;		/* Mapped at index 0x2051, subindex 0x00*/
extern INTEGER16 TargetTorque2;		/* Mapped at index 0x2052, subindex 0x00*/
extern INTEGER16 TargetTorque3;		/* Mapped at index 0x2053, subindex 0x00*/
extern INTEGER16 TargetTorque4;		/* Mapped at index 0x2054, subindex 0x00*/
extern INTEGER16 TargetTorque5;		/* Mapped at index 0x2055, subindex 0x00*/
extern INTEGER16 TargetTorque6;		/* Mapped at index 0x2056, subindex 0x00*/
extern INTEGER16 ActualTorque1;		/* Mapped at index 0x2061, subindex 0x00*/
extern INTEGER16 ActualTorque2;		/* Mapped at index 0x2062, subindex 0x00*/
extern INTEGER16 ActualTorque3;		/* Mapped at index 0x2063, subindex 0x00*/
extern INTEGER16 ActualTorque4;		/* Mapped at index 0x2064, subindex 0x00*/
extern INTEGER16 ActualTorque5;		/* Mapped at index 0x2065, subindex 0x00*/
extern INTEGER16 ActualTorque6;		/* Mapped at index 0x2066, subindex 0x00*/
extern INTEGER8 OperationMode1;		/* Mapped at index 0x2071, subindex 0x00*/
extern INTEGER8 OperationMode2;		/* Mapped at index 0x2072, subindex 0x00*/
extern INTEGER8 OperationMode3;		/* Mapped at index 0x2073, subindex 0x00*/
extern INTEGER8 OperationMode4;		/* Mapped at index 0x2074, subindex 0x00*/
extern INTEGER8 OperationMode5;		/* Mapped at index 0x2075, subindex 0x00*/
extern INTEGER8 OperationMode6;		/* Mapped at index 0x2076, subindex 0x00*/
extern UNS64 PVTbuff1;		/* Mapped at index 0x2081, subindex 0x00*/
extern UNS64 PVTbuff2;		/* Mapped at index 0x2082, subindex 0x00*/
extern UNS64 PVTbuff3;		/* Mapped at index 0x2083, subindex 0x00*/
extern UNS64 PVTbuff4;		/* Mapped at index 0x2084, subindex 0x00*/
extern UNS64 PVTbuff5;		/* Mapped at index 0x2085, subindex 0x00*/
extern UNS32 PVTbuff6;		/* Mapped at index 0x2086, subindex 0x00*/
extern UNS8 PVTalert1;		/* Mapped at index 0x2091, subindex 0x00*/
extern UNS8 PVTalert2;		/* Mapped at index 0x2092, subindex 0x00*/
extern UNS8 PVTalert3;		/* Mapped at index 0x2093, subindex 0x00*/
extern UNS8 PVTalert4;		/* Mapped at index 0x2094, subindex 0x00*/
extern UNS8 PVTalert5;		/* Mapped at index 0x2095, subindex 0x00*/
extern UNS8 PVTalert6;		/* Mapped at index 0x2096, subindex 0x00*/

#endif // TESTMASTER2_H
