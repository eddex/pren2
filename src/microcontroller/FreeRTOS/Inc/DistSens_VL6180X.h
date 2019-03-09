#ifndef _DISTSENS_VL6180X_H_
#define _DISTSENS_VL6180X_H_

#include "fsm.h"

#define  finalDevAddress_VL6180X 0x29

taskState_t VL6180X_Init(void);
taskState_t measureDistanceValue(void);
int8_t getDistanceValue(void);



#endif
