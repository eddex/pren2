#ifndef _DISTSENS_VL6180X_H_
#define _DISTSENS_VL6180X_H_


#define  finalDevAddress_VL6180X 0x29

void VL6180X_Init(void);
void measureDistanceValue(void);
int8_t getDistanceValue(void);



#endif
