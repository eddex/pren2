/*
 * RadioModule.h
 *
 *  Created on: 20.01.2019
 *      Author: Jan
 */

#ifndef RADIOMODULE_H_
#define RADIOMODULE_H_


void sendSensorDatatoRadioModule(void);
uint8_t getDrehrichtung(void);
void setDrehrichtung(uint8_t);
uint16_t getfinalVelocity(void);
void setfinalVelocity(uint16_t);

uint8_t getSpeedGroupValue(void);
void setSpeedGroupValue(uint8_t groupValue);

#endif /* RADIOMODULE_H_ */
