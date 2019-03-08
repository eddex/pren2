/*
 * fsm.h
 *
 *  Created on: 02.03.2019
 *      Author: Jan
 */

#ifndef FSM_H_
#define FSM_H_


enum fsm{
	STARTUP,
	WURFEL_ERKENNEN,
	WURFEL_VORFAHREN,
	SERVO_RUNTER,
	SERVO_RAUF,
	WURFEL_ZURUCKFAHREN,
	STARTPOSITION,
	SCHNELLFAHRT,
	BREMSEN,
	FINALES_HALTESIGNAL,
	HALTESIGNAL_ANFAHREN,
	HALTESIGNAL_STOPPEN,
	STOP
};

typedef enum {
	TASK_OK, TASK_ERROR, TASK_TIME_OVERFLOW
}taskState;


taskState wurfel_erkennen(void);

taskState haltesignal_erkennen(void);

#endif /* FSM_H_ */
