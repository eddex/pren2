/*
 * pid.h
 *
 *  Created on: 14.12.2018
 *      Author: ANDREAS
 */

#ifndef PID_H_
#define PID_H_

#define Kp_v 200 	// P-Anteil Geschwindigkeitsregler
#define Ki_v 3		// I-Anteil Geschwindigkeitsregler
//#define Kd_v 10	// D-Anteil Geschwindigkeitsregler
#define Aw_v 10000	// Anti Reset Windup Geschwindigkeitsregler

#define Kp_p 100	// P-Anteil Positionsregler
#define Ki_p 10		// I-Anteil Positionsregler
//#define Kd_p 10	// D-Anteil Positionsregler
#define Aw_p 2000	// Anti Rese Windup Positionsregler
#define MAX_VELO 60	// Maximaler Speed für Positionsregler

// Init Routine
void PID_Init();

// Geschwindigkeitsregler mm/s
void PID_Velo(int32_t set_velo);

// Positionsregler mm
void PID_Pos(int32_t set_pos);

// Error reset
void PID_ClearError();

//Für Zugriff auf Variable vom Main aus und FreeRTOS
void PID_SetEnable(uint8_t);
uint8_t PID_GetEnable(void);


#endif /* PID_H_ */
