/*
 * pid.h
 *
 *  Created on: 14.12.2018
 *      Author: ANDREAS
 */

#ifndef PID_H_
#define PID_H_

// Geschwindigkeitsregler
#define Kp_v 30 	// P-Anteil Geschwindigkeitsregler
#define Ki_v 7		// I-Anteil Geschwindigkeitsregler
//#define Kd_v 10	// D-Anteil Geschwindigkeitsregler
#define Aw_v 100	// Anti Reset Windup Geschwindigkeitsregler

// Positionsregler
#define Kp_p 100	// P-Anteil Positionsregler
#define Ki_p 10		// I-Anteil Positionsregler
//#define Kd_p 10	// D-Anteil Positionsregler
#define Aw_p 2000	// Anti Rese Windup Positionsregler
#define MAX_VELO 60	// Maximaler Speed für Positionsregler

#define IN_POS_RANGE 5 // Range für InPosition Methode

// Init Routine
void PID_Init();

// Geschwindigkeitsregler mm/s
void PID_Velo(int32_t set_velo);

// Positionsregler mm
void PID_Pos(int32_t set_pos);

// Position erreicht
// 0 = Position nicht erreicht
// 1 = Position erreicht
uint8_t PID_InPos();

// Error reset
void PID_ClearError();

//Für Zugriff auf Variable vom Main aus und FreeRTOS
void PID_SetEnable(uint8_t);
uint8_t PID_GetEnable(void);


#endif /* PID_H_ */
