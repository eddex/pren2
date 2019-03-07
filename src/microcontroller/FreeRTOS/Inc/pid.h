/*
 * pid.h
 *
 *  Created on: 14.12.2018
 *      Author: ANDREAS
 */

#ifndef PID_H_
#define PID_H_

#define Kp_v 1000 	// P-Anteil Geschwindigkeitsregler
#define Ki_v 10		// I-Anteil Geschwindigkeitsregler
//#define Kd_v 10		// D-Anteil Geschwindigkeitsregler
//#define Aw_v 1		// AntiWindup Geschwindigkeitsregler

#define Kp_p 100	// P-Anteil Positionsregler
#define Ki_p 10		// I-Anteil Positionsregler
//#define Kd_p 10		// D-Anteil Positionsregler
#define Aw_p 2000	// AntiWindup Positionsregler
#define MAX_VELO 60	// Maximaler Speed f�r Positionsregler

// Init Routine
void PID_Init();

// Geschwindigkeitsregler U/s
void PID_Velo(int32_t set_velo);

// Positionsregler
void PID_Pos(int32_t set_pos);

// Error reset
void clearError();

//F�r Zugriff auf Variable vom Main aus und FreeRTOS
void setPidEnable(uint8_t);
uint8_t getPidEnable(void);


#endif /* PID_H_ */
