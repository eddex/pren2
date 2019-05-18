/*
 * quad.h
 *
 *  Created on: 08.12.2018
 *      Author: ANDREAS
 */

#ifndef QUAD_V_H_
#define QUAD_V_H_

#include <stdint.h>

// Enumeration
enum quad_v_e{
	sv00, sv01, sv10, sv11
};

// Init Routine
void Quad_V_Init();

// Returns Encoder Position
int32_t Quad_V_GetPos();

// Samples Encoder
void Quad_V_Sample();


#endif /* QUAD_H_ */
