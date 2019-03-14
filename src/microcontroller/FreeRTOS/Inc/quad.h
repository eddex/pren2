/*
 * quad.h
 *
 *  Created on: 08.12.2018
 *      Author: ANDREAS
 */

#ifndef QUAD_H_
#define QUAD_H_

// Enumeration
enum quad_e{
	s00, s01, s10, s11
};

// Init Routine
void Quad_Init();

// Returns Encoder Position mm
int32_t Quad_GetPos();

// Samples Encoder
void Quad_Sample();


#endif /* QUAD_H_ */
