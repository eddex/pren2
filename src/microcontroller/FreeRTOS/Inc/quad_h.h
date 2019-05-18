/*
 * quad.h
 *
 *  Created on: 08.12.2018
 *      Author: ANDREAS
 */

#ifndef QUAD_H_H_
#define QUAD_H_H_

#include <stdint.h>

// Enumeration
enum quad_h_e{
	sh00, sh01, sh10, sh11
};

// Init Routine
void Quad_H_Init();

// Returns Encoder Position
int32_t Quad_H_GetPos();

// Samples Encoder
void Quad_H_Sample();


#endif /* QUAD_H_H_ */
