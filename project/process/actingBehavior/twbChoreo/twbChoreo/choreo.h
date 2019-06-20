/*
 * choreo.h
 *
 *  Created on: May, 2016
 *      Author: mbarther
 */

#ifndef CHOREO_H_
#define CHOREO_H_

// define a type to access all values of the light ring
typedef std::array<std::array<int,3>,8> light_t;
// define a type to access all values of the position
typedef std::array<float,3> position_t;

// define a step in the choreography
typedef struct {
	int braking;
	position_t position;
	light_t lights;
	bool directMovement;
} ChoreoStep;

// define type for the choreography
typedef std::vector<ChoreoStep> Choreo;

#endif /* CHOREO_H_ */
