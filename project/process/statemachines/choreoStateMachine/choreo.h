/*
 * choreo.h
 *
 *  Created on: Mar 10, 2015
 *      Author: florian
 */

#ifndef CHOREO_H_
#define CHOREO_H_

// define a type to access all values of the light ring
typedef std::array<std::array<int,3>,8> light_t;

// define a step in the choreography
typedef struct {
	int time;
	int v;
	int w;
	int brightness;
	light_t lights;
} ChoreoStep;

// define type for the choreography
typedef std::vector<ChoreoStep> Choreo;

#endif /* CHOREO_H_ */
