/*
 * freefall.h
 *
 *  Created on: Feb 11, 2018
 *      Author: eric_
 */

#ifndef FREEFALL_H_
#define FREEFALL_H_
typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
}accelerometer_t;
void startI2C();
void IMUcapture(accelerometer_t *rawdata);
void freefallCalculation(int16_t x, int16_t y, int16_t z);
void ledRoutine();



#endif /* FREEFALL_H_ */
