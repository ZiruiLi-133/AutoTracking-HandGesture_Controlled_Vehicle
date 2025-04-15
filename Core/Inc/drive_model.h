/*
 * drive_model.h
 *
 *  Created on: Apr 4, 2025
 *      Author: Le138
 */

#ifndef INC_DRIVE_MODEL_H_
#define INC_DRIVE_MODEL_H_

void compute_ideal_speed(volatile float v, volatile float w,
		volatile float *lb_speed, volatile float *rb_speed,
		volatile float *lf_speed, volatile float *rf_speed);

#endif /* INC_DRIVE_MODEL_H_ */
