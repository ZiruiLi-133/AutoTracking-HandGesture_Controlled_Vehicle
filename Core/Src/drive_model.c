/*
 * drive_model.c
 *
 *  Created on: Apr 4, 2025
 *      Author: Le138
 */

#include "drive_model.h"
#include "chassis_param.h"

void compute_ideal_speed(volatile float v, volatile float w, volatile float *lb_speed, volatile float *rb_speed) {
	*lb_speed = v - w * CHASSIS_WIDTH / 2.0;
	*rb_speed = v + w * CHASSIS_WIDTH / 2.0;
}
