/*
 * h_bridge_control.c
 *
 *  Created on: Apr 4, 2025
 *      Author: Le138
 */
#include "h_bridge_control.h"
#include "main.h"


void lb_forward(void){
	HAL_GPIO_WritePin(LB_F0_GPIOx, LB_F0_GPIO_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LB_F1_GPIOx, LB_F1_GPIO_PIN, GPIO_PIN_RESET);
}

void lb_backward(void){
	HAL_GPIO_WritePin(LB_F0_GPIOx, LB_F0_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LB_F1_GPIOx, LB_F1_GPIO_PIN, GPIO_PIN_SET);
}

void lb_stop(void){
	HAL_GPIO_WritePin(LB_F0_GPIOx, LB_F0_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LB_F1_GPIOx, LB_F1_GPIO_PIN, GPIO_PIN_RESET);
}

void rb_forward(void){
	HAL_GPIO_WritePin(RB_F0_GPIOx, RB_F0_GPIO_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RB_F1_GPIOx, RB_F1_GPIO_PIN, GPIO_PIN_RESET);
}

void rb_backward(void){
	HAL_GPIO_WritePin(RB_F0_GPIOx, RB_F0_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RB_F1_GPIOx, RB_F1_GPIO_PIN, GPIO_PIN_SET);
}

void rb_stop(void){
	HAL_GPIO_WritePin(RB_F0_GPIOx, RB_F0_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RB_F1_GPIOx, RB_F1_GPIO_PIN, GPIO_PIN_RESET);
}
