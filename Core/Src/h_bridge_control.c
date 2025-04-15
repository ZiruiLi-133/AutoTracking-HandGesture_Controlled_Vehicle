/*
 * h_bridge_control.c
 *
 *  Created on: Apr 4, 2025
 *      Author: Le138
 */
#include "h_bridge_control.h"
#include "main.h"

//LB
void lb_forward(void){
	HAL_GPIO_WritePin(LB_F0_GPIOx, LB_F0_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LB_F1_GPIOx, LB_F1_GPIO_PIN, GPIO_PIN_SET);
}

void lb_backward(void){
	HAL_GPIO_WritePin(LB_F0_GPIOx, LB_F0_GPIO_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LB_F1_GPIOx, LB_F1_GPIO_PIN, GPIO_PIN_RESET);
}

void lb_stop(void){
	HAL_GPIO_WritePin(LB_F0_GPIOx, LB_F0_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LB_F1_GPIOx, LB_F1_GPIO_PIN, GPIO_PIN_RESET);
}

//RB
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

//LF
void lf_forward(void){
	HAL_GPIO_WritePin(LF_F0_GPIOx, LF_F0_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LF_F1_GPIOx, LF_F1_GPIO_PIN, GPIO_PIN_SET);
}

void lf_backward(void){
	HAL_GPIO_WritePin(LF_F0_GPIOx, LF_F0_GPIO_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LF_F1_GPIOx, LF_F1_GPIO_PIN, GPIO_PIN_RESET);
}

void lf_stop(void){
	HAL_GPIO_WritePin(LF_F0_GPIOx, LF_F0_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LF_F1_GPIOx, LF_F1_GPIO_PIN, GPIO_PIN_RESET);
}

//RF
void rf_forward(void){
	HAL_GPIO_WritePin(RF_F0_GPIOx, RF_F0_GPIO_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(RF_F1_GPIOx, RF_F1_GPIO_PIN, GPIO_PIN_RESET);
}

void rf_backward(void){
	HAL_GPIO_WritePin(RF_F0_GPIOx, RF_F0_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RF_F1_GPIOx, RF_F1_GPIO_PIN, GPIO_PIN_SET);
}

void rf_stop(void){
	HAL_GPIO_WritePin(RF_F0_GPIOx, RF_F0_GPIO_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(RF_F1_GPIOx, RF_F1_GPIO_PIN, GPIO_PIN_RESET);
}
