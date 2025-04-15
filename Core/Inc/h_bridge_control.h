/*
 * h_bridge_control.h
 *
 *  Created on: Apr 4, 2025
 *      Author: Le138
 */

#ifndef H_BRIDGE_CONTROL_H
#define H_BRIDGE_CONTROL_H

#define LB_F0_GPIOx GPIOF
#define LB_F0_GPIO_PIN GPIO_PIN_13
#define LB_F1_GPIOx GPIOF
#define LB_F1_GPIO_PIN GPIO_PIN_14

#define RB_F0_GPIOx GPIOE
#define RB_F0_GPIO_PIN GPIO_PIN_4
#define RB_F1_GPIOx GPIOE
#define RB_F1_GPIO_PIN GPIO_PIN_5

#define LF_F0_GPIOx GPIOC
#define LF_F0_GPIO_PIN GPIO_PIN_8
#define LF_F1_GPIOx GPIOC
#define LF_F1_GPIO_PIN GPIO_PIN_9

#define RF_F0_GPIOx GPIOG
#define RF_F0_GPIO_PIN GPIO_PIN_2
#define RF_F1_GPIOx GPIOG
#define RF_F1_GPIO_PIN GPIO_PIN_3

void lb_forward(void);
void lb_backward(void);
void lb_stop(void);

void rb_forward(void);
void rb_backward(void);
void rb_stop(void);

void lf_forward(void);
void lf_backward(void);
void lf_stop(void);

void rf_forward(void);
void rf_backward(void);
void rf_stop(void);

#endif /* H_BRIDGE_CONTROL_H */
