#ifndef MOTOR_H_
#define MOTOR_H_

#include "Ifx_Types.h"

void Motor_init(void);
void Motor_stopLeft(void);
void Motor_stopRight(void);
void Motor_driveLeft(uint8 dir, uint8 duty);
void Motor_driveRight(uint8 dir, uint8 duty);

#endif /* MOTOR_H_ */
