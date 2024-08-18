#ifndef MOTOR_H
#define MOTOR_H


#include <stdint.h>
#include <stdbool.h>
//#include "driverlib/gpio.h"
//#include "driverlib/sysctl.h"
//#include "driverlib/systick.h"
//#include "driverlib/interrupt.h"
//#include "driverlib/sysctl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define CLOCKWISE 0  // close window
#define ANTICLOCKWISE 1  // open window 

void motor_init(void);

void motor_run(int direction);

void motor_stop(void);


#endif