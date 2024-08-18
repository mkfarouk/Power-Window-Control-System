#include "motor.h"
#include <stdint.h>
#include "tm4c123gh6pm.h" // Assuming TM4C123 microcontroller


#define MOTOR_PORT_BASE GPIO_PORTB_DATA_R
#define MOTOR_PIN_0 (1 << 0)   // for IN1 
#define MOTOR_PIN_1 (1 << 1)   // for IN2

void motor_init(void) {
    // Enable Port B clock
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;
    // Wait until Port B is ready
    while(!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R1));

    // Configure motor pins as outputs
    GPIO_PORTB_DIR_R |= MOTOR_PIN_0 | MOTOR_PIN_1;
    // Enable digital function for motor pins
    GPIO_PORTB_DEN_R |= MOTOR_PIN_0 | MOTOR_PIN_1;
}

void motor_run(int direction) {
    // CLOCKWISE MEANS CLOSING
    if (direction == CLOCKWISE){
        MOTOR_PORT_BASE |= MOTOR_PIN_0; // Set IN1 high
        MOTOR_PORT_BASE |= ~ MOTOR_PIN_1; // Set IN2 low
    }
    // ANTICLOCKWISE MEANS OPENING
    else {
        MOTOR_PORT_BASE &= ~MOTOR_PIN_0; // Set IN1 low
        MOTOR_PORT_BASE |= MOTOR_PIN_1;  // Set IN2 high
    }
}

void motor_stop(void) {
    // Stop the motor by setting both pins low
    MOTOR_PORT_BASE &= ~(MOTOR_PIN_0 | MOTOR_PIN_1);
}
