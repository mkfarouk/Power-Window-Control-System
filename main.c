#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "motor.h"





//**************************************************************************//
//																																					//
// 								Declarations																							//
//																																					//
//**************************************************************************//

void portF_init(void);
void portD_init(void);
void portA_init(void);
void portC_init(void);

void passenger_open_task(void *pvParameters);
void passenger_close_task(void *pvParameters);

void driver_open_task(void *pvParameters);
void driver_close_task(void *pvParameters);


SemaphoreHandle_t passenger_open_sem;
SemaphoreHandle_t passenger_close_sem;

SemaphoreHandle_t driver_open_sem;
SemaphoreHandle_t driver_close_sem;


SemaphoreHandle_t mutex;

xQueueHandle q_lock_flag;
xQueueHandle q_jam_flag;

xQueueHandle q_limit_open_flag;  
xQueueHandle q_limit_close_flag;  



portBASE_TYPE xStatus;
int lock_flag=0;
int jam_flag=0;

int limit_open_flag=0;
int limit_close_flag=0;


//**************************************************************************//
//																			//																		
// 									Main									//										        
//																			//																		
//**************************************************************************//




int main(void) {

	// PORTS INITIALLIZATION
		portF_init();
		portD_init();
		portA_init();
		portC_init();

	// MOTOR INITIALLIZATION
	
		motor_init();
	
	// SEMAPHORES CREATION
	
		passenger_open_sem = xSemaphoreCreateBinary();
		passenger_close_sem = xSemaphoreCreateBinary();
	
		driver_open_sem = xSemaphoreCreateBinary();
		driver_close_sem = xSemaphoreCreateBinary();

		mutex = xSemaphoreCreateMutex();

    // QUEUES CREATION
	
		q_lock_flag = xQueueCreate(1, sizeof(int));
		q_jam_flag = xQueueCreate(1, sizeof(int));
		
		q_limit_open_flag = xQueueCreate(1, sizeof(int));
		q_limit_close_flag = xQueueCreate(1, sizeof(int));

	//Queues initial overwrite
		
		xQueueSendToBack(q_lock_flag, &lock_flag , 0);
		xQueueSendToBack(q_jam_flag, &jam_flag , 0);

		xQueueSendToBack(q_limit_open_flag, &limit_open_flag , 0);
		xQueueSendToBack(q_limit_close_flag, &limit_close_flag , 0);

	// Tasks creation
    xTaskCreate(passenger_open_task, "PassengerOpenTask", configMINIMAL_STACK_SIZE, NULL, 6, NULL);
    
    xTaskCreate(passenger_close_task, "PassengerCloseTask", configMINIMAL_STACK_SIZE, NULL, 6, NULL);
		
	xTaskCreate(driver_open_task, "DriverOpenTask", configMINIMAL_STACK_SIZE, NULL, 7, NULL);

	xTaskCreate(driver_close_task, "DriverCloseTask", configMINIMAL_STACK_SIZE, NULL, 7, NULL);
    
    vTaskStartScheduler();
    
    while (1) {
        // Code should not reach here
    }
}





//**************************************************************************//
//																		    //
// 							Ports and interrupts   						    //
//									inits									//
//																			//
//**************************************************************************//



// tiva leds for testing
void portF_init(void) {
	
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5; 
    while(!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R5)); 
    
    // Set PF2 and PF3 as output for the blue and green LEDs
    GPIO_PORTF_DIR_R |= (1 << 2) | (1 << 3) | (1 << 1);
    
    GPIO_PORTF_DEN_R |= (1 << 2) | (1 << 3) | (1 << 1);
}



// driver , passenger window opening closing
void portD_init(void) {

    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R3;

    while(!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R3));
    
    // Configure Port D pin 0 (PD0) and pin 1 (PD1) as input for the buttons
    GPIO_PORTD_DIR_R &= ~((1 << 0) | (1 << 1) | (1 << 2) | (1 << 3)); // Clear bits 0 and 1 for input
    GPIO_PORTD_DEN_R |= (1 << 0) | (1 << 1) | ( 1 << 2 ) | (1 << 3 );    // Enable digital function
    
    // Enable internal pull-up resistors for PD0 and PD1
    GPIO_PORTD_PUR_R |= (1 << 0) | (1 << 1) | ( 1 << 2 ) | (1 << 3 );
    
    // Configure interrupt for PD0 and PD1 (GPIOD) for the buttons
    GPIO_PORTD_IS_R &= ~((1 << 0) | (1 << 1) | ( 1 << 2 ) | (1 << 3 ));  // Set edge-sensitive interrupt
    GPIO_PORTD_IBE_R &= ~((1 << 0) | (1 << 1)| ( 1 << 2 ) | (1 << 3 )); // Interrupt triggered on the falling edge
    GPIO_PORTD_ICR_R |= ((1 << 0) | (1 << 1) | ( 1 << 2 ) | (1 << 3 ));  // Clear interrupt flags
    GPIO_PORTD_IM_R |= ((1 << 0) | (1 << 1) | ( 1 << 2 ) | (1 << 3 ));   // Enable interrupt mask
    
    // Enable GPIO Port D interrupt in NVIC
    NVIC_EN0_R |= 1 << (INT_GPIOD - 16);
}


//lock and jam
void portA_init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R0;
    while(!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R0));
    GPIO_PORTA_DIR_R &= ~(1 << 2 | 1 << 3 ); 
    GPIO_PORTA_DEN_R |= (1 << 2 | 1 << 3);
	
	  GPIO_PORTD_PUR_R |= ( 1 << 2  | 1 << 3); //pull up 
	
    GPIO_PORTA_IS_R &= ~(1 << 2 | 1 << 3);
    GPIO_PORTA_IBE_R &= ~(1 << 2 | 1 << 3);
    GPIO_PORTA_ICR_R |= (1 << 2 | 1 << 3);
    GPIO_PORTA_IM_R |= (1 << 2 | 1 << 3);
    NVIC_EN0_R |= 1 << (INT_GPIOA - 16);
}


void portC_init(void) {
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R2;
    while(!(SYSCTL_PRGPIO_R & SYSCTL_PRGPIO_R2));
    
    // Configure Port C pin 4 and pin 5 as input
    GPIO_PORTC_DIR_R &= ~((1 << 4) | (1 << 5)); // Clear bits 4 and 5 for input
    GPIO_PORTC_DEN_R |= (1 << 4) | (1 << 5);    // Enable digital function
    
    // Enable internal pull-up resistors for Port C pin 4 and pin 5
    GPIO_PORTC_PUR_R |= (1 << 4) | (1 << 5);
    
    // Configure interrupt for Port C pin 4 and pin 5 (GPIOC)
    GPIO_PORTC_IS_R &= ~((1 << 4) | (1 << 5));  // Set edge-sensitive interrupt
    GPIO_PORTC_IBE_R &= ~((1 << 4) | (1 << 5)); // Interrupt triggered on the falling edge
    GPIO_PORTC_ICR_R |= ((1 << 4) | (1 << 5));  // Clear interrupt flags
    GPIO_PORTC_IM_R |= ((1 << 4) | (1 << 5));   // Enable interrupt mask
    
    // Enable GPIO Port C interrupt in NVIC
    NVIC_EN0_R |= 1 << (INT_GPIOC - 16);
}



//**************************************************************************//
//																		    //
// 													Passenger tasks		    //
//																			//
//**************************************************************************//


void passenger_open_task(void *pvParameters) {
	
		int lock_flag1 , limit_open_flag1;
		xSemaphoreTake(passenger_open_sem, 0);
		
		while(1) {						
					// Wait for semaphore signal
					if ( (xSemaphoreTake(passenger_open_sem, portMAX_DELAY) == pdTRUE) &&(xQueuePeek(q_lock_flag, &lock_flag1, 0) == pdTRUE) &&!lock_flag1){

									xSemaphoreTake(mutex, 0); // Take mutex to access shared resource

									
						
									motor_run(ANTICLOCKWISE);
									GPIO_PORTF_DATA_R |= (1 << 1); // Red led for anticlockwise
									vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for 3 second to detect manual or auto
									

									// Check if the button is still pressed
									if ((GPIO_PORTD_DATA_R & (1 << 0)) == 0) { // Button is still pressed (manual opening)
											

										while (((GPIO_PORTD_DATA_R & (1 << 0)) == 0)  && (xQueuePeek(q_limit_open_flag, &limit_open_flag1, 0) == pdTRUE) &&!limit_open_flag1 ) {
													// Button is still pressed
											}
										if(limit_open_flag1){
												limit_open_flag &=~(1<<0);
												xQueueOverwrite(q_limit_open_flag, &limit_open_flag);
										}
											// Button released
											vTaskDelay(pdMS_TO_TICKS(500));
											motor_stop();
											GPIO_PORTF_DATA_R &= ~(1 << 1); // Turn off the red LED

									} else {
										
										// automatic opening)
										while((xQueuePeek(q_limit_open_flag, &limit_open_flag1, 0) == pdTRUE) &&!limit_open_flag1){
																	
																	// toggle up the red LED (PF1) to indicate automatic opening
																	vTaskDelay(pdMS_TO_TICKS(1000)); 
																	GPIO_PORTF_DATA_R |= (1 << 1); 
																	vTaskDelay(pdMS_TO_TICKS(1000)); 
																	GPIO_PORTF_DATA_R &= ~(1 << 1); 
													}
													
													limit_open_flag &=~(1<<0);
													xQueueOverwrite(q_limit_open_flag, &limit_open_flag);
													vTaskDelay(pdMS_TO_TICKS(1000)); 
													motor_stop();
													GPIO_PORTF_DATA_R &= ~(1 << 1); // Turn off the red LED
											
								}

									xSemaphoreGive(mutex); // Release the mutex
									}

		}
}





void passenger_close_task(void *pvParameters) {
	
	int lock_flag2  , jam_flag2 , limit_close_flag1;
	
	xSemaphoreTake(passenger_close_sem, 0);
    while(1) {
        // Wait for semaphore signal
        if ( (xSemaphoreTake(passenger_close_sem, portMAX_DELAY) == pdTRUE) &&(xQueuePeek(q_lock_flag, &lock_flag2, 0) == pdTRUE) &&!lock_flag2) {
           
			xSemaphoreTake(mutex, 0); // Take mutex to access shared resource
			
			motor_run(CLOCKWISE);
			GPIO_PORTF_DATA_R |= (1 << 2); // blue led for clockwise

            vTaskDelay(pdMS_TO_TICKS(3000)); 

            // Check if the button is still pressed 
            if ((GPIO_PORTD_DATA_R & (1 << 1)) == 0) { // Button is still pressed (manual closing)
                while (((GPIO_PORTD_DATA_R & (1 << 1)) ==0)   && (xQueuePeek(q_limit_close_flag, &limit_close_flag1, 0) == pdTRUE) &&!limit_close_flag1) {
                    // Button is still pressed
                }
								if(limit_close_flag1){
												limit_close_flag &=~(1<<0);
												xQueueOverwrite(q_limit_close_flag, &limit_close_flag);
										}
								motor_stop();
								GPIO_PORTF_DATA_R &= ~(1 << 2); // Turn off the blue LED

								
            } else { // Button is released after 4 seconds (automatic closing)
							
							
							
							while((xQueuePeek(q_jam_flag, &jam_flag2, 0) == pdTRUE) && !jam_flag2   && (xQueuePeek(q_limit_close_flag, &limit_close_flag1, 0) == pdTRUE) &&!limit_close_flag1){
																	
																	vTaskDelay(pdMS_TO_TICKS(1000)); 
																
																	GPIO_PORTF_DATA_R |= (1 << 2); // Turn on the blue
																	vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second
																	GPIO_PORTF_DATA_R &= ~(1 << 2); // Turn off blue
													}
							
													if(jam_flag2){
													jam_flag &=~(1<<0);
													xQueueOverwrite(q_jam_flag, &jam_flag);
													
													motor_stop();
													GPIO_PORTF_DATA_R &= ~(1 << 2); // Turn off blue

													vTaskDelay(pdMS_TO_TICKS(2000)); // Wait for 1 second

													motor_run(ANTICLOCKWISE);
													GPIO_PORTF_DATA_R |= (1 << 1); // Turn on red

													vTaskDelay(pdMS_TO_TICKS(4000)); // Wait for 1 second
													motor_stop();	
													GPIO_PORTF_DATA_R &= ~(1 << 1); // Turn off red
							
													}
													
													else {
													
													limit_close_flag &=~(1<<0);
													xQueueOverwrite(q_limit_close_flag, &limit_close_flag);
														
													vTaskDelay(pdMS_TO_TICKS(1000)); 
														
													motor_stop();
													GPIO_PORTF_DATA_R &= ~( 1 << 2 );

													
													
													}
							
							
							
            }

            xSemaphoreGive(mutex); // Release the mutex
        }
    }
}




//**************************************************************************//
//																																					//
// 													Driver tasks																		//
//																																					//
//**************************************************************************//




void driver_open_task(void *pvParameters) {
	
	int limit_open_flag2;
	xSemaphoreTake(driver_open_sem, 0);
	
    while(1) {
        // Wait for semaphore signal
        if (xSemaphoreTake(driver_open_sem, portMAX_DELAY) == pdTRUE) {
            xSemaphoreTake(mutex, 0); // Take mutex to access shared resource

					
			motor_run(ANTICLOCKWISE);
			GPIO_PORTF_DATA_R |= (1 << 1); // red led on


            vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for 4 seconds

            // Check if the button is still pressed after 4 seconds
            if ((GPIO_PORTD_DATA_R & (1 << 2)) == 0) { // Button is still pressed (manual opening)
                while (((GPIO_PORTD_DATA_R & (1 << 2)) == 0 ) && (xQueuePeek(q_limit_open_flag, &limit_open_flag2, 0) == pdTRUE) &&!limit_open_flag2) {
                    // Button is still pressed, red LED stays on
                }
                // Button released, turn off the red LED
								if(limit_open_flag2){
										limit_open_flag &=~(1<<0);
										xQueueOverwrite(q_limit_open_flag, &limit_open_flag);
										}
								motor_stop();
								GPIO_PORTF_DATA_R &= ~(1 << 1); // turn off red led
								
            } else { // Button is released after 4 seconds (automatic opening)
							
							
							
							while((xQueuePeek(q_limit_open_flag, &limit_open_flag2, 0) == pdTRUE) &&!limit_open_flag2){
																	// Light up the red LED (PF1) to indicate automatic opening
																	GPIO_PORTF_DATA_R &= ~(1 << 1);
																	vTaskDelay(pdMS_TO_TICKS(1000)); 
																
																	GPIO_PORTF_DATA_R |= (1 << 1); // Turn on the red LED
																	vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second
																	GPIO_PORTF_DATA_R &= ~(1 << 1); // Turn off the red LED
													}
							

													
													limit_open_flag &=~(1<<0);
													xQueueOverwrite(q_limit_open_flag, &limit_open_flag);
													
													motor_stop();
													GPIO_PORTF_DATA_R &= ~(1 << 1); // red led off
																				
            }

            xSemaphoreGive(mutex); // Release the mutex
        }
    }
}


void driver_close_task(void *pvParameters) {
	
	int jam_flag4 , limit_close_flag2;
	
	xSemaphoreTake(driver_close_sem, 0);
    while(1) {
        // Wait for semaphore signal
        if (xSemaphoreTake(driver_close_sem, portMAX_DELAY) == pdTRUE) {
            xSemaphoreTake(mutex, 0); // Take mutex to access shared resource

					
			motor_run(CLOCKWISE);
			GPIO_PORTF_DATA_R |= (1 << 2); // blue led on
			

            vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for 4 seconds

            // Check if the button is still pressed after 4 seconds
            if ((GPIO_PORTD_DATA_R & (1 << 3)) == 0) { // Button is still pressed (manual closing)
                while (((GPIO_PORTD_DATA_R & (1 << 3)) == 0 ) && (xQueuePeek(q_limit_close_flag, &limit_close_flag2, 0) == pdTRUE) &&!limit_close_flag2) {
                    // Button is still pressed, LEDs stay on
                }
                // Button released
				
								if(limit_close_flag2){
										limit_close_flag &=~(1<<0);
										xQueueOverwrite(q_limit_close_flag, &limit_close_flag);
								}
								motor_stop();
								 GPIO_PORTF_DATA_R &= ~(1 << 2) ; // blue led off
								
            } else { // Button is released after 4 seconds (automatic closing)
							
							
							
							while((xQueuePeek(q_jam_flag, &jam_flag4, 0) == pdTRUE) && !jam_flag4  && (xQueuePeek(q_limit_close_flag, &limit_close_flag2, 0) == pdTRUE) &&!limit_close_flag2){
																	
								

																	vTaskDelay(pdMS_TO_TICKS(1000)); 
																
																	GPIO_PORTF_DATA_R |= (1 << 2); // Turn on the blue LED
																	vTaskDelay(pdMS_TO_TICKS(1000)); // Wait for 1 second
																	GPIO_PORTF_DATA_R &= ~(1 << 2); // Turn off the blue LED
													}
							
													if(jam_flag4){
														
													jam_flag &=~(1<<0);
													xQueueOverwrite(q_jam_flag, &jam_flag);
													
														
													vTaskDelay(pdMS_TO_TICKS(2000)); 
													motor_stop();
													GPIO_PORTF_DATA_R &= ~(1 << 2); // Turn off the blue LED
													
													vTaskDelay(pdMS_TO_TICKS(2000)); 

													motor_run(ANTICLOCKWISE);
													GPIO_PORTF_DATA_R |= (1 << 1); // Turn on the red LED
													
													vTaskDelay(pdMS_TO_TICKS(4000)); 
														
													motor_stop();	
													GPIO_PORTF_DATA_R &= ~(1 << 1); // Turn off the red LED	
														
													}
													
													else {
													
													limit_close_flag &=~(1<<0);
													xQueueOverwrite(q_limit_close_flag, &limit_close_flag);
														
													
													motor_stop();
													GPIO_PORTF_DATA_R &= ~(1 << 2); // Turn off the blue LED
													}
							
							
							
							
            }

            xSemaphoreGive(mutex); // Release the mutex
        }
    }
}






//**************************************************************************//
//																																					//
// 													GPIO interrupt Handlers												  //
//																																					//
//**************************************************************************//





// GPIO Port D interrupt handler for the buttons (PD0 and PD1)
void GPIOD_Handler(void) {
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    // Check if the interrupt is triggered by PD0 (Button connected to pin 0)
    if (GPIO_PORTD_MIS_R & (1 << 0)) {
        // Clear interrupt flag for PD0
        GPIO_PORTD_ICR_R |= (1 << 0);
        // Give semaphore to passenger_open_task
        xSemaphoreGiveFromISR(passenger_open_sem, &xHigherPriorityTaskWoken);
    }
    
    // Check if the interrupt is triggered by PD1 (Button connected to pin 1)
     if (GPIO_PORTD_MIS_R & (1 << 1)) {
        // Clear interrupt flag for PD1
        GPIO_PORTD_ICR_R |= (1 << 1);
        // Give semaphore to passenger_close_task
        xSemaphoreGiveFromISR(passenger_close_sem, &xHigherPriorityTaskWoken);
    }
		
		 if (GPIO_PORTD_MIS_R & (1 << 2)) {
        
        GPIO_PORTD_ICR_R |= (1 << 2);
        
        xSemaphoreGiveFromISR(driver_open_sem, &xHigherPriorityTaskWoken);
    }
		
		 if (GPIO_PORTD_MIS_R & (1 << 3)) {
        
        GPIO_PORTD_ICR_R |= (1 << 3);
        
        xSemaphoreGiveFromISR(driver_close_sem, &xHigherPriorityTaskWoken);
    }

		
}


//detect if lokc is on or off atm
void GPIOA_Handler(void) {

			portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    
    if (GPIO_PORTA_MIS_R & (1 << 2)) {
        GPIO_PORTA_ICR_R |= (1 << 2);
	

        lock_flag ^= (1<<0);
				xQueueOverwriteFromISR(q_lock_flag, &lock_flag, &xHigherPriorityTaskWoken);
    }
		
		 if (GPIO_PORTA_MIS_R & (1 << 3)) {
        GPIO_PORTA_ICR_R |= (1 << 3);
		

        jam_flag |= (1<<0);
				xQueueOverwriteFromISR(q_jam_flag, &jam_flag, &xHigherPriorityTaskWoken);
    }
    
		
}

void GPIOC_Handler(void) {
	
			portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    
    if (GPIO_PORTC_MIS_R & (1 << 4)) {
        GPIO_PORTC_ICR_R |= (1 << 4);
			

        limit_open_flag |= (1<<0);
				xQueueOverwriteFromISR(q_limit_open_flag, &limit_open_flag, &xHigherPriorityTaskWoken);
    }
		
		if (GPIO_PORTC_MIS_R & (1 << 5)) {
        GPIO_PORTC_ICR_R |= (1 << 5);
			

        limit_close_flag |= (1<<0);
				xQueueOverwriteFromISR(q_limit_close_flag, &limit_close_flag, &xHigherPriorityTaskWoken);
    }
		
		
    		
}