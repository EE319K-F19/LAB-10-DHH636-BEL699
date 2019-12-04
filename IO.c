// IO.c
// This software configures the switch and LED
// You are allowed to use any switch and any LED, 
// although the Lab suggests the SW1 switch PF4 and Red LED PF1
// Runs on LM4F120 or TM4C123
// Date Created: March 30, 2018
// Last Modified:  12/2/2019: Drew Hardie & Ben Liu
// Lab number: 10

#include "../inc/tm4c123gh6pm.h"
#include <stdint.h>

//------------IO_Init------------
// Initialize GPIO Port for a switch and an LED
// Input: none
// Output: none
void IO_Init(void) {
 // --UUU-- Code to initialize PF4 and PF2
	SYSCTL_RCGCGPIO_R |= 0x20;
	SYSCTL_RCGCGPIO_R+=0;
	SYSCTL_RCGCGPIO_R+=0;
	GPIO_PORTF_DIR_R |= 0x04;
	GPIO_PORTF_DIR_R &= ~(0x10);
	GPIO_PORTF_DEN_R |= 0x14;
	GPIO_PORTF_AFSEL_R &= ~0x04; 
	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // unlock PF2
	GPIO_PORTF_CR_R |= 0x4;
	GPIO_PORTF_PUR_R |= 0x14;    // use pull up resistor for PF2
  GPIO_PORTF_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFF0F0FF)+0x00000000;
  GPIO_PORTF_AMSEL_R = 0;
}

//------------IO_HeartBeat------------
// Toggle the output state of the  LED.
// Input: none
// Output: none
void IO_HeartBeat(void) {
 // --UUU-- PF2 is heartbeat
	GPIO_PORTF_DATA_R ^= 0x04;
}


//------------IO_Touch------------
// wait for release and press of the switch
// Delay to debounce the switch
// Input: none
// Output: none
void IO_Touch(void) {
//	uint32_t i=0;
//	while(1){
//	if((GPIO_PORTF_DATA_R & 0x10) == 0){
//		for(i = 1600000; i>0;i--)
//	{
//	}
//	i=0;
//		while(i==0){
//			if((GPIO_PORTF_DATA_R & 0x10) != 0){
//				i++;
//			}
//		}
//			
//	}
//}
	while((GPIO_PORTF_DATA_R & 0x10) != 0){
	}
	
	for(uint32_t i = 1600000; i>0;i--)
	{
	}
	
	while((GPIO_PORTE_DATA_R & 0x10) != 0){
	}
	
 // --UUU-- wait for release; delay for 20ms; and then wait for press
}  
