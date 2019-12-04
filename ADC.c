// ADC.c
// Runs on LM4F120/TM4C123
// Provide functions that initialize ADC0
// Last Modified: 9/2/2019
// Student names: Drew Hardie & Dat Nguyen
// Last modification date: 11/12/2019

#include <stdint.h>
#include "../inc/tm4c123gh6pm.h"

// ADC initialization function 
// Input: none
// Output: none
// measures from PD2, analog channel 5

void delay3(){
	uint32_t wait = 0xFFFF;
	while(wait >= 0x05){
		wait--;
	}
	return;
}

void ADC_Init(void){ 
	SYSCTL_RCGCGPIO_R |= 0x08;
	SYSCTL_RCGCGPIO_R +=0;
	SYSCTL_RCGCGPIO_R +=0;
	delay3();
	GPIO_PORTD_DIR_R &= ~0x04;
	GPIO_PORTD_DEN_R &= ~0x04;
	GPIO_PORTD_AMSEL_R |= 0x04; //sets analog
	GPIO_PORTD_AFSEL_R |= 0x04; //sets alternate funciton
	SYSCTL_RCGCADC_R |= 0x01;//activates ADC 0
	SYSCTL_RCGCADC_R +=0;
	SYSCTL_RCGCADC_R+=0;
	delay3();
	ADC0_PC_R |= 0x01;//sets sampling rate to 125kHz
	delay3();
	ADC0_SSPRI_R = 0x0123;//sets sequencer3() to highest priority
	ADC0_ACTSS_R &= 0x08;//disables sample sequencer3 (ss3)
	ADC0_EMUX_R &= ~0xF000;//makes sequence3 s/w trigger
	ADC0_SSMUX3_R &= 0xFFFFFFF0;
	ADC0_SSMUX3_R += 5;
	ADC0_SSCTL3_R = 0x0006;
	ADC0_IM_R &= ~0x0008;//disables SS3 interrupts
	ADC0_ACTSS_R |= 0x08;//reenables ss3
	ADC0_SAC_R = 0x06;//helps sample accuracy but comment out in simulator
 }

//------------ADC_In------------
// Busy-wait Analog to digital conversion
// Input: none
// Output: 12-bit result of ADC conversion
// measures from PD2, analog channel 5
uint32_t ADC_In(void){  
	uint32_t value;
	ADC0_PSSI_R = 0x08; //initialize sample sequencer 3
	while((ADC0_RIS_R&0x08)==0){};//wait for conversion
	value=ADC0_SSFIFO3_R&0xFFF;//take result
	ADC0_ISC_R = 0x08; //done
	
  return value; // return the result
}


