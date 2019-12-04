// Lab10Main.c
// Runs on LM4F120/TM4C123
// Test the functions in ST7735.c by printing basic
// patterns to the LCD.
//    16-bit color, 128 wide by 160 high LCD
// Daniel Valvano
// Ramesh Yerraballi modified 3/20/2017
//
// Last Modified 12/4/2019: Drew Hardie & Ben Liu

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to Arm Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2019
 Copyright 2019 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// hardware connections
// **********ST7735 TFT and SDC*******************
// ST7735
// Backlight (pin 10) connected to +3.3 V
// MISO (pin 9) unconnected
// SCK (pin 8) connected to PA2 (SSI0Clk)
// MOSI (pin 7) connected to PA5 (SSI0Tx)
// TFT_CS (pin 6) connected to PA3 (SSI0Fss)
// CARD_CS (pin 5) unconnected
// Data/Command (pin 4) connected to PA6 (GPIO), high for data, low for command
// RESET (pin 3) connected to PA7 (GPIO)
// VCC (pin 2) connected to +3.3 V
// Gnd (pin 1) connected to ground

// **********wide.hk ST7735R with ADXL345 accelerometer *******************
// Silkscreen Label (SDC side up; LCD side down) - Connection
// VCC  - +3.3 V
// GND  - Ground
// !SCL - PA2 Sclk SPI clock from microcontroller to TFT or SDC
// !SDA - PA5 MOSI SPI data from microcontroller to TFT or SDC
// DC   - PA6 TFT data/command
// RES  - PA7 TFT reset
// CS   - PA3 TFT_CS, active low to enable TFT
// *CS  - (NC) SDC_CS, active low to enable SDC
// MISO - (NC) MISO SPI data from SDC to microcontroller
// SDA  – (NC) I2C data for ADXL345 accelerometer
// SCL  – (NC) I2C clock for ADXL345 accelerometer
// SDO  – (NC) I2C alternate address for ADXL345 accelerometer
// Backlight + - Light, backlight connected to +3.3 V

// **********wide.hk ST7735R with ADXL335 accelerometer *******************
// Silkscreen Label (SDC side up; LCD side down) - Connection
// VCC  - +3.3 V
// GND  - Ground
// !SCL - PA2 Sclk SPI clock from microcontroller to TFT or SDC
// !SDA - PA5 MOSI SPI data from microcontroller to TFT or SDC
// DC   - PA6 TFT data/command
// RES  - PA7 TFT reset
// CS   - PA3 TFT_CS, active low to enable TFT
// *CS  - (NC) SDC_CS, active low to enable SDC
// MISO - (NC) MISO SPI data from SDC to microcontroller
// X– (NC) analog input X-axis from ADXL335 accelerometer
// Y– (NC) analog input Y-axis from ADXL335 accelerometer
// Z– (NC) analog input Z-axis from ADXL335 accelerometer
// Backlight + - Light, backlight connected to +3.3 V

// **********HiLetgo ST7735 TFT and SDC (SDC not tested)*******************
// ST7735
// LED-   (pin 16) TFT, connected to ground
// LED+   (pin 15) TFT, connected to +3.3 V
// SD_CS  (pin 14) SDC, chip select
// MOSI   (pin 13) SDC, MOSI
// MISO   (pin 12) SDC, MISO
// SCK    (pin 11) SDC, serial clock
// CS     (pin 10) TFT, PA3 (SSI0Fss)
// SCL    (pin 9)  TFT, SCK  PA2 (SSI0Clk)
// SDA    (pin 8)  TFT, MOSI PA5 (SSI0Tx)
// A0     (pin 7)  TFT, Data/Command PA6 (GPIO), high for data, low for command
// RESET  (pin 6)  TFT, to PA7 (GPIO)
// NC     (pins 3,4,5) not connected
// VCC    (pin 2)  connected to +3.3 V
// GND    (pin 1)  connected to ground


//We need to detect how many blocks are directly below a block when it is falling
//#of blocks *33 needs to be added to the y coordinate of drawblocks2 ctrf F 90
#include <stdio.h>
#include <stdint.h>
#include "ST7735.h"
#include "PLL.h"
#include "../inc/tm4c123gh6pm.h"
#include "IO.h"
#include "Print.h"
#include "images.h"
#include "piano.h"
#include "random.h"
#include "dac.h"
#include "Sound.h"
#include "ADC.h"
#include "TExaS.h"


#define SIZE 16
int16_t prevx = 50;
int16_t prevy = 90;
int16_t prevgetx,x;
int16_t prevgety,y;
int16_t gravity=0;
uint8_t start = 0;
int16_t PCollision[4];
int16_t RightC[20];
int16_t LeftC[20];
int16_t TopC[20];
int16_t BottomC[20];
uint16_t BCount=0;
int16_t jump = 12;
int16_t testvar = 112;
uint32_t GPIO_PORTF_DATA_R2;
uint32_t GPIO_PORTE_DATA_R2;
uint8_t landed = 0;
int16_t landedY = 0;
uint8_t hug = 0;
uint16_t blocksize = 33;
uint16_t block_x = 90;
uint16_t block_y = 192;
uint16_t HighPoint[129] = {68};
uint16_t xBlock[200] = {0};
uint16_t yBlock[200] = {0};
uint16_t SizeBlock[200] = {0};
int indexes = 0;
uint16_t CCount=0;
uint16_t CCheck=0;
uint16_t CCheck2=0;
uint16_t xB[20];
uint16_t yB[20];
uint16_t wB[20];
uint16_t hB[20];
uint16_t shine = 0xFFFF;
uint8_t startG = 0;
uint8_t colorLock = 1;
uint8_t spanish =0;
uint32_t height;
uint32_t height2;
uint32_t maxheight=28;
uint32_t maxheight2=0;
uint8_t langcheck;
uint16_t xB2[8];
uint16_t yB2[8];
uint16_t wB2[8];
uint16_t hB2[8];
uint16_t xB3[8];
uint16_t yB3[8];
uint16_t wB3[8];
uint16_t hB3[8];
uint16_t xB4[20];
uint16_t yB4[20];
uint16_t wB4[20];
uint16_t hB4[20];
int16_t yFloor;
int16_t yFloor2;
uint16_t standard = 158;
//int16_t xvel;
//int16_t yvel;
int16_t yfinal[20];
int16_t stop=191;
uint16_t height5[20];
uint16_t height55;
uint16_t syscount;
uint16_t DCount=0;
int16_t prevblockx[20];
uint8_t pause=0;

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts



uint32_t const TestData[SIZE] ={
  0,7,9,10,99,100,409,654,999,1000,9999,10000,20806,65535,
  123400009,0xFFFFFFFF
};

/*void POSInit(void){
	uint16_t x = 50;
	uint16_t y = 90;
	return;
}*/

void delay(){
	uint32_t wait = 0xFFFFF;
	while(wait >= 0x05){
		wait--;
	}
	
	return;
}

void PortEInit(void){
	SYSCTL_RCGCGPIO_R |= 0x10;
//	SYSCTL_RCGC2_R |= 0x10;
//	SYSCTL_RCGC2_R+=0;
//	SYSCTL_RCGC2_R+=0;
	SYSCTL_RCGCGPIO_R+=0;
	SYSCTL_RCGCGPIO_R+=0;
//	GPIO_PORTB_AMSEL_R &= ~0xF;
//	GPIO_PORTB_PCTL_R &= ~0x0000FFFF;
//	GPIO_PORTB_AFSEL_R &= ~0xF;
	GPIO_PORTE_DIR_R &= ~0x0F;
	GPIO_PORTE_DEN_R |= 0x0F;
	return;
}

//void PortBInit2(void){
//	
//	SYSCTL_RCGCGPIO_R |= 0x02;
//	SYSCTL_RCGCGPIO_R+=0;
//	SYSCTL_RCGCGPIO_R+=0;
////	SYSCTL_RCGC2_R |= 0x02;
////	SYSCTL_RCGC2_R+=0;
////	SYSCTL_RCGC2_R+=0;
//	SYSCTL_RCGCGPIO_R+=0;
//	SYSCTL_RCGCGPIO_R+=0;
//	GPIO_PORTB_AMSEL_R &= ~0xF;
////	GPIO_PORTB_PCTL_R &= ~0x0000FFFF;
//	GPIO_PORTB_AFSEL_R &= ~0xF;
//	GPIO_PORTB_DIR_R &= ~0x0F;
//	GPIO_PORTB_DEN_R |= 0x0F;
//	return;
//}
//void PortBInit(void) {
// // --UUU-- Code to initialize PF4 and PF2
//	SYSCTL_RCGCGPIO_R |= 0x02;
//	SYSCTL_RCGCGPIO_R+=0;
//	SYSCTL_RCGCGPIO_R+=0;
//	//GPIO_PORTB_DIR_R |= 0xF;
//	GPIO_PORTB_DIR_R &= ~(0x0F);
//	GPIO_PORTB_DEN_R |= 0xF;
//	//GPIO_PORTF_AFSEL_R &= ~0x04; 
////	GPIO_PORTF_LOCK_R = GPIO_LOCK_KEY; // unlock PF2
////	GPIO_PORTF_CR_R |= 0x4;
//	GPIO_PORTB_PUR_R |= 0x0F;    // use pull up resistor for PF2

//	//  GPIO_PORTB_PCTL_R = (GPIO_PORTF_PCTL_R&0xFFFF0000)+0x00000000;
//  GPIO_PORTB_AMSEL_R = 0;
//	return;
//}


void SysTick_Init(void){
	NVIC_ST_CTRL_R = 0;
	NVIC_ST_RELOAD_R = 0x4FFFFF;	// set default reload value to max
	NVIC_ST_CURRENT_R = 0;
	NVIC_ST_CTRL_R = 0x05;
	 NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x80000000;

	NVIC_ST_CTRL_R = 0x07;
}

uint8_t CheckCollisionsx(){
		//put a for statement that increment topc, rightc and such once I get to multiple blocks
	if(((((PCollision[1])+7)%160)>=(xB[CCount]))&&((((PCollision[1])+7)%160)<=(xB[CCount]+wB[CCount]))){// || (PCollision[0]>=LeftC[0]-16)){
//		if(start==2){
//			start=3;
//		}
//	if(start==1){
//		start =2;
//	}
	return 1;
	}else{
//		if(((((PCollision[1]+7)%160))==(xB[CCount]))){
//			return 2;
//		}
		return 0;
	}
}


uint8_t CheckCollisionsx2(){
	//put a for statement that increment topc, rightc and such once I get to multiple blocks
	if(((((PCollision[0])-7)%160)<=(xB[CCount]+wB[CCount]))&&((((PCollision[0])-7)%160)>=(xB[CCount])) ){// || (PCollision[0]>=LeftC[0]-16)){
//		if(start==2){
//			start=3;
//		}
//	if(start==1){
//		start =2;
//	}
	return 1;
	}else{
//		if(((((PCollision[0]-7)%160))==(LeftC[0]))){
//			return 2;
//		}
		return 0;
	}
}

uint8_t CheckCollisionsy(){
	if(((((PCollision[2]-22))+gravity)<=(yB[CCount]))){
	return 1;
	}else{
		return 0;
	}
}

uint8_t checkBoundsx(){
	if(((PCollision[0]<=(xB[CCount]+wB[CCount]))&&(PCollision[0]>=xB[CCount])) || ((PCollision[1]>=xB[CCount])&& (PCollision[1] <=(xB[CCount]+wB[CCount])) )){
		return 1;
	}//endif
	else{
		return 0;
	}
	
}//endcheckbounds

uint8_t CheckFloor(){
	if(yFloor<50 || yFloor>80){
		return 0;
	}
	if(((((PCollision[3]))+gravity)<=(68))){
	return 1;
	}else{
		return 0;
	}
}

int16_t getx(){
prevx=x;
	if((GPIO_PORTE_DATA_R2 & 0x01) ==0x01){
		for(CCount=0;CCount<20;CCount++){
		if((CheckCollisionsx()==1) && CheckCollisionsy()==1){
			return xB[CCount]-13;
		}
	}//end for
		return (x+7);
	}//end button check
	if((GPIO_PORTE_DATA_R2 & 0x02) ==0x02){
		for(CCount=0;CCount<20;CCount++){
		if(CheckCollisionsx2()==1 && CheckCollisionsy()==1){
		return xB[CCount]+33;
		}
	}//end for
		return(x-7);
	}//end button check
	//xmove();
	return x;
}




int16_t gety(){
	prevy=y;
	if((landed==1)&&((GPIO_PORTE_DATA_R2&0x07)==0x00)){
		return yfinal[0]-68;
	}
	if((landed==1)&&((GPIO_PORTE_DATA_R2&0x04)==0x04)){
		gravity=jump;
					GPIO_PORTB_DATA_R ^= 0xF;
			GPIO_PORTB_DATA_R ^= 0xF;
			GPIO_PORTB_DATA_R ^= 0xF;
		landed=0;
	}
	else{
	if(CheckFloor()==1){
		if((GPIO_PORTE_DATA_R2&0x04)==0x04){
			gravity=jump;
						GPIO_PORTB_DATA_R ^= 0xF;
			GPIO_PORTB_DATA_R ^= 0xF;
			GPIO_PORTB_DATA_R ^= 0xF;
			landed=0;
		}//endcheckjump
		else{
			gravity=0;
			return 90;
		}
	}//endfloor
	for(CCount=0;CCount<20;CCount++){
		if((CheckCollisionsy()==1) && (checkBoundsx()==1)){
			ST7735_DrawBitmap(0,(yB[CCount]+13), SRowE, 40, 33);
			ST7735_DrawBitmap(40,(yB[CCount]+13), SRowE, 40, 33);
			ST7735_DrawBitmap(80,(yB[CCount]+13), SRowE, 40, 33);
			ST7735_DrawBitmap(120,(yB[CCount]+13), SRowE, 40, 33);
			if((GPIO_PORTE_DATA_R2&0x04)==0x04){
				gravity=jump;
							GPIO_PORTB_DATA_R ^= 0xF;
			GPIO_PORTB_DATA_R ^= 0xF;
			GPIO_PORTB_DATA_R ^= 0xF;
				landed=0;
				landedY=0;
			}else{
				gravity =0;
				if(landed==1){
					return yfinal[0]-68;
				}
				landed=1;
				return yfinal[0]-68;
			}
		}//endif
	}//endfor
	landed=0;
	landedY=0;
}
	gravity-=2;
	y+=gravity;
	return y;
}//endgety	
	
	

void RandomDelay() {
	//make random counter value
	//count down from that
	uint16_t RandCounter= Random32()%3000;
	
	RandCounter = RandCounter *100;
	
	while(RandCounter>0){
		RandCounter--;
	}
	return;
}
	
//void CheckHeight() {
//	int v = 0;
//	int16_t highest = 0;
//	while(v<=blocksize)	{			//collision check
//		if((HighPoint[block_x+blocksize-v])>highest) {
//			highest = HighPoint[block_x+blocksize-v];
//		}
//	v++;	
//	}		
//	height5 = highest;
//	height6=(10*(height5%10))+((height5-(height5%10))/10);
//}	
	//once block collides should be locked, x value placed in array
	//should redraw in for loop length of array all permanent blocks
	
void RedrawBlocks(uint8_t i){
	//ST7735_DrawBitmap(TopC[0],RightC[0], SBlockG, (LeftC[0]-RightC[0]), (TopC[0]-BottomC[0]));
	//ST7735_DrawBitmap(block_x,block_y, SBlockG,blocksize,blocksize);
	
	//ST7735_DrawBitmap(block_x,block_y+1-y,SBlockE ,blocksize,blocksize);		//ADD in block background sprite
	//ST7735_DrawBitmap(block_x,block_y-y, SBlockG,blocksize,blocksize);
	
	//ST7735_DrawBitmap(xB4[i],yB4[i], SBlockE, wB4[i], hB4[i]);
	
	if(height5[i]==0){
	ST7735_DrawBitmap(xB4[i],yB4[i]+gravity+(height5[i]*33), SBlockE, wB4[i], hB4[i]);
	ST7735_DrawBitmap(xB[i],yB[i]+(height5[i]*33), SBlockG, wB[i], hB[i]);
	}
	else{
	ST7735_DrawBitmap(xB4[i],yB4[i]+gravity+((1-height5[i])*33), SBlockE, wB4[i], hB4[i]);
	ST7735_DrawBitmap(xB[i],yB[i]+((1-height5[i])*33), SBlockG, wB[i], hB[i]);
	}
	
	if(height5[i]==2)
	{
		CCheck++;
	}
	//ST7735_DrawBitmap(xB4[0],yB4[0], SBlockE, wB4[0], hB4[0]);
	//ST7735_DrawBitmap(xB[0],yB[0], SBlockG, wB[0], hB[0]);
	
	//ST7735_DrawBitmap(90,101, SBlockG, 33, 33);
	return;
}
void ClearRow(){
	ST7735_DrawBitmap(0,101, SRowE, 40, 33);
	ST7735_DrawBitmap(40,101, SRowE, 40, 33);
	ST7735_DrawBitmap(80,101, SRowE, 40, 33);
	ST7735_DrawBitmap(120,101, SRowE, 40, 33);
	return;
}

void SetCollision(int16_t x1, int16_t y1, int16_t w, int16_t h, uint8_t i){
	RightC[i]=x;
	LeftC[i]=x+w;
	TopC[i]=y;
	BottomC[i]=y-h;
	xB4[i]=xB[i];
	yB4[i]=yB[i];
	wB4[i]=wB[i];
	hB4[i]=hB[i];
	xB[i]=x1;
	yB[i]=y1;
	wB[i]=w;
	hB[i]=h;
	return;
}

void RedrawBlocks2(){
	//ST7735_DrawBitmap(TopC[0],RightC[0], SBlockG, (LeftC[0]-RightC[0]), (TopC[0]-BottomC[0]));
	//ST7735_DrawBitmap(block_x,block_y, SBlockG,blocksize,blocksize);
	int f = 0;
	ST7735_DrawBitmap(block_x,block_y+1-y+90+gravity+(height55*33),SBlockE ,blocksize,blocksize);		//ADD in block background sprite
	ST7735_DrawBitmap(block_x,block_y-y+90+(height55*33), SBlockG,blocksize,blocksize);
	//SetCollision(block_x,block_y-y,blocksize,blocksize, BCount);
	
	while(xBlock[f] != 0){
		ST7735_DrawBitmap(xBlock[f],yBlock[f], SBlockG,SizeBlock[f],SizeBlock[f]);
		f++;
	}
	//ST7735_DrawBitmap(90,101, SBlockG, 33, 33);
	return;
}

void BlockSpawn(void) {			//function to randomize size of block, falling initiated in main
	//size: 33 by default.  Randomized range of 33 or 17, set width and height
	//initiate fall, subtracting from y values
	
	int temp = Random32()%2;			//random value 0 or 1
	//if(temp == 0) {								//converted to 33 or 17,set as both block width and height
		blocksize = 33;
	//}
	//else {
	//	blocksize =17;
	//}
	block_x = Random32()%160;			//sets random x value for the block to spawn
	prevblockx[DCount]=block_x;
	for(uint16_t i = 0; i<DCount; i++){
		if(prevblockx[DCount]==prevblockx[i]){
			height55++;
	}
	}
	if(height55!=0){
		height55-=1;
	}
	height5[DCount]=height55;
	DCount++;
	
	return;
}

	
void SetCollisionP(int16_t x, int16_t y, int16_t w, int16_t h){
	PCollision[0] = x;
	PCollision[1]=x+w;
	PCollision[2]=y;
	PCollision[3]=y-h;
	return;
}



void SetCollisionFloor(int16_t x, int16_t y, int16_t w, int16_t h){
	RightC[19]=x;
	LeftC[19]=x+w;
	TopC[19]=158-y;
	BottomC[19]=158-y-h;
	return;
}


void SetDrawFloor(int16_t x, int16_t y, int16_t w, int16_t h, uint8_t i){
	xB3[i]=xB2[i];
	yB3[i]=yB2[i];
	wB3[i]=wB2[i];
	hB3[i]=hB2[i];
	xB2[i]=160;
	yB2[i]=158-yFloor;
	wB2[i]=w;
	hB2[i]=h;
	return;
}
void fillEmpty(uint16_t BX){
	for(uint16_t i=0; i<=155; i+=33){
	ST7735_DrawBitmap(BX,i, SBlockE, blocksize, blocksize);
	}
}

void ReDrawFloor(){
//	for(uint8_t i=0; i<=7;i++){
//	SetDrawFloor(xB2[i],yFloor, wB2[i], hB2[i], i);
//	}
	for(uint8_t i=0; i<=7;i++){
	ST7735_DrawBitmap(20*i,68, FLOORE, 20, 68);
	}
	for(uint8_t i=0; i<=7;i++){
	ST7735_DrawBitmap(20*i,yFloor, FLOOR, 20, 68);
	}
	SetCollisionFloor(160,yFloor,160,68);
}

void DrawFloor(){
		for(testvar=0;testvar<=140;testvar+=20){
		ST7735_DrawBitmap(testvar,68, FLOOR, 20, 68); //20x68
		SetDrawFloor(testvar, y, 20, 68, ((testvar)/20));
		}
		SetCollisionFloor(160,158-y,160,68);
		return;
}

void setyfinal(int16_t yF, uint16_t i){
	yfinal[i]=yF;
	return;
}

  
void SysTick_Handler(void){
if(pause==0){	
	
	GPIO_PORTE_DATA_R2 = GPIO_PORTE_DATA_R;
	GPIO_PORTF_DATA_R2 = GPIO_PORTF_DATA_R;

	
	
	Random_Init(NVIC_ST_CURRENT_R);
	
	//shine-=0x0E;
//	if(shine<=0x0100){
//		shine = 0xFFEE;
//	}
	if(start==0){
		x=50;
		y=90;
		yFloor=(158-y-gravity);
		start = 1;
//		ST7735_DrawBitmap(90,191-y, SBlockG, 33, 33);
//		SetCollision(90,191-y,33,33, BCount);
		//SetCollision(90,191-y,33,33, BCount);
		
		//BCount++;
		DrawFloor();
		//
//		ST7735_DrawBitmap(80,68, FLOOR, 80, 68); //80x68
		SetCollisionP(prevx,y,13,22);
		ST7735_DrawBitmap(prevx,90,NGround, 13,22);
	}
	x = (getx());
	if(x<0){
		x=1599;
	}
	
	x= x%160;
	y = gety();
	yFloor2=yFloor;
	yFloor=158-y;
	GPIO_PORTF_DATA_R ^= 0x04;
	//ClearRow();
	if(prevgetx!=x){
	ST7735_DrawBitmap(prevgetx, 90, NEmpty, 13, 22); //displays the background over the players last location if they have moved still need to touch this up a little
	}else{
		if(prevgety!=y){
			ST7735_DrawBitmap(prevgetx, 90, NEmpty, 13, 22);
		}
	}
	if(x>prevgetx){
		if((gravity!=0)){
			ST7735_DrawBitmap2(x, 90, LAir, 13, 22, shine);//this is where it draws the player at their current location
			SetCollisionP(x,y,13,22);
		}else{																 //I still need to set up rightwards motion but I already have the sprites
		ST7735_DrawBitmap2(x, 90, LGround, 13, 22, shine);
		SetCollisionP(x,y,13,22);
		}
	}else{
		if(x<prevgetx){
			if((gravity!=0)){
			ST7735_DrawBitmap2(x, 90, RAir, 13, 22, shine);//this is where it draws the player at their current location
			SetCollisionP(x,y,17,23);
		}else{																 //I still need to set up rightwards motion but I already have the sprites
		ST7735_DrawBitmap2(x, 90, RGround, 13, 22, shine);
		SetCollisionP(x,y,17,23);
		}
		}else{
		if((gravity!=0)){
			ST7735_DrawBitmap2(x, 90, NAir, 13, 22, shine);
			SetCollisionP(x,y,13,22);
		}else{
	ST7735_DrawBitmap2(x, 90, NGround, 13, 22, shine);
	SetCollisionP(x,y,13,22);
	}
} 
		}
	GPIO_PORTB_DATA_R ^= 0xF;
	prevgetx = x;
	prevgety = y;
	
	
	height=y-68;
	height2=(10*(height%10))+((height-(height%10))/10);
	if(maxheight2<height){
		maxheight2=height;
		maxheight=(10*(height%10))+((height-(height%10))/10);
		
	}
	ST7735_DrawBitmap(0,160, CHeight, 40,20);
	ST7735_DrawBitmap(0,150, CHeight, 40,20);
	ST7735_SetCursor(2,14);
	LCD_OutFix(height2);
	ST7735_SetCursor(2,15);
	LCD_OutFix(maxheight);

	
	block_y--;
	
	
	//void CheckHeight();
	ReDrawFloor();
	RedrawBlocks2();				
	for(uint8_t i=0; i<20; i++){
	yB[i]=191-y+(height5[i]*33);
	SetCollision(xB[i], yB[i], wB[i], hB[i], i);
	for(uint8_t j=0; j<6; j++){
	RedrawBlocks(i);
	}
	}	//Redraw blocks (old and new one) based on info in BottomBlock,TopBlock, and SizeBlock arrays
		
	int16_t j=0;
	uint8_t exit=0;
	while((j<=blocksize)&&(exit==0))	{			//collision check
		if((block_y-blocksize) <= HighPoint[block_x+blocksize-j]) {				//collision protocol
			xBlock[indexes] = block_x;
			fillEmpty(block_x);                    
			yBlock[indexes] = block_y-y;
			SizeBlock[indexes] = blocksize;
			int k = 0;
			while(k<=blocksize) {
				HighPoint[block_x+blocksize-k] = block_y;
						k++;
			}
			
			SetCollision(block_x,block_y-y+33, blocksize,blocksize, BCount);
			RedrawBlocks(BCount);
			indexes++;
			
			
			block_y = 192+y;
			BlockSpawn();
			RedrawBlocks2();
			exit = 1;
			GPIO_PORTB_DATA_R ^= 0xF;
			GPIO_PORTB_DATA_R ^= 0xF;
			GPIO_PORTB_DATA_R ^= 0xF;
			setyfinal(stop,BCount);
			BCount++;
		}
		
			GPIO_PORTB_DATA_R ^= 0xF;
			//IF COLLIDES WITH PLAYER AND NOT BLOCK GAME OVER
		if(((block_y-(blocksize)+(height55*33)) <= y)&&((block_x+blocksize-j)==x)) {
			ST7735_FillScreen(0xCFFA);
			if(spanish==0){
			
			while(1){
			ST7735_SetCursor(9,10);
			ST7735_OutString("!!REVO EMAG");
			ST7735_SetCursor(5,8);
			ST7735_OutString(":EROCS");
			ST7735_SetCursor(6,7);
			LCD_OutFix(maxheight);
			}
			}else{
			while(1) {
			ST7735_SetCursor(2,10);
			ST7735_OutString("!!ODANIMRET OGEUJ");
			ST7735_SetCursor(5,8);
			ST7735_OutString(":LATOT");
			ST7735_SetCursor(6,7);
			LCD_OutFix(maxheight);
				
			}
			}
		}
		//if((block_y-blocksize) == //height of player) {
							//endgame protocol
			//game over
			
			//block_y = 128;	
				//rest of collision protocol
		j++;
	}
	j=0;
}
if((GPIO_PORTF_DATA_R &0x10)!=0x10){
	pause ^=1;
	delay();
}
	return;
}
//add 90 subtract y for displayed coordinate


void colorselect(){
				ST7735_DrawBitmap(120, 105, cursorclear, 2, 2);
		ST7735_DrawBitmap(120, 85, cursorclear, 2, 2);
		ST7735_DrawBitmap(120, 95, cursor, 2, 2);
		while((ADC_In() < 0xAA2)&&(ADC_In()> 0x551)){
		while((GPIO_PORTF_DATA_R & 0x10)!=0x10){
			ST7735_DrawBitmap2(30, 97, CBlock, 5, 5, shine);
			if(ADC_In() <(0x0551/2)){
				shine=0x7E0+(ADC_In()/40);
			}
			if((ADC_In() < 0x0551) && (ADC_In()>(0x0551/2))){
				shine=0x07FF+(ADC_In()/40);
			}
			if((ADC_In() < (0x551+(0x551/2)))&&(ADC_In()> 0x551) ){
				shine=0x139B+(ADC_In()/100);
			}
			if((ADC_In() < (0xAA2))&&(ADC_In()> (0x551+(0x551/2)))){
				shine=(0x7E0)-((ADC_In()/14)<<5);
				shine|=((ADC_In()/14)<<10);
			}
			if((ADC_In() > (0xAA2))&&(ADC_In()< (0xAA2+(0x551/2)))){
				shine=(0xF81F)+((ADC_In()/100)<<10);
			}
			if(ADC_In() >(0xAA2+(0x551/2))){
				shine=0xFFFF-(ADC_In()/0x100);
			}
		}
	return;	
	
}
		}

int main(void){  
  //uint32_t i;
	//POSInit();
	DisableInterrupts();
	for(int m = 0;m<128;m++) {
		HighPoint[m] = 68;
	}
	PLL_Init(Bus80MHz);    // set system clock to 80 MHz

  IO_Init();
//	Sound_Init();
	PortEInit();
	DAC_Init();
	ADC_Init();
	
  ST7735_InitR(INITR_REDTAB);
	IO_Touch();
	ST7735_FillScreen(0xCFFA);
//	ST7735_DrawBitmap(80,140,AVALA, 19, 4);
//	ST7735_DrawBitmap(64,140,NCHE, 16, 4);
	while(startG == 0){
	if(spanish==0){
	ST7735_SetCursor(8,14);
	ST7735_OutString("EHCNALAVA");
	ST7735_SetCursor(10,10);
	ST7735_OutString("EMAG TRATS");
	ST7735_SetCursor(8,9);
	ST7735_OutString("ROLOC ESOOHC");
	ST7735_SetCursor(13,8);
	ST7735_OutString("LONAPSE");
	}else{
		ST7735_SetCursor(8,14);
	ST7735_OutString("AHCNALAVA");
	ST7735_SetCursor(12,10);
	ST7735_OutString("OZNEIMOC");
	ST7735_SetCursor(8,9);
	ST7735_OutString("ROLOC RIGELE");
	ST7735_SetCursor(13,8);
	ST7735_OutString("HSILGNE");
	}
	if(ADC_In() < 0x551){
		ST7735_DrawBitmap(120, 85, cursorclear, 2, 2);
		ST7735_DrawBitmap(120, 95, cursorclear, 2, 2);
		ST7735_DrawBitmap(120, 105, cursor, 2, 2);
		while((ADC_In() < 0x551)&&(startG==0)){
		if((GPIO_PORTF_DATA_R & 0x10)!=0x10){
			startG =1;
		}
	}
	}
		if(startG==0){
		if((ADC_In() < 0xAA2)&&(ADC_In()> 0x551) ){
		colorselect();
		}
		if(ADC_In()>0xAA2){
		ST7735_DrawBitmap(120, 95, cursorclear, 2, 2);
		ST7735_DrawBitmap(120, 105, cursorclear, 2, 2);
		ST7735_DrawBitmap(120, 85, cursor, 2, 2);
		langcheck=spanish;
		while((ADC_In()>0xAA2) || langcheck==spanish ){
			if((GPIO_PORTF_DATA_R & 0x10)!=010){
			spanish ^= 0x01;
			ST7735_FillScreen(0xCFFA);
			}
		}
		}
		}//endif
	}
	
	ST7735_FillScreen(0xCFFA);
	SysTick_Init();
	EnableInterrupts();
	
	
	//	DisableInterrupts();
//  PLL_Init(Bus80MHz);    // set system clock to 80 MHz

//	delay();
//  IO_Init();
//	delay();
//	PortEInit();
//	//Piano_Init();
	
//	delay();
//  ST7735_InitR(INITR_REDTAB);
//	IO_Touch();
//	ST7735_FillScreen(0xCFFA); 
//	SysTick_Init();
//	EnableInterrupts();
  // test DrawChar() and DrawCharS()

/*	IO_Touch();
	ST7735_FillScreen(0xCFFA); 
	ST7735_DrawBitmap(50, 90, NGround, 13, 22);
	IO_Touch();
	ST7735_FillScreen(0xCFFA); 
	ST7735_DrawBitmapR(43, 90, RGround2, 19, 25);
	IO_Touch();
	ST7735_FillScreen(0xCFFA); 
	ST7735_DrawBitmapR(35, 90, RGround2, 19, 25);
	IO_Touch();
	ST7735_FillScreen(0xCFFA); 
	ST7735_DrawBitmapR(27, 90, RGround2, 19, 25);
	IO_Touch();
	ST7735_FillScreen(0xCFFA); 
	ST7735_DrawBitmapR(19, 90, RGround2, 19, 25);
	IO_Touch();
	ST7735_FillScreen(0xCFFA); 
	ST7735_DrawBitmap(61, 97, NAir, 13, 22);
	IO_Touch();
	ST7735_FillScreen(0xCFFA); 
	ST7735_DrawBitmap(61, 102, NAir, 13, 22);
	IO_Touch();
	ST7735_FillScreen(0xCFFA); 
	ST7735_DrawBitmap(63, 105, RAir, 17, 23);
	IO_Touch();
	ST7735_FillScreen(0xCFFA); 
	ST7735_DrawBitmap(66, 106, RAir, 17, 23);
	IO_Touch();
	ST7735_FillScreen(0xCFFA); 
	ST7735_DrawBitmap(69, 105, RAir, 17, 23);
	IO_Touch();
	ST7735_FillScreen(0xCFFA); 
	ST7735_DrawBitmap(69, 102, NAir, 13, 22);
	IO_Touch();
	ST7735_FillScreen(0xCFFA); 
	ST7735_DrawBitmap(69, 97, NAir, 13, 22);
	IO_Touch();
	ST7735_FillScreen(0xCFFA); 
	ST7735_DrawBitmap(69, 90, NGround, 13, 22);
	*/
  while(1){	
		EnableInterrupts();
		RedrawBlocks2();
		
		if((GPIO_PORTF_DATA_R &0x10)==0x10){
		DisableInterrupts();
		for(uint16_t i=0; i<400;i++){
		}
		while((GPIO_PORTF_DATA_R & 0x10) == 0x10){
			
		}
	}
		
		
	
	
	
	
	
	
	
	
	
	
	
		
		//after every subtraction of block_y if block_y equals any number in "top" array = COLLISION
		//if block collides, sets block_y to 0, goes back to main
		//stores bottom of block in "bottom" array and top of block in "top" array
		//block_x-size index in each array is highest y value with something present, array is 0 through 160.  
		//If top of block after collision is higher than top(block_x), 
		
		//for loop in redraw blocks to redraw everything
		//when scrolls up, clear array 
		
		

	/*	IO_Touch();
	ST7735_FillScreen(0xCFFA);
	IO_Touch();
	ST7735_FillScreen(0x001F;*/
  }
}
int main2(void){
	PLL_Init(Bus6_154MHz);

	ADC_Init();
	while(1)
	{
		GPIO_PORTF_DATA_R2=ADC_In();
	}
}


