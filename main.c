//***************************************************************************
// * 
// * Created on:  01 11 2016
// * Author:      Xuweu Dai  (xuewu.dai at northumbria.ac.uk)
// *
// * File:        Example Programme ADC at KL25Z
// *
// * This program converts the analog input from ADC channel 13 (PTB3) 
// * using software trigger continuously.
// * PTB3 is connect to the potentiometer POT1 at the TFC-Shield board.
// * When sdjusting POT1, the voltage of PTB3 varies from 0 volts to 3.3 volts.
// * The value of this voltage is used to control the tri-color LEDs. 
// * When the potentiometer is turned, the LEDs colour changes.
// * At the same time, the result of the each A/D conversionn is also sent to 
// * the PC and through the UART0 and displayed at the PC's terminal windows.
// *
// * Copyright:   (C) 2016 Northumbria University, Newcastle upon Tyne, UK.
// *
// * This program is free software: you can redistribute it and/or modify
// * it under the terms of the GNU Lesser General Public License as published by
// * the Free Software Foundation, either version 3 of the License, or
// * (at your option) any later version.
// 
// * This program is distributed in the hope that it will be useful,
// * but WITHOUT ANY WARRANTY; without even the implied warranty of
// * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// * GNU Lesser General Public License for more details.
// *
// %              _
// %  \/\ /\ /   /  * _  '
// % _/\ \/\/ __/__.'(_|_|_
// **************************************************************************/


#include "MKL25Z4.h"
#include "stdio.h"

#include "LEDDriver.h"
#include "UART0_TxRx.h"
#include "TFC_Driver.h"
#include "TFC_ADCCamera.h"

extern short unsigned int imageData[128]; // a ping-pong buffer
extern volatile uint8_t cameraImageReady;


int main (void)
{
    short int adcPOT1;  // POT one
    
	  char buf [100];   // UART buffer 
    int n,i,j;            // number of characters in b2097uf to be sent
	
    int nImgRd; // number of image reading (how many times has the image been red) 
		
	short unsigned int initMODValue, TPMMOD1us, TPMMOD_;
	float timeExposure; // delay for exposure time in us 
	
	  UART0_init();  // Initialized UART0, 57600 baud
    LED_init();                     /* Configure LEDs */
	  
	 	TFC_init();   // configure switch at TFC board
	 
	  // configure ADC0, camera (interrupt mode) and TPM)
	  TFC_InitADCInterruptMode();
	
	  timeExposure=50;
	  nImgRd=0;
	
	  // initialize TPM0 for 0.5 us delay
	  // initMODValue=(short unsigned int)(0.5*(float)(FRQ_MCGFLLCLK)/1000000.0); //MOD value for 1us delay
	  
    sendBootMsg();
		sendHelloWorld();
		
		
    while (1) {

			n = sprintf(buf, "> Press button A to start capturing an image \r\n>"); // convert integer value into ASCII
  		sendStr(buf, n);
	  
				while((GPIOC_PDIR & 0x0002000)==0) {}; // wait for SW1 pressed
					
			//	adcPOT1=readADC(13);
			 // adcPOT1=readADC(6);
//			  LED_set(adcPOT1); /* display the voltage range on LED */
			  // n = sprintf(buf, "POT1=%d\r\n", adcPOT1); // convert integer value into ASCII
			  sendStr("Detecting Button A pressed\r\n", 28);

					
			// read 1 images
			nImgRd++;
			// cam_ReadImage(&imageData[1][0]);
			TFC_startReadingImage();
		  
			// wait for impage cpature finish			
			while(cameraImageReady==FALSE) {};
				
				
			// send the images data to PC
			for(j=0;j<1;j++)
			{
				n=sprintf(buf, "nImgRd%d=[\r\n", nImgRd+j);
			  sendStr(buf, n);	
				for (i=0;i<128;i++)
			    {	n = sprintf(buf, "%d ", imageData[i]); // convert integer value into ASCII
			      sendStr(buf, n);
			    }
			  sendStr("]\r\n", 3);
				 sendStr("\r\n", 2);
		 }
			
    }
}
