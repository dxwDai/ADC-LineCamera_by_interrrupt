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
#include "TFC_ADCCamera.h"





// #define ADC_SC1_ADCH_MASK      0x1Fu   
// #define ADC_SC1_ADCH(x)     (((uint32_t)(x))&ADC_SC1_ADCH_MASK)

/*
enum FSMADC            // Defines an enumeration type    
{
    init=-1,        
	  ADC_STATE_IDLE=0,
    ADC_STATE_CAPTURE_POT_0 = 1,     
	  ADC_STATE_CAPTURE_POT_1 =2,
	  ADC_STATE_CAPTURE_BATTERY_LEVEL=3,
    ADC_STATE_CAPTURE_LINE_SCAN = 4
} adcState;
*/

#define ADC_STATE_INIT							-1
#define ADC_STATE_IDLE							0
#define ADC_STATE_CAPTURE_POT_0			        1
#define ADC_STATE_CAPTURE_POT_1			        2
#define ADC_STATE_CAPTURE_BATTERY_LEVEL			3
#define ADC_STATE_CAPTURE_LINE_SCAN		      4

static uint8_t 	CurrentADC_State =	ADC_STATE_INIT;	

volatile uint16_t PotADC_Value[2];
volatile uint16_t BatSenseADC_Value;

volatile uint8_t CurrentLineScanPixel = 0;

volatile uint8_t cameraImageReady = 0;

short unsigned int imageData[128]; // a ping-pong buffer


void camInit(void)
{  
	//Initial the IO port
	// SI (PTD7) output, CLK (PTE1) output
	// AO (PTD5) input
	SIM->SCGC5 |=(0x1<<12 | 0x1<<13); // enable clock to Port D, E
	
    PORTD->PCR[7] = 0x100;     /* make PTD7 pin as GPIO */
    PTD->PDDR |= (0x1<<7);     /* make PTD7 as output pin */
	
    PORTE->PCR[1] = 0x100;     /* make PTE1 pin as GPIO */
    PTE->PDDR |= (0x1<<1);     /* make PTE1 as output pin */
	
		// PORTD->PCR[5] = 0x100;     /* make PTD5 pin as GPIO */
		// PTD->PDDR &= ~(0x1<<5);    /* make PTD5 as input pin */
		PORTD->PCR[5] = 0; // PTD5.MUX[10 9 8]=000, analog input
	
	
	  SIM->SCGC6 |= 0x08000000;   // enable clock to ADC0 ; 0x8000000u
	  //  SIM_SCGC6 |= (SIM_SCGC6_ADC0_MASK);
    
		
    // Configure ADC as it will be used, but because ADC_SC1_ADCH is 31,
    // the ADC will be inactive.  Channel 31 is just disable function.
    // There really is no channel 31.
	  // disable AIEN, Signle-ended, channel 31
    // ADC0->SC1[0] = AIEN_MASK|DIFF_SINGLE|ADC_SC1_ADCH(31);  
	  ADC0->SC1[0] = (~AIEN_MASK)& DIFF_SINGLE |ADC_SC1_ADCH(31);  
		
		ADC0->SC2 &= ~0x40;   // ADTRG=0, software trigger
	
	  // clock div by 4, long sample time, single ended 12 bit, bus clock 
    ADC0->CFG1 =(0x1<<6 | 0x1<<4 |0x1<<2); //0b01010100; 
	  
	  // select the B set of ADC input channels for camera input, PTD5 (SE6b)
	  ADC0->CFG2 |=(0x1<<4); //CFG2.MUXSEL=1, ADxxb channels are selected; 
	
	  CAM_SI_LOW;
	  CAM_CK_LOW;

}   
		

void TFC_InitADCInterruptMode()
{

	__disable_irq();    /* disable all IRQs */
	
	  NVIC->ICER[0] = 1 << 15; // disable IRQ15 (ADC0) interrupt
	
	  ADC0_init();
		camInit();
		TPM0_init();
	
		NVIC->ISER[0] |= 1u<<15;    // enable IRQ15 (bit 15 of ISER[0]) for ADC0 interrupt
		
		
    __enable_irq();             /* global enable IRQs */
	
	CurrentADC_State =	ADC_STATE_IDLE;	
	
}

	//All Adc processing of the Pots and linescan will be done in the ADC0 IRQ!
	//A state machine will scan through the channels.
	//This is done to automate the linescan capture on Channel 0 to ensure that timing is very even
void TFC_startReadingImage()
{
	// short unsigned int Junk;
	
	// start camera's integration immediately to guarantee the same integration time
	CAM_SI_HIGH;
	TPM0_Delay(1);
	CAM_CK_HIGH;
	TPM0_Delay(1);
	// for(Junk = 0;Junk<50;Junk++){}
	CAM_SI_LOW;
	
	CurrentLineScanPixel = 0;
	
  // ADC0_CFG2  |= ADC_CFG2_MUXSEL_MASK; //Select the B side of the mux for POT0
	ADC0->CFG2 |=(0x1<<4); //CFG2.MUXSEL=1, Select the B side of the mux
	
	// enable IRQ15 (bit 15 of ISER[0]) for ADC0 interrupt
	NVIC->ISER[0] |= 1u<<15;    
	
	// ADC0->SC1[0]  =  AIEN_MASK| ADC_SC1_ADCH(TFC_LINESCAN0_ADC_CHANNEL);
	ADC0_SC1A=  AIEN_MASK| ADC_SC1_ADCH(TFC_LINESCAN0_ADC_CHANNEL);
	
}


void ADC0_IRQHandler()
{
	short unsigned int Junk;
		
	if(CurrentLineScanPixel<128)
	{
		imageData[CurrentLineScanPixel] = ADC0_RA;
	        
		// for next pixel
		CurrentLineScanPixel++;
							
	  CAM_CK_LOW;
		for(Junk = 0;Junk<50;Junk++) {}
		CAM_CK_HIGH;
		ADC0_SC1A=  AIEN_MASK| ADC_SC1_ADCH(TFC_LINESCAN0_ADC_CHANNEL);
	}
		
	else
	{
						// done with the capture sequence.  we can wait for the PIT0 IRQ to restart
						Junk =  ADC0_RA;
					 // one more clock to ensure the camera return a known state
						// DXW: this was dow when CurrentLineScanPixel=127
						CAM_CK_HIGH; //??
											
						for(Junk = 0;Junk<50;Junk++){	}
						
						CAM_CK_LOW;
						cameraImageReady = TRUE;
				}
	
}

		
void ADC0_init(void)
{
	
    // initialization for PORTB PTB3 (ADC0_SE13) 
    SIM->SCGC5 |= 0x0400;  // enable clock to PORTB 
	  PORTB->PCR[3] = 0;         // PTB3.MUX[10 9 8]=000, analog input
    		
	  SIM->SCGC6 |= 0x08000000;   // enable clock to ADC0 
	
		// Configure ADC as it will be used, but because ADC_SC1_ADCH is 31,
    // the ADC will be inactive.  Channel 31 is just disable function.
    // There really is no channel 31.
	  // disable AIEN, Signle-ended, channel 31
    ADC0->SC1[0] = DIFF_SINGLE|ADC_SC1_ADCH(31);  
    ADC0->SC2 &= ~0x40;   // ADTRG=0, software trigger
  	
	/* clock div by 4, long sample time, single ended 12 bit, bus clock */
    ADC0->CFG1 =(0x1<<6 | 0x1<<4 |0x1<<2); //0b01010100; 
	  //  ADC0->CFG1 = 0x40 | 0x10 | 0x04 | 0x00;
	
    // select the A set of ADC input channels for PTD5 (SE6b)
	  ADC0->CFG2 &=~(0x1<<4); //CFG2.MUXSEL=0, ADxxa channels are selected; 
}

short int readADC(short ChID) 
{
	short int result;     	
	
	ADC0->SC1[0] = ChID; //software triger conversion on channel 13, SE13
	while(!(ADC0->SC1[0] & 0x80)) { } /* wait for conversion complete */
	result = ADC0->R[0];        /* read conversion result and clear COCO flag */
	return result;
}

// Initialize the TPM0 to generate a specified delay in number of MCGFLLCLK clocks
// By default, the MCGFLLCLK set by system setup is 20.97152MHz
void TPM0_init()
{
	
	SIM->SCGC6 |= (0x01<<24); // 0x01000000;, enale clk to TPM0
	SIM->SOPT2 |=(0x01<<24); // 0x01000000, use MCGFLLCLK as timer counter clk
	TPM0->SC = 0; // diable timer when configuring
	TPM0->MOD = 0xFFFF; // by default, set the 16-bit modulo value to maximum
	                    // thus results in maximum delay
	// TPM0->MOD = initMODvalue;
	
	TPM0->SC|=0x80; // clear TOF
	// TPM0->SC|=0x08; // enable timer free-rnning mode
}

// Initialize the TPM0 to generate a specified delay in number of MCGFLLCLK clocks
// By default, the MCGFLLCLK set by system setup is 20.97152MHz

// A signle delay for a pre-specified period. 
// The amount of delay should be specified bedore invoking TPM0_DelayOnce()
// by setting TPM0->MOD to an expected value
void TPM0_DelayOnce(void)
{
	TPM0->SC|=0x80; // clear TOF
	TPM0->SC|=0x08; // enable timer free-rnning mode
	while((TPM0->SC & 0x80) == 0) { } // wait until the TOF is set
	TPM0->SC = 0; // diable timer when configuring
}

void TPM0_Delay(float time_in_us)
{
  float t;	
	TPM0->SC = 0; // diable timer when configuring
	
	//Figure out how many clocks we need for for the delay time
	t=time_in_us*(float)(FRQ_MCGFLLCLK)/1000000.0; //MOD value for delay
	
	TPM0->MOD = (unsigned int) t;
	TPM0->SC|=0x80; // clear TOF
	TPM0->SC|=0x08; // enable timer free-rnning mode
	while((TPM0->SC & 0x80) == 0) { } // wait until the TOF is set
	TPM0->SC = 0; // diable timer when configuring
}