
#include "MKL25Z4.h"
#include "TFC_Driver.h"

void TFC_init(void)
{
	// enable clock to PORTA,B, C, D, E 
  SIM->SCGC5 |= ((1U<<9)|(1U<<10)|(1U<<11)|(1U<<12)|(1U<<11));
	
	PORTC->PCR[13]=(1U << 8); // SW1, PTC13, digital IO
	PORTC->PCR[17]=(1U << 8); // SW2, PTC17, digital IO
	PORTC->ISFR &= (1U << 13);
	PORTC->ISFR &= (1U << 17);
	GPIOC->PDDR &=  (0U << 13);
	GPIOC->PDDR &=  (0U << 17);
	
	
	PORTB->PCR[2]=(0U << 8); // POT1, PTB2, analogue input
	PORTB->PCR[3]=(0U << 8); // POT2, PTB3, analogue input
	
	
	PORTD->PCR[5]=(0U << 8); // camera AO, ADC0_SE6b, 
	PORTD->PCR[6]=(0U << 8); // camera AO, ADC0_SE7b
	
	
}