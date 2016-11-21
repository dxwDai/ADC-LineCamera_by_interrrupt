

#include "MKL25Z4.h"

#ifndef TFC_ADCCAMERA
  #define TFC_ADCCAMERA
	
	
	
#define TFC_POT_0_ADC_CHANNEL		13
#define TFC_POT_1_ADC_CHANNEL		12
#define TFC_BAT_SENSE_CHANNEL		4
#define TFC_LINESCAN0_ADC_CHANNEL	6
#define TFC_LINESCAN1_ADC_CHANNEL	7

#define FALSE 0
#define TRUE 1



#define DIFF_SINGLE 0x00
#define DIFF_DIFFERENTIAL (0x01<<5)

#define AIEN_MASK (0x01<<6)


#define CAM_SI_HIGH PTD->PSOR=(0x01<<7); //
#define CAM_SI_LOW PTD->PCOR=(0x01<<7);  //

#define CAM_CK_HIGH PTE->PSOR=(0x01<<1); //
#define CAM_CK_LOW PTE->PCOR=(0x01<<1);  //

#define FRQ_MCGFLLCLK 20971520 




void ADC0_init(void);
void camInit(void);

void TFC_InitADCInterruptMode(void);

void TFC_startReadingImage(void);

// void ADC0_IRQHandler(void);

short int readADC(short ChID);

void TPM0_init();
void TPM0_DelayOnce(void);
void TPM0_Delay(float time_in_us);


#endif // TFC_ADCCAMERA 