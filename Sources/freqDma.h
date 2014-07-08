/*
 * freqDma.h
 *
 *  Created on: Jul 2, 2014
 *      Author: Marco.HenryGin
 */

#ifndef FREQDMA_H_
#define FREQDMA_H_

/*************************************************************************
  *   $INCLUDES
*************************************************************************/
#include "PE_types.h"

/*************************************************************************
  *   $DEFINES
*************************************************************************/
#define NBR_FREQ_CHANNELS				1		/* It will be 4 */
#define NBR_DMA_CAPTURE_SAMPLES 100	/* 100 */
#define CAPTURE_CLOCK_HZ 1125000	// 1121549   /* Ideally should be 1.125MHz */

/*
 * Define the FTM register used for capture
 */
#define FREQ_TCAP_REG_CNT	FTM0_CNT
#define FREQ_TCAP_REG_CMP FTM0_C0V


#define FREQIN_DMA_MSK_NEW_FREQ 	0x01
#define FREQIN_DMA_MSK_HIGH_RANGE 0x02
#define FREQIN_DMA_MSK_OVF 				0x04





/*!
 * 	A structure for measuring frequency using DMA to capture pulses
 * 	
 * 	The DMA frequency measurement result is either a 32bit period ulPeriod (low 
 * 	to med frequency) or a 16bit quotient uPeriodNum/uPeriodDen when high
 * 	frequency range is detected fHighFreq.
 * 	
 * 	Every time we have a new reading either by pulse detection of input signal
 * 	or every half	the capture timer overflows, the flag fNewFreq is set.
 * 	   
 */
typedef struct
{

	uint16 dmaTblIndex, dmaTblIndexPrev;
	uint16 uMswCapTmr;
	uint32 uLastCapture;					// Last captured timer value
	
	union
	{
		uint32 ulTot;
		struct stPeriod
		{
			uint16 uNum, uDen;
		} sFraction;
		
	} sPeriod;
	
	bool		fNewFreq, fHighFreq, fCaptureInIsr, fDisplay;
	// uint16	*awDmaCaptureTbl;
	// The DMA transfer descriptor
	LDD_DMA_TTransferDescriptor tDmaTraDesc;
	
	
	
	
} stDmaFrequency;


/*************************************************************************
  *   $GLOBAL PROTOTYPES
*************************************************************************/

void freqDmaRun(void);		// The application
void timerCaptureIsr(void );


/*************************************************************************
  *   $GLOBAL VARIABLES
*************************************************************************/
extern unsigned int uCapture;			// Test 
extern unsigned int mSSamplePre, timerTicks;
extern LDD_TDeviceData *TimerPtr;
extern uint16 tPeriodIsr;
extern  stDmaFrequency atFreqIn[];

/*************************************************************************
  *   $INLINE FUNCTIONS 
  *************************************************************************/


#endif /* FREQDMA_H_ */
