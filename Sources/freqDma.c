/*
 * FreqDma.c
 *
 *  Created on: Jul 2, 2014
 *      Author: Marco.HenryGin
 */
//==============================================================================
//  INCLUDES
//==============================================================================
// Typical PEX include
#include "Cpu.h"
#include "Events.h"
#include "CsIO1.h"
#include "freqDma.h"
#include "IO1.h"
#include "TCAP.h"
#include "TP3.h"
// #include "TINT1.h"
#include "DMA0.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include <PE_Types.h>
// Libraries
#include <stdio.h>
#include <string.h>
//==============================================================================
//  LOCAL DEFINES
//==============================================================================
//	DMA samples are move to a circular array with indexes 0,1,.. NBR_DMA_CAPTURE_SAMPLES-1
#define DMA_PREV_POS_INDEX(INDX,PPOS) ((int16)(INDX-PPOS) >= 0 ?  INDX - PPOS : NBR_DMA_CAPTURE_SAMPLES - (PPOS - INDX)  )
#define NO_FREQ_DETECT_OVF_COUNTER_PRE	80 /* Time = 43 * 65536/1,125,000  ~ 2.5 Sec */
#define TCAPT_MSW_MAX_VAL 0x3F		/* Define the maximum higher bits value of capture counter. If 0x3F will give 0 to 0x3FFFFF */
//==============================================================================
//  LOCAL PROTOTYPES.
//==============================================================================


//==============================================================================
//  GLOBAL DATA
//==============================================================================
unsigned int uCapture=0;
LDD_TDeviceData *TP3_ptr = NULL, *TP4_ptr=NULL;
LDD_TDeviceData *TimerPtr = NULL;
unsigned int timerTicks, ulPeriod;
unsigned short uNbrRestarts, uNbrOvfNoPulsePoll;
uint16 tPeriodIsr;
uint16 uMswDelta, uMswIndexSampled;
bool bDisplay = FALSE;
stDmaFrequency atFreqIn[NBR_FREQ_CHANNELS];
uint32 uPeriodLimit;
//==============================================================================
//  LOCAL DATA
//==============================================================================
int uWastePreset = 500;
// This is the Table where the Frequency Signals are captured
static uint16	aDmaCaptureTbl[NBR_FREQ_CHANNELS][NBR_DMA_CAPTURE_SAMPLES];
bool monitorEnable = FALSE;
	
/*!
 * Define a NULL Transfer descriptor to Initialize
 */
const LDD_DMA_TTransferDescriptor DMA_TRANSFER_DESC_NULL = {
  /* UserDataPtr = */                  NULL,
  /* SourceAddress = */                (LDD_DMA_TAddress)0,
  /* SourceAddressOffset = */          (LDD_DMA_TAddressOffset)0,
  /* SourceTransferSize = */           (LDD_DMA_TTransferSize)0,
  /* SourceModuloSize = */             (LDD_DMA_TModuloSize)0,
  /* DestinationAddress = */           (LDD_DMA_TAddress)0,
  /* DestinationAddressOffset = */     (LDD_DMA_TAddressOffset)0,
  /* DestinationTransferSize = */      0,
  /* DestinationModuloSize = */        (LDD_DMA_TModuloSize)0,
  /* TransferMode = */                 LDD_DMA_SINGLE_TRANSFER,
  /* ByteCount = */                    (LDD_DMA_TByteCount)0,
  /* OuterLoopCount = */               (LDD_DMA_TOuterLoopCount)0,
  /* InnerLoopChannelLink = */         FALSE,
  /* InnerLoopLinkedChannel = */       0,
  /* OuterLoopChannelLink = */         FALSE,
  /* OuterLoopLinkedChannel = */       0,
  /* AfterRequestComplete = */         (LDD_DMA_TAfterRequest)LDD_DMA_NO_ACTION,
  /* AddressOffset = */                (LDD_DMA_TAddressOffset)0,
  /* AfterTransferComplete = */        (LDD_DMA_TAfterTransfer)LDD_DMA_NO_ACTION,
  /* SourceAddressAdjustment = */      (LDD_DMA_TAddressOffset)0,
  /* DestinationAddressAdjustment = */ (LDD_DMA_TAddressOffset)0,
  /* ScatterGatherAddress = */         (LDD_DMA_TAddress)0,
  /* BandwidthControl = */             (LDD_DMA_TBandwidthControl)DMA_PDD_NO_STALL,
  /* ChannelAutoSelection = */         TRUE,
  /* ChannelNumber = */                (LDD_DMA_TChannelNumber)0,
  /* TriggerType = */                  LDD_DMA_SW_TRIGGER,
  /* TriggerSource = */                (LDD_DMA_TTriggerSource)0,
  /* PeriodicTrigger = */              FALSE,
  /* DisableAfterRequest = */          FALSE,
  /* Interrupts = */                   FALSE,
  /* OnComplete = */                   FALSE,
  /* OnHalfComplete = */               FALSE,
  /* OnError = */                      FALSE,
  /* OnCompleteEventPtr = */           NULL,
  /* OnErrorEventPtr = */              NULL,
  /* ChannelEnabled = */               FALSE 
};


//==============================================================================
// FUNCTIONS
//==============================================================================

uint32 convTo32(uint16 uHi, uint16 uLo)
{
	// uint32 uData = uHi << 16;
	return (uint32)((uint32)uHi << 16) + uLo;
}


void TransferComplete(LDD_TUserData *UserData)
{
  volatile bool *Completed = (volatile bool*)UserData;
  *Completed = TRUE;
  
  
}

void clearTable()
{
	memset(&aDmaCaptureTbl[0], 0,sizeof(aDmaCaptureTbl[0][0]));	// Empty data buffer
}

void wasteSometime(uint16 ticks)
{
	uint16 i;
	for(i=0; i< ticks; ++i)
		__asm volatile ("nop");
}

/*!
 * Timed routine called at the middle and the overflow of timer capture counter
 * 
 * The DMA capture table is used to provide the Period of the input signal.
 * 
 * The routine keeps the high part of the capture timer providing extra bits defined by ::TCAPT_MSW_MAX_VAL 
 * 
 * The current DMA index is compared with the one in the previous isr and any change represents the pulses that 
 * have been arrived since then, uNbrEdges. Calculations are made only when at least a pulse is detected, but
 * the high portion of the counter is incremented on the ovf (i.e. when the FTM0_CNT has rolled over 0xFFFF) 
 * 
 * When uNbrEdges is GE than 2, the period is within 16 bits and can be calculated with the last two samples from 
 * DMA table. But also, if more than two pulses are detected, we can go back into the DMA table to get the capture of
 * the pulse uNbrEdges before, with the two captures we can create precise period by the quotient between samples
 * divided by the number of edges.
 * 
 * When the uNbrEdges is 1, we keep a track of the last capture (in full extended bit format) to calculate the low 
 * frequency range.
 *   
 */
void timerCaptureIsr(void)
{
	uint16 uNbrEdges;
	
	// Increase chance to get a capture in the isr
	wasteSometime(uWastePreset);	
	
	// Translate DMA major loop iteration counter (CITR) to index in the destination table 
	atFreqIn[0].dmaTblIndex = NBR_DMA_CAPTURE_SAMPLES - DMA_TCD0_CITER_ELINKNO;	// position in table that DMA is going to write in
		
	// Get the Number of Pulse Edges between this and last OverFlow
	uNbrEdges = atFreqIn[0].dmaTblIndex >= atFreqIn[0].dmaTblIndexPrev ?  atFreqIn[0].dmaTblIndex - atFreqIn[0].dmaTblIndexPrev \
			: atFreqIn[0].dmaTblIndex + (NBR_DMA_CAPTURE_SAMPLES - atFreqIn[0].dmaTblIndexPrev);
	atFreqIn[0].dmaTblIndexPrev = atFreqIn[0].dmaTblIndex;	// for next detection
	
	// The uNbrEdges indicates the amount of captures since last time-isr
	if(uNbrEdges)
	{
		uint16 uTblIndex1;	
		uint32 uTmrCapture;		// 	Compute the last captured period using full 22 bits
		uTblIndex1 = DMA_PREV_POS_INDEX(atFreqIn[0].dmaTblIndex, 1);		// 	Index of Ts-1 Last (and safe) capture position
		
		//TP3_SetVal(TimerPtr); 
		TP3_NegVal(TimerPtr);
		
		// Keep building the 32bit capture of the last pulse 
		uTmrCapture =  (uint32)((uint32)atFreqIn[0].uMswCapTmr << 16) +	aDmaCaptureTbl[0][uTblIndex1];	// 	Capture at Ts-1
		
		// Get the previous to last capture: 
		if(uNbrEdges >=2)		// If we had 2 or more captures in previous isr need to get it from DMA
		{
			uint16 uTblIndexN = DMA_PREV_POS_INDEX(atFreqIn[0].dmaTblIndex, uNbrEdges);		//	Index of Ts-2 at DMA
			// Unambiguous last 2 captures from DMA
			if(aDmaCaptureTbl[0][uTblIndex1] >= aDmaCaptureTbl[0][uTblIndexN])
				atFreqIn[0].sPeriod.sFraction.uNum = aDmaCaptureTbl[0][uTblIndex1] - aDmaCaptureTbl[0][uTblIndexN];
			else
				atFreqIn[0].sPeriod.sFraction.uNum = aDmaCaptureTbl[0][uTblIndex1] + (0xFFFF - aDmaCaptureTbl[0][uTblIndexN]) +1;
			atFreqIn[0].sPeriod.sFraction.uDen = uNbrEdges-1;
			atFreqIn[0].fHighFreq = TRUE;
		}
		else
		{
			// Need to get the unambiguous from 32 bit values
			if(uTmrCapture >= atFreqIn[0].uLastCapture)
			{
				atFreqIn[0].sPeriod.ulTot = uTmrCapture - atFreqIn[0].uLastCapture;
				atFreqIn[0].fCaptureInIsr = FALSE;
			}
			else
			{		
				atFreqIn[0].fCaptureInIsr = TRUE;
				// Ambiguity on lower 16bits
				if((uTmrCapture & 0xFFFF0000) == (atFreqIn[0].uLastCapture & 0xFFFF0000))			// 1) MSW are the same
					atFreqIn[0].sPeriod.ulTot = (uTmrCapture + 0x10000) - atFreqIn[0].uLastCapture;	// Carry over from lower 16 bits
				else	
					// Ambiguity on full 22 bits
					atFreqIn[0].sPeriod.ulTot = (uTmrCapture + (uint32)((uint32)(TCAPT_MSW_MAX_VAL + 1) << 16) ) - atFreqIn[0].uLastCapture;	// Rollover 0x3FFFFF
				
			}
			atFreqIn[0].fHighFreq = FALSE;
		}
		// ========  Just a Debug artifact ========
		if(monitorEnable)
		{
			if(atFreqIn[0].fDisplay) // && iCiter == DMA_TCD0_CITER_ELINKNO)	// Don't display if a new capture
			{
				float f1;
				if(atFreqIn[0].fHighFreq)
					f1 = (float)1125000.0*atFreqIn[0].sPeriod.sFraction.uDen/(float)atFreqIn[0].sPeriod.sFraction.uNum;
				else
					f1 = (float)1125000.0/atFreqIn[0].sPeriod.ulTot;
				//
				printf("\r\n%f ",f1 );
				if(atFreqIn[0].fCaptureInIsr)
					printf("= Amb solved=");   
				atFreqIn[0].fDisplay = FALSE;
			}
		}
		else
		if(atFreqIn[0].fDisplay || atFreqIn[0].fCaptureInIsr) // && iCiter == DMA_TCD0_CITER_ELINKNO)	// Don't display if a new capture
		{
			if(atFreqIn[0].fHighFreq)
				printf("\r\n%u / %u ", atFreqIn[0].sPeriod.sFraction.uNum, atFreqIn[0].sPeriod.sFraction.uDen);
			else
				printf("\r\n%6lu E%d ", atFreqIn[0].sPeriod.ulTot, uNbrEdges);
			if(atFreqIn[0].fCaptureInIsr)
				printf("= Capture inside=");   
			atFreqIn[0].fDisplay = FALSE;
		}
		
		atFreqIn[0].uLastCapture = uTmrCapture;
		atFreqIn[0].fNewFreq = TRUE;
		// 
		//TP3_ClrVal(TimerPtr);
	}
	// 	else		bNoPulseIsr = TRUE;	// We have a Timed isr with no pulse in previous - resync the next isr for low frequency if needed
	
	// Prepare next Interrupt - we avoid getting captures close to 0xFFFF-0x0000 when in the interrupt
		if(FREQ_TCAP_REG_CNT > 0x7FFF )
			FREQ_TCAP_REG_CMP = 0xFFFF;		// 	Next isr when compare this value
		else
		{
			// 	We just Crossed 0xFFFF - Keep the higher portion of capture counter 
			if(++atFreqIn[0].uMswCapTmr > TCAPT_MSW_MAX_VAL)	// We just need 6 extra bits
				atFreqIn[0].uMswCapTmr =0;		// Roll-over 0x3FFFFF to 0
			FREQ_TCAP_REG_CMP = 0x7FFF;			//	Next isr will happen at mid range of counter
		}
}



void freqDmaRun(void)
{

	LDD_TDeviceData *DMAPtr = NULL;
	LDD_DMA_TTransferDescriptor TransferDesc;
	
	volatile bool Completed = FALSE;
	unsigned int uTimerCaptureClock = CAPTURE_CLOCK_HZ;
  
	
	
	// My local init
	clearTable();
	
	uPeriodLimit = 40000L;
	
	// Init the DMA		
	TransferDesc = DMA_TRANSFER_DESC_NULL;
  TP3_ptr = TP3_Init(NULL);
  TP4_ptr = TP4_Init(NULL);
  // TINT1Ptr = TINT1_Init(NULL);
  DMAPtr = DMA0_Init(NULL);
  /* Initialize transfer descriptor */
  	// Source
  TransferDesc.SourceAddress = (LDD_DMA_TAddress)&FTM0_C4V;											//	(FTM1_C0SC, FTM1_C0V)  FTM1_CH0 capture
  TransferDesc.SourceAddressOffset = (LDD_DMA_TAddressOffset)0;										//	Single source		
  TransferDesc.SourceTransferSize = (LDD_DMA_TTransferSize)DMA_PDD_16_BIT;				//	16-bit value
  TransferDesc.SourceModuloSize = (LDD_DMA_TModuloSize)0;													// 	non relevant (single source)
  	// Destination
  TransferDesc.DestinationAddress = (LDD_DMA_TAddress)&aDmaCaptureTbl[0];								//	Move to Capture table	
  TransferDesc.DestinationAddressOffset = (LDD_DMA_TAddressOffset)sizeof(aDmaCaptureTbl[0][0]);	// Next write offset
  TransferDesc.DestinationTransferSize = (LDD_DMA_TTransferSize)DMA_PDD_16_BIT;		// moving 16-bit data
  TransferDesc.DestinationModuloSize = (LDD_DMA_TModuloSize)0;										// we will set manually a new address
    
    // We have Major loop = Move 4 bytes per interrupt and repeat 8 times 
  TransferDesc.TransferMode = LDD_DMA_NESTED_TRANSFERS;						
  TransferDesc.ByteCount = (LDD_DMA_TByteCount)2;	
  TransferDesc.OuterLoopCount = (LDD_DMA_TOuterLoopCount)NBR_DMA_CAPTURE_SAMPLES;
    
    //	DMA channel
  TransferDesc.ChannelAutoSelection = FALSE;// TRUE;								// Fixed to ch 0	
  TransferDesc.ChannelNumber = (LDD_DMA_TChannelNumber)0;		// Channel 0 in this example
    
    //	Trigger
  TransferDesc.TriggerType = LDD_DMA_HW_TRIGGER;						// Triggered by Peripheral request
  																										//	
  TransferDesc.TriggerSource = (LDD_DMA_TTriggerSource)24;	// DMA source Table 3-24 == FTM0_CH4 
  TransferDesc.Interrupts = TRUE;	// we are here									We are not going to interrupt on DMA transfer complete 
  TransferDesc.OnComplete = TRUE;										// Signal an DMA-done	
  TransferDesc.OnCompleteEventPtr = &TransferComplete;			// call this function
  TransferDesc.UserDataPtr = (LDD_TUserData*)&Completed;		// the UserDataPtr to pass data	
    
    // AFter transfer
  TransferDesc.AfterTransferComplete = LDD_DMA_ADDRESS_ADJUSTMENT;
  TransferDesc.DestinationAddressAdjustment = -(2*NBR_DMA_CAPTURE_SAMPLES);	
  
  /* Start DMA transfer */
  DMA0_AllocateChannel(DMAPtr, &TransferDesc);					// Still need to call this method even  we have a fixed channel
  DMA0_EnableChannel(DMAPtr, &TransferDesc);						// This moves the TransferDesc to the TCD 32 bytes
  // DMA0_StartChannelTransfer(DMAPtr, &TransferDesc);
    
  
      
  //============ Using PEX DMA, TCAP Components =========================
  // TCAP:TimerUnit_LDD  !!! DO not enable in init code and do not Autoninit in PEX properties !!  
  TimerPtr = TCAP_Init(NULL);		// 	Set the desired PEX Properties and get the pointer to static memory
  //	Individually disable Channels 
  FTM0_C0SC &= ~FTM_CnSC_CHIE_MASK;		// Disable CH0 interrupt and repeat for other channels
  FTM0_C4SC &= ~FTM_CnSC_CHIE_MASK;		// Disable CH1 interrupt
  FTM0_C0V = 0x7FFF;									//	Need to have DMA ready before OVrs
    
  // 	Enable the base Timer Module by selecting the clk source in CLKS[FTM1_SC]
  TCAP_Enable(TimerPtr);
  // DONE! to start capturing, enable the Channel capture IE in the App
  
  
  // remove Debug UART buffering (stream i/o)
  setvbuf(stdout, NULL, _IONBF, 0); // no buffering on stdout - for printf()
  setvbuf(stdin, NULL, _IONBF, 0); // no buffering on stdin - for getchar() etc
  
  printf("Porting DMA: Capture FTM1_CH0 at PTA12 - press '1','t' to start, 'm' to monitor\n\r");
  printf("\n\r");
  unsigned int uLocalCapture=0;
  
  // Init Periodic Time Interrupt
  // TINT1_Enable(TINT1Ptr);	// Enable interrupts after we get the pointer to PTA17
  int i, ch =0;
  uCapture = uLocalCapture =0;
  float fFreq;
  unsigned int uCtr=0;
    	
  while(1)
  {
  	if(uLocalCapture != uCapture)
  	{
  		if(uCapture > 99)
  			uCapture =0;
  		uLocalCapture = uCapture; 
  		  		
  		// PTA17_NegVal(PTA17_ptr);
  		printf("(%2u,%u) ", uCapture, tPeriodIsr);	// Isr executed 
  	}
  	if( (ch = getchar()) != EOF )
  	switch(ch)
  	{
  	// Diable all interrupts
  	case '0':	
  		FTM0_C0SC &= 	~FTM_CnSC_CHIE_MASK;
  		FTM0_C4SC &=  ~(FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);		
  		break;
  	// Enable DMA 
  	case '1' :
  		FTM0_C4SC |= (FTM_CnSC_CHIE_MASK | FTM_CnSC_DMA_MASK);		// Enable TCAP_CH4 DMA request
  		break;
  		
  	// Enable Isr in Capture 
  	case '2' :
  		FTM0_C4SC |= FTM_CnSC_CHIE_MASK;													// Test TCAP_CH4 ISR is working
  		break;
  		
  	// Enable (OvfIsr)
  	case 't' : case 'T':
  	  FTM0_C0SC |= FTM_CnSC_CHIE_MASK;													// Test TCAP_CH0 (OVF) ISR is working		
  	  break;
  	
  	case '4':
 			monitorEnable = monitorEnable ? 0 :1;
 			break;
 			
 		case 'm':
 			//  			bDisplay = TRUE;
 			monitorEnable = monitorEnable? FALSE : TRUE;
 			break;
 		case '?': 
#if 0
 			printf("\r\nMon=%d, Clk= %u, type '+'/'-' to inc/dec", monitorEnable, uTimerCaptureClock);
 			if(tFreqInput.uNbrEdges >1 )
 				fFreq = (float)tFreqInput.uNbrEdges * uTimerCaptureClock/tFreqInput.utCalcPeriod.l;	//(saves one multiplication)
 			else
 				fFreq = (float)uTimerCaptureClock/tFreqInput.utCalcPeriod.l;
 			//	Output the value
 			printf("%5u %f\r\n", ++uCtr, fFreq);
#endif  		
 			break;
 			
 		case ' ':
 			bDisplay = TRUE;
 			atFreqIn[0].fDisplay = TRUE;
 			break;
 		case '+':
 			++uTimerCaptureClock;
 			break;
 		case '-':
 			--uTimerCaptureClock;
 			break;
 			
 		case 'c': case 'C':
 			clearTable();
 			break;
 			
  	}
  	if(monitorEnable && atFreqIn[0].fNewFreq)
  	{
  		//TP3_SetVal(TimerPtr);
  		atFreqIn[0].fNewFreq = FALSE;
#if 0
  		if(tFreqInput.uNbrEdges >1 )
  			fFreq = (float)tFreqInput.uNbrEdges * uTimerCaptureClock/tFreqInput.utCalcPeriod.l;	//(saves one multiplication)
  		else
  			fFreq = (float)uTimerCaptureClock/tFreqInput.utCalcPeriod.l;
  		//	Output the value
  		printf("%5u %f\r\n", ++uCtr, fFreq);
#endif
  		//TP3_ClrVal(TimerPtr); 
  	}
  		
  }
  //  DMA_Deinit(DMAPtr);
  // TIMER_Deinit(TimerPtr);
	
}


