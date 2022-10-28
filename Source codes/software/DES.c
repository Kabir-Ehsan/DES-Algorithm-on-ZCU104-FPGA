
/*
 * DES.c: DES application

 */

#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xaxidma.h" // Device driver API for AXI DMA
#include "xaxidma_hw.h"
#include "plaintext.h"
#include "xparameters.h"
#include "xil_cache.h"
#include "fiboSW.h"
#include "ctype.h"
#include "math.h"
#include "time.h"


XAxiDma axiDma;

int init_dma(XAxiDma *axiDmaPtr);


int main()
{
    print("DES ALGORITHM ON ZCU104\n\r");
    print("\n Software Results: \n\n\r");


	fiboSW(); // Calling Software Function

    init_platform();

	int status, i, j;

	static uint64_t din[1024];

    static uint64_t dout[1024];


	for(i=0;i<1024;i=i+1)
	{
		din[i] = p[i];
		//printf("HW_in[%d]= %lu  \n\r", i, din[i]);
	}

	status = init_dma(&axiDma);

	if (status != XST_SUCCESS)
	{
		exit(-1);
	}
	print("\rDMA Initiation done\n\r");


	 Xil_DCacheFlushRange((UINTPTR)din, (1024)*sizeof(uint64_t));
	 Xil_DCacheFlushRange((UINTPTR)dout, (1024)*sizeof(uint64_t));

	 print("\rCache cleared successfully\n\r");

	status = XAxiDma_SimpleTransfer(&axiDma, (UINTPTR)din, (1024)*sizeof(uint64_t), XAXIDMA_DMA_TO_DEVICE);
	status = XAxiDma_SimpleTransfer(&axiDma, (UINTPTR)dout, (1026)*sizeof(uint64_t), XAXIDMA_DEVICE_TO_DMA);

	while(XAxiDma_Busy(&axiDma, XAXIDMA_DEVICE_TO_DMA)||XAxiDma_Busy(&axiDma, XAXIDMA_DMA_TO_DEVICE));

	Xil_DCacheInvalidateRange((UINTPTR)dout, (1024)*sizeof(uint64_t));

	print("\rDMA Transfer Done\n\r");


	print("\nHardware Results: \n\n\r");

	for(j=2;j<9;j++)
		printf("HW_out[%d]= %llX  \n\n\r", j, dout[j]);

	for(j=299;j<303;j++)
		printf("HW_out[%d]= %llX  \n\n\r", j, dout[j]);

	for(j=1014;j<1017;j=j+1)
	{
		printf("HW_out[%d]= %llX  \n\n\r", j, dout[j]);
	}

    cleanup_platform();


    return 0;
}


int init_dma(XAxiDma *axiDmaPtr)
{
	//  AXI DMA config

	XAxiDma_Config *CfgPtr;
	int status;

	// Get pointer to DMA configuration
	CfgPtr = XAxiDma_LookupConfig(XPAR_AXIDMA_0_DEVICE_ID);

   if(!CfgPtr)
   {
      printf("Error looking for AXI DMA config\n\r");
      return XST_FAILURE;
   }
   // Initialize the DMA handle
   status = XAxiDma_CfgInitialize(axiDmaPtr,CfgPtr);
   if(status != XST_SUCCESS)
   {
      printf("Error initializing DMA\n\r");
      return XST_FAILURE;
   }

   //check for scatter gather mode - this example must have simple mode only
   if(XAxiDma_HasSg(axiDmaPtr))
   {
       printf("Error DMA configured in SG mode\n\r");
       return XST_FAILURE;
   }

   //disable the interrupts
  XAxiDma_IntrDisable(axiDmaPtr, XAXIDMA_IRQ_ALL_MASK,XAXIDMA_DEVICE_TO_DMA);
  XAxiDma_IntrDisable(axiDmaPtr, XAXIDMA_IRQ_ALL_MASK,XAXIDMA_DMA_TO_DEVICE);


   return XST_SUCCESS;
};


