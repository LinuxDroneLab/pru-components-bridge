/*
 * edma.c
 *
 *  Created on: 23 gen 2019
 *      Author: andrea
 */
#include <edma.h>
#include <pru_edma.h>

EDMA_PaRAM_STRUCT* EDMA_PaRAM = (EDMA_PaRAM_STRUCT*)EDMA_0_PARAM;

uint32_t* CM_PER_TPCC_CLKCTRL  = (uint32_t*) 0x44E000BC;
uint32_t* CM_PER_TPTC0_CLKCTRL = (uint32_t*) 0x44E00024;

char* DATA_MEMORY_BASE_ADDRESS = (char*)0x4A302000;

// buffer src dei segnali da inviare
#define NUM_EDMA_FRAME_BLOCK 64
uint32_t FRAME_TO_TRANSFER[NUM_EDMA_FRAME_BLOCK] = { 0 };

uint32_t* edma_get_Data() {
    return FRAME_TO_TRANSFER;
}

void edma_reset_Data() {
    uint8_t i = 0;
    for(i = 0; i < NUM_EDMA_FRAME_BLOCK; i++) {
        FRAME_TO_TRANSFER[i] = 0;
    }
}
uint8_t edma_init_PaRAM() {
    // prepare PaRAM0 for Ecap
    (EDMA_PaRAM)->optBits.tcc = EDMA3CC_ECAP0_EVT; // event 1 as am335x datasheet table pag. 1540
    (EDMA_PaRAM)->optBits.tcinten = 1; // completion interrupt enabled
    (EDMA_PaRAM)->src = 0x4a330008; // CAP1
    (EDMA_PaRAM)->acnt = 16; // cap1-4, 4 bytes for 4 registers
    (EDMA_PaRAM)->bcnt = 16; // Ogni 4 B-cicli catturo 'rcRises -1'.
    (EDMA_PaRAM)->dst = (uint32_t)(DATA_MEMORY_BASE_ADDRESS + (uint32_t)FRAME_TO_TRANSFER);
    (EDMA_PaRAM)->srcbidx = 0;
    (EDMA_PaRAM)->dstbidx = 16;
    (EDMA_PaRAM)->link = 0x4020;
    (EDMA_PaRAM)->bcntrld = 0;
    (EDMA_PaRAM)->srccidx = 0;
    (EDMA_PaRAM)->dstcidx = 0;
    (EDMA_PaRAM)->ccnt = 1;

    // PaRAM1 Link
    (EDMA_PaRAM + 1)->optBits.tcc = EDMA3CC_ECAP0_EVT; // Transfer Complete interrupt enabled;
    (EDMA_PaRAM)->optBits.tcinten = 1; // completion interrupt enabled
    (EDMA_PaRAM + 1)->src = 0x4a330008; // CAP1
    (EDMA_PaRAM + 1)->acnt = 16; // cap1-4, 4 bytes for 4 registers
    (EDMA_PaRAM + 1)->bcnt = 16; // Ogni 4 B-cicli catturo 'rcRises -1'.
    (EDMA_PaRAM + 1)->dst = (uint32_t)(DATA_MEMORY_BASE_ADDRESS + (uint32_t)FRAME_TO_TRANSFER);
    (EDMA_PaRAM + 1)->srcbidx = 0;
    (EDMA_PaRAM + 1)->dstbidx = 16;
    (EDMA_PaRAM + 1)->link = 0x4020;
    (EDMA_PaRAM + 1)->bcntrld = 0;
    (EDMA_PaRAM + 1)->srccidx = 0;
    (EDMA_PaRAM + 1)->dstcidx = 0;
    (EDMA_PaRAM + 1)->ccnt = 1;

    return 1;
}
uint8_t edma_Init() {
    volatile uint32_t *ptr;
    uint32_t channelMask = (1 << EDMA3CC_ECAP0_EVT);
    ptr = EDMA0_CC_BASE;

    if(!edma_init_PaRAM()) {
        return 0;
    }

/* Shadow Region Registers
 ER,ERH,QER,ECR,ECRH,QEER,ESR,ESRH,QEECR,CER,CERH,QEESR,
 EER,EERH,EECR,EECRH,EESR,EESRH
 SER,SERH,SECR,SECRH
 IER,IERH,IECR,IECRH,IESR,IESRH
 IPR,IPRH,ICR,ICRH
*/

    // Set Channel for Ecap
    CT_TCC.TCC_DCHMAP_bit[EDMA3CC_ECAP0_EVT].TCC_DCHMAP_PAENTRY = 0;
    CT_TCC.TCC_DMAQNUM_bit[0].TCC_DMAQNUM_E1 = 0; // coda 0 per channel ECAP0EVT
    ptr[DRAE1] |= channelMask; // region 1 for ECAP0EVT channel

    // tratto da /hd/linuxlab/beagleboard/ti/pru-software-support-package/examples/am335x/PRU_edmaConfig'

    // clears
    /* Clear channel event from EDMA event registers */
    ptr[SECR] |= channelMask;
    ptr[ICR] |= channelMask;

    /* Enable channel interrupt */
    ptr[IESR] |= channelMask;

    /* Enable channel */
    ptr[EESR] |= channelMask;

    /* Clear event missed register */
    CT_TCC.TCC_EMCR |= channelMask;

    return 1;
}

