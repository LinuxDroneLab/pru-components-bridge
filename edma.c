/*
 * edma.c
 *
 *  Created on: 23 gen 2019
 *      Author: andrea
 */
#include <edma.h>
#include <pru_edma.h>
uint32_t* CM_PER_TPCC_CLKCTRL  = (uint32_t*) 0x44E000BC;
uint32_t* CM_PER_TPTC0_CLKCTRL = (uint32_t*) 0x44E00024;

char* DATA_MEMORY_BASE_ADDRESS = (char*)0x4A300000;

// buffer src dei segnali da inviare
#define NUM_EDMA_FRAME_BLOCK 64
uint32_t FRAME_TO_TRANSFER[NUM_EDMA_FRAME_BLOCK] = { 0 };


#define EDMA3CC_ECAP0_EVT 38 //evento EDMA per ecap0

uint32_t* edma_get_Data() {
    return FRAME_TO_TRANSFER;
}

uint8_t edma_Init() {

    // prepare PaRAM0 for Ecap
    (EDMA_PaRAM)->optBits.tcc = EDMA3CC_ECAP0_EVT; // Transfer Complete interrupt enabled;
    (EDMA_PaRAM)->src = 0x48300108; // CAP1
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
    (EDMA_PaRAM + 1)->src = 0x48300108; // CAP1
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

    // Set Channel for Ecap
    CT_TCC.TCC_DCHMAP_bit[EDMA3CC_ECAP0_EVT].TCC_DCHMAP_PAENTRY = 0;
    CT_TCC.TCC_DMAQNUM_bit[4].TCC_DMAQNUM_E6 = 0; // coda 0 per channel ECAP0EVT

    CT_TCC.TCC_ECRH |= (1 << (EDMA3CC_ECAP0_EVT - 31)); // clear evento ecap0
    CT_TCC.TCC_IECRH |= (1 << (EDMA3CC_ECAP0_EVT - 31)); // clear interrupt ecap0
    CT_TCC.TCC_SECRH |= (1 << (EDMA3CC_ECAP0_EVT - 31)); // clear secondary event ecap0
    CT_TCC.TCC_ICRH |= (1 << (EDMA3CC_ECAP0_EVT - 31)); // clear interrupt ecap0
    CT_TCC.TCC_EMCRH |= (1 << (EDMA3CC_ECAP0_EVT - 31)); // clear event missing ecap0

    CT_TCC.TCC_EESRH = (1 << (EDMA3CC_ECAP0_EVT - 31)); // abilito evento ecap0
    CT_TCC.TCC_IESRH = (1 << (EDMA3CC_ECAP0_EVT - 31)); // abilito interrupt ecap0

    // TODO: vedere per le regions

    return 1;
}

