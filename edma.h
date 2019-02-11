/*
 * edma.h
 *
 *  Created on: 23 gen 2019
 *      Author: andrea
 */
#include <stdint.h>

#ifndef EDMA_H_
#define EDMA_H_

#define NUM_EDMA_FRAME_BLOCK 64

#define EDMA0_CC_BASE   ((volatile uint32_t *)(0x49000000))
#define CM_PER_BASE     ((volatile uint32_t *)(0x44E00000))
#define TPTC0_CLKCTRL (0x24 / 4)
#define TPCC_CLKCTRL  (0xBC / 4)
#define ON (0x2)

#define EDMA3CC_ECAP0_EVT 1 //evento EDMA per ecap0

#define DRAE1           (0x0348 / 4)

/* EDMA Shadow Region 1 */
#define ESR             (0x2210 / 4)
#define ESRH            (0x2214 / 4)
#define EESR            (0x1030 / 4) //
#define EECR            (0x2228 / 4)
#define EECRH           (0x222C / 4)
#define SECR            (0x2240 / 4)
#define SECRH           (0x2244 / 4)
#define IPR             (0x2268 / 4)
#define IPRH            (0x226C / 4)
#define ICR             (0x2270 / 4)
#define ICRH            (0x2274 / 4)
#define IESR            (0x2260 / 4)
#define IER             (0x2250 / 4)
#define IESRH           (0x2264 / 4)
#define IEVAL           (0x2278 / 4)
#define IECR            (0x2258 / 4)
#define IECRH           (0x225C / 4)

uint8_t edma_Init();
uint8_t edma_init_PaRAM();
uint32_t* edma_get_Data();
void edma_reset_Data();

#endif /* EDMA_H_ */
