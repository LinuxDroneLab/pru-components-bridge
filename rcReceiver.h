/*
 * edma.h
 *
 *  Created on: 23 gen 2019
 *      Author: andrea
 */
#include <stdint.h>

#ifndef _RC_RECEIVER_H_
#define _RC_RECEIVER_H_

/********************
 * ECAP DEFINITIONS
 ********************/
#define ECCTL1_CFG       0xC1EE /* DIV1,ENABLED,DELTA_MODE, RISING/FALLING */
#define ECCTL2_CFG       0x00DE /* RE-ARM, ECAP_MODE, RUN, SYNCO/I DISABLED, CONTINUOUS */
#define ECEINT_CFG       0x0002 /* EVT1 interrupt enabled */
#define ECCLR_MSK        0x00FF /* clear all */
#define EC_STOP_MSK      0xFFEF /* mask stop ecap */

uint8_t ecap_Init();
uint8_t ecap_Start();
uint8_t ecap_Stop();

/*******************
 * EDMA DEFINITIONS
 *******************/
#define NUM_EDMA_FRAME_BLOCK 64
#define EDMA0_CC_BASE   ((volatile uint32_t *)(0x49000000))
#define CM_PER_BASE     ((volatile uint32_t *)(0x44E00000))
#define EDMA3CC_ECAP0_EVT 1 // event EDMA for ecap0

/*
 * FIXME: fix pru_edma.h some field does not works.
 * TCC_DRAE in pru_edma.h does not works (why?).
 * with this definition works fine.
 */
#define DRAE1           (0x0348 / 4)

/* EDMA Shadow Region 1 */
#define ESR             (0x2210 / 4)
#define ESRH            (0x2214 / 4)
#define EESR            (0x1030 / 4) // global EESR
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


/**************************
 * RC RECEIVER DEFINITIONS
 **************************/
#define RC_RECEIVER_TX_NOT_PRESENT 0x2
#define RC_RECEIVER_TX_COMPLETE    0x1

uint8_t rc_receiver_Init();
uint8_t rc_receiver_Start();
uint8_t rc_receiver_Stop();
uint8_t rc_receiver_PulseNewData();

#endif /* _RC_RECEIVER_H_ */