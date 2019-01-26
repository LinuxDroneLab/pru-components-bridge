/*
 * ecap.h
 *
 *  Created on: 22 gen 2019
 *      Author: andrea
 */
#include <stdint.h>

#ifndef ECAP_H_
#define ECAP_H_

#define ECCTL1_CFG       0xC1EE /* DIV1,ENABLED,DELTA_MODE, RISING/FALLING */
#define ECCTL2_CFG       0x00DE /* RE-ARM, ECAP_MODE, RUN, SYNCO/I DISABLED, CONTINUOUS */
#define ECEINT_CFG       0x0000 /* none */
#define ECFLG_MSK        0x00FF /* none */
#define ECCLR_MSK        0x00FF /* clear all */
#define EC_STOP_MSK      0xFFEF /* stop ecap */

uint8_t ecap_Init();
uint8_t ecap_Start();
uint8_t ecap_Stop();

struct EcapData
{
    uint32_t cap1[8];
    uint32_t cap2[8];
    uint32_t cap3[8];
    uint32_t cap4[8];
};


#endif /* ECAP_H_ */
