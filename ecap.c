/*
 * ecap.c
 *
 *  Created on: 22 gen 2019
 *      Author: andrea
 */

#include <ecap.h>
#include <pru_ecap.h>

uint8_t ecap_Init() {
    // Disabilito ed azzero interrupts
    CT_ECAP.ECEINT = 0x00;
    CT_ECAP.ECCTL2 &= EC_STOP_MSK; // Stop ecap
    CT_ECAP.ECCLR &= ECCLR_MSK;

    // TODO: sono tutte disabilitate ... serve abilitare?
    // Abilito interrupts
    CT_ECAP.ECEINT = ECEINT_CFG;

    // Configure & start ecap
    CT_ECAP.ECCTL1 = ECCTL1_CFG; // all rising edge, reset counter at any capture
    CT_ECAP.ECCTL2 = ECCTL2_CFG & EC_STOP_MSK; // continuous, capture mode, wrap after capture 4, rearm, free running,synci/o disabled
    return 1;
}
uint8_t ecap_Start() {
    CT_ECAP.TSCTR = 0x00000000;
    CT_ECAP.ECCTL2 = ECCTL2_CFG; // start ecap
    return 1;

}
uint8_t ecap_Stop(){
    CT_ECAP.ECCTL2 = ECCTL2_CFG & EC_STOP_MSK; // stop ecap
    return 1;

}


