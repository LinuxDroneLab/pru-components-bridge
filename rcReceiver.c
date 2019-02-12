/*
 * edma.c
 *
 *  Created on: 23 gen 2019
 *      Author: andrea
 */
#include <stdint.h>
#include <pru_intc.h>
#include <pru_ecap.h>
#include <pru_edma.h>
#include <rcReceiver.h>

EDMA_PaRAM_STRUCT* EDMA_PaRAM = (EDMA_PaRAM_STRUCT*)EDMA_0_PARAM;

uint32_t* CM_PER_TPCC_CLKCTRL  = (uint32_t*) 0x44E000BC;
uint32_t* CM_PER_TPTC0_CLKCTRL = (uint32_t*) 0x44E00024;

#ifdef pru1
char* DATA_MEMORY_BASE_ADDRESS = (char*)0x4A302000;
#else
#ifdef pru0
char* DATA_MEMORY_BASE_ADDRESS = (char*)0x4A300000;
#endif
#endif

// ping/pong buffers for Capture PPM signals
uint32_t FRAME_TO_TRANSFER[2][NUM_EDMA_FRAME_BLOCK] = { 0 };
uint8_t rc_receiver_ReadBufferIdx = 0;
uint8_t rc_receiver_WriteBufferIdx = 1;
uint8_t rc_receiver_TmpBufferIdx = 0;

// variables
volatile uint32_t *_edma_registers_ptr = EDMA0_CC_BASE;
uint32_t _edma_channel_mask = (1 << EDMA3CC_ECAP0_EVT);
uint8_t _rc_receiver_counter8 = 0;
uint8_t _rc_receiver_found = 0;
uint8_t _rc_receiver_curr_channel = 0;
uint32_t* _rc_receiver_ecap_data;
uint8_t result = 0;

uint8_t rc_receiver_ecap_Init() {
    // Disabilito ed azzero interrupts
    CT_ECAP.ECEINT = 0x00;
    CT_ECAP.ECCTL2 &= EC_STOP_MSK; // Stop ecap
    CT_ECAP.ECCLR = ECCLR_MSK; // clear interrupts

    // Enable interrupt at EVT1
    CT_ECAP.ECEINT = ECEINT_CFG;

    // Configure & start ecap
    CT_ECAP.ECCTL1 = ECCTL1_CFG; // all rising edge, reset counter at any capture
    CT_ECAP.ECCTL2 = ECCTL2_CFG & EC_STOP_MSK; // continuous, capture mode, wrap after capture 4, rearm, free running,synci/o disabled
    return 1;
}
uint8_t rc_receiver_ecap_Start() {
    CT_ECAP.TSCTR = 0x00000000;
    CT_ECAP.ECCTL2 = ECCTL2_CFG; // start ecap
    return 1;

}
uint8_t rc_receiver_ecap_Stop(){
    CT_ECAP.ECCTL2 = ECCTL2_CFG & EC_STOP_MSK; // stop ecap
    return 1;

}

uint32_t* rc_receiver_edma_get_Data() {
    return &FRAME_TO_TRANSFER[rc_receiver_ReadBufferIdx][0];
}

void rc_receiver_switch_edma_Buffer() {
    rc_receiver_TmpBufferIdx = rc_receiver_ReadBufferIdx;
    rc_receiver_ReadBufferIdx = rc_receiver_WriteBufferIdx;
    rc_receiver_WriteBufferIdx = rc_receiver_TmpBufferIdx;
}

uint8_t rc_receiver_edma_init_PaRAM() {
    // prepare PaRAM0 for Ecap
    (EDMA_PaRAM)->optBits.tcc = EDMA3CC_ECAP0_EVT; // event 1 as am335x datasheet table pag. 1540
    (EDMA_PaRAM)->optBits.tcinten = 1; // completion interrupt enabled
    (EDMA_PaRAM)->src = 0x4a330008; // CAP1
    (EDMA_PaRAM)->acnt = 16; // cap1-4, 4 bytes for 4 registers
    (EDMA_PaRAM)->bcnt = 16; // Ogni 4 B-cicli catturo 'rcRises -1'.
    (EDMA_PaRAM)->dst = (uint32_t)(DATA_MEMORY_BASE_ADDRESS + (uint32_t)(&FRAME_TO_TRANSFER[rc_receiver_WriteBufferIdx][0]));
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
    (EDMA_PaRAM + 1)->dst = (uint32_t)(DATA_MEMORY_BASE_ADDRESS + (uint32_t)(&FRAME_TO_TRANSFER[rc_receiver_WriteBufferIdx][0]));
    (EDMA_PaRAM + 1)->srcbidx = 0;
    (EDMA_PaRAM + 1)->dstbidx = 16;
    (EDMA_PaRAM + 1)->link = 0x4020;
    (EDMA_PaRAM + 1)->bcntrld = 0;
    (EDMA_PaRAM + 1)->srccidx = 0;
    (EDMA_PaRAM + 1)->dstcidx = 0;
    (EDMA_PaRAM + 1)->ccnt = 1;

    return 1;
}
uint8_t rc_receiver_edma_Init() {
    if(!rc_receiver_edma_init_PaRAM()) {
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
    _edma_registers_ptr[DRAE1] |= _edma_channel_mask; // region 1 for ECAP0EVT channel
//    CT_TCC.TCC_DRAE1 |= channelMask;
    // tratto da /hd/linuxlab/beagleboard/ti/pru-software-support-package/examples/am335x/PRU_edmaConfig'

    // clears
    /* Clear channel event from EDMA event registers */
    _edma_registers_ptr[SECR] |= _edma_channel_mask;
    _edma_registers_ptr[ICR] |= _edma_channel_mask;

    /* Enable channel interrupt */
    _edma_registers_ptr[IESR] |= _edma_channel_mask;

    /* Enable channel */
    _edma_registers_ptr[EESR] |= _edma_channel_mask;

    /* Clear event missed register */
    CT_TCC.TCC_EMCR |= _edma_channel_mask;

    return 1;
}
uint8_t rc_receiver_intc_Init() {

    CT_INTC.CMR3_bit.CH_MAP_15 = INT_ECAP_CHAN;
    CT_INTC.HMR2_bit.HINT_MAP_8 = INT_ECAP_HOST;
    CT_INTC.HIER_bit.EN_HINT |= INT_ECAP_HOST; // enable host interrupt 8
    CT_INTC.EISR_bit.EN_SET_IDX = INT_ECAP; // enable ecap interrupt
    CT_INTC.GER_bit.EN_HINT_ANY = 1; // enable all host interrupt
    return 1;
}
uint8_t rc_receiver_Init() {
    return rc_receiver_intc_Init() && rc_receiver_ecap_Init() & rc_receiver_edma_Init();
}
void rc_receiver_clean_Interrupts() {
    CT_ECAP.ECCLR |= ECCLR_MSK; // remove EVT1-EVT4 interrupts and INT
    _edma_registers_ptr[ICR] = _edma_channel_mask; // reset completion interrupt

}
uint8_t rc_receiver_Start() {
    result = rc_receiver_ecap_Stop();
    rc_receiver_switch_edma_Buffer();
    result &= rc_receiver_edma_init_PaRAM();
    rc_receiver_clean_Interrupts();
    return result & rc_receiver_ecap_Start();
}
uint8_t rc_receiver_Stop() {
    result = rc_receiver_ecap_Stop();
    result &= rc_receiver_edma_init_PaRAM();
    rc_receiver_clean_Interrupts();
    return result;
}

/* Check for New Data from RC
 * TX not connected <=> rc_receiver_newData & RC_RECEIVER_TX_NOT_PRESENT
 * New Data <=> rc_receiver_newData & RC_RECEIVER_TX_COMPLETE
 * it is not idempotent
 */
uint8_t rc_receiver_PulseNewData() {
    uint8_t result = 0;
    if (CT_ECAP.ECFLG & 0x0002)
    {
        CT_ECAP.ECCLR |= ECCLR_MSK; // remove EVT1-EVT4 interrupts and INT
    }
    else
    if (CT_ECAP.ECFLG & 0x0020) // counter overflow
    {
        CT_ECAP.ECCLR |= ECCLR_MSK; // remove EVT1-EVT4 interrupts and INT
        result = 2;
    }

    result |= ((_edma_registers_ptr[IPR] & _edma_channel_mask) >> 1);
    if(result) {
        rc_receiver_ecap_Stop();
        rc_receiver_clean_Interrupts();
        rc_receiver_switch_edma_Buffer();
        rc_receiver_edma_init_PaRAM();
        rc_receiver_ecap_Start();
    }
    return result;
}

void rc_receiver_extract_Data(uint32_t* rc_buffer)
{
    _rc_receiver_ecap_data = rc_receiver_edma_get_Data();

    for (_rc_receiver_counter8 = 0; _rc_receiver_counter8 < NUM_EDMA_FRAME_BLOCK; _rc_receiver_counter8++)
    {
        if ((_rc_receiver_found == 0) && (_rc_receiver_ecap_data[_rc_receiver_counter8] > MAX_CHANNEL_CYCLES))
        {
            _rc_receiver_found = 1;
        }
        if (_rc_receiver_found == 1)
        {
            if (_rc_receiver_ecap_data[_rc_receiver_counter8] > MAX_PULSE_CYCLES)
            {
                if (_rc_receiver_ecap_data[_rc_receiver_counter8] > MAX_CHANNEL_CYCLES)
                {
                    _rc_receiver_curr_channel = 0;
                }
                rc_buffer[_rc_receiver_curr_channel] = _rc_receiver_ecap_data[_rc_receiver_counter8];
                _rc_receiver_curr_channel++;
            }
        }
        if (_rc_receiver_curr_channel > 8)
        {
            _rc_receiver_curr_channel = 0;
            break;
        }
    }
}


