#include <stdint.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <resource_table.h>
#include <pru_i2c_driver.h>
#include <pru_hmc5883l_driver.h>
#include <MPU6050.h>
#include <pru_mpu6050_driver.h>
#include <pru_ecap.h>
#include <pru_edma.h>
#include <rcReceiver.h>

/**
 * main.c
 */
volatile register uint32_t __R30;
volatile register uint32_t __R31;

unsigned char pru0_data[sizeof(PrbMessageType)] = { '\0' };
PrbMessageType* pru0_data_struct = (PrbMessageType*) pru0_data;
uint8_t counter8 = 0;
uint8_t currRcChannel = 0;
uint8_t found = 0;

#define MPU_SENSOR_NUM          0
#define COMPASS_SENSOR_NUM      1
#define BAROMETER_SENSOR_NUM    2
#define RC_SENSOR_NUM           3

#define INT_ECAP       15
#define INT_ECAP_CHAN  8
#define INT_ECAP_HOST  8

uint32_t active_sensors = 0;
uint8_t testConnectionOk = 0;
uint8_t rc_receiver_newData = 0;

//uint32_t* CM_PER_PWMCSS_CLKCTRL[3] = { (uint32_t*) 0x44E000D4,
//                                       (uint32_t*) 0x44E000CC,
//                                       (uint32_t*) 0x44E000D8 };
//uint32_t* PWMSS_CTRL_REG = (uint32_t*) 0x44E10664;
//uint32_t* ECAP_CTRL_PIN = (uint32_t*) 0x44E10964;

#define PRU_CTRL_CTR_EN 0x8
uint32_t* PRU_CTRL = (uint32_t*) 0x00024000;
uint32_t* PRU_CYCLE = (uint32_t*) 0x0002400C;
uint32_t BUFF_DEBUG[4] = { 0 };
uint32_t RC_BUFFER[9] = { 0 };

int main(void)
{
    volatile uint32_t *ptr = EDMA0_CC_BASE;
    uint32_t edmaChannelMask = (1 << EDMA3CC_ECAP0_EVT);
    uint32_t* ecapData;

    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

    /* Clear the status of the registers that will be used in this programs
     * As they have been un-serviced in the last software tun
     */
    CT_INTC.SICR_bit.STS_CLR_IDX = INT_P1_TO_P0;
    CT_INTC.SICR_bit.STS_CLR_IDX = INT_P0_TO_P1;

    CT_INTC.EISR_bit.EN_SET_IDX = INT_P0_TO_P1; // enable interrupt from PRU0
    CT_INTC.EISR_bit.EN_SET_IDX = INT_P1_TO_P0;

    CT_INTC.CMR3_bit.CH_MAP_15 = INT_ECAP_CHAN;
    CT_INTC.HMR2_bit.HINT_MAP_8 = INT_ECAP_HOST;
    CT_INTC.HIER_bit.EN_HINT |= INT_ECAP_HOST; // enable host interrupt 8
    CT_INTC.EISR_bit.EN_SET_IDX = INT_ECAP; // enable ecap interrupt
    CT_INTC.GER_bit.EN_HINT_ANY = 1; // enable all host interrupt

    /*
     * Forzo RC attivo
     */
    rc_receiver_Init();
    rc_receiver_Start();

    // TODO: gestire multiple istanze mpu e su canali i2c differenti
    uint8_t i2cInit = pru_i2c_driver_Init(2);
    testConnectionOk = pru_mpu6050_driver_TestConnection();
    while (1)
    {
        BUFF_DEBUG[0] = (uint32_t) &(ptr[DRAE1]);
        BUFF_DEBUG[1] = (uint32_t) &(ptr[IPR]);

        // EDMA completion interrupt (must be cleared)
        if (rc_receiver_newData = rc_receiver_PulseNewData())
        {
            // TODO: inviare l'intero buffer a pru0
            if(rc_receiver_newData & RC_RECEIVER_TX_NOT_PRESENT) {
                for(counter8 = 0; counter8 < 9; counter8++) {
                    RC_BUFFER[counter8] = 0;
                }
            } else {
                ecapData = edma_get_Data();
                found = 0;

                for (counter8 = 0; counter8 < NUM_EDMA_FRAME_BLOCK; counter8++)
                {
                    if ((found == 0) && (ecapData[counter8] > 400000))
                    {
                        found = 1;
                    }
                    if (found == 1)
                    {
                        if (ecapData[counter8] > 70000)
                        {
                            if (ecapData[counter8] > 400000)
                            {
                                currRcChannel = 0;
                            }
                            RC_BUFFER[currRcChannel] = ecapData[counter8];
                            currRcChannel++;
                        }
                    }
                    if (currRcChannel > 8)
                    {
                        currRcChannel = 0;
                        break;
                    }
                }
            }
        }
        else
        // receive message from PRU0
        if (CT_INTC.SECR0_bit.ENA_STS_31_0 & (1 << INT_P0_TO_P1))
        {
            CT_INTC.SICR_bit.STS_CLR_IDX = INT_P0_TO_P1;
            __xin(SP_BANK_0, 3, 0, pru0_data);
            switch (pru0_data_struct->message_type)
            {
            case MPU_ENABLE_MSG_TYPE:
            {
                pru_mpu6050_driver_Initialize();
                active_sensors |= (1 << MPU_SENSOR_NUM);
                break;
            }
            case MPU_DISABLE_MSG_TYPE:
            {
                // TODO: eseguire stop del sensore
                active_sensors &= ~(1 << MPU_SENSOR_NUM);
                break;
            }
            case RC_ENABLE_MSG_TYPE:
            {
                ecap_Init();
                edma_Init();
                ecap_Start();
                active_sensors |= (1 << RC_SENSOR_NUM);
                break;
            }
            case RC_DISABLE_MSG_TYPE:
            {
                ecap_Stop();
                active_sensors &= ~(1 << RC_SENSOR_NUM);
                break;
            }
            }
            // TODO: interpret message
        }
        else if (active_sensors & (1 << BAROMETER_SENSOR_NUM))
        {
            // TODO: read data from baro
            // TODO: send data to pru0
        }
        else if (active_sensors & (1 << COMPASS_SENSOR_NUM))
        {
            // TODO: read data from compass
            // TODO: send data to pru0
        }
        else if (active_sensors & (1 << RC_SENSOR_NUM))
        {
            // TODO: read data from buffer
            // TODO: send data to pru0
            uint32_t* buffer = edma_get_Data();

        }
        else if (active_sensors & (1 << MPU_SENSOR_NUM))
        {
            // TODO: Inserire interrupt invece di leggere via i2c
            if (pru_mpu6050_driver_GetIntDataReadyStatus())
            {
                pru_mpu6050_driver_GetMotion6(
                        &(pru0_data_struct->mpu_accel_gyro.ax),
                        &(pru0_data_struct->mpu_accel_gyro.ay),
                        &(pru0_data_struct->mpu_accel_gyro.az),
                        &(pru0_data_struct->mpu_accel_gyro.gx),
                        &(pru0_data_struct->mpu_accel_gyro.gy),
                        &(pru0_data_struct->mpu_accel_gyro.gz));

                pru0_data_struct->message_type = MPU_DATA_MSG_TYPE;
                // send data to PRU0
                __xout(SP_BANK_1, 6, 0, pru0_data);
                // send interrupt to P0
                CT_INTC.SRSR0_bit.RAW_STS_31_0 |= (1 << INT_P1_TO_P0);
            }
        }
    }
}
