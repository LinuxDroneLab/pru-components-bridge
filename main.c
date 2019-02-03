#include <stdint.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <resource_table.h>
#include <pru_i2c_driver.h>
#include <pru_hmc5883l_driver.h>
#include <MPU6050.h>
#include <pru_mpu6050_driver.h>
#include <ecap.h>
#include <pru_ecap.h>
#include <edma.h>

/**
 * main.c
 */
volatile register uint32_t __R30;
volatile register uint32_t __R31;

unsigned char pru0_data[sizeof(PrbMessageType)] = { '\0' };
PrbMessageType* pru0_data_struct = (PrbMessageType*) pru0_data;
uint32_t counter = 0;

#define MPU_SENSOR_NUM          0
#define COMPASS_SENSOR_NUM      1
#define BAROMETER_SENSOR_NUM    2
#define RC_SENSOR_NUM           3

#define INT_ECAP       15
#define INT_ECAP_CHAN  8
#define INT_ECAP_HOST  8

uint32_t active_sensors = 0;
uint8_t testConnectionOk = 0;

//uint32_t* CM_PER_PWMCSS_CLKCTRL[3] = { (uint32_t*) 0x44E000D4,
//                                       (uint32_t*) 0x44E000CC,
//                                       (uint32_t*) 0x44E000D8 };
//uint32_t* PWMSS_CTRL_REG = (uint32_t*) 0x44E10664;
//uint32_t* ECAP_CTRL_PIN = (uint32_t*) 0x44E10964;


int main(void)
{
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
    CT_INTC.HIER_bit.EN_HINT |= 0x8;
    CT_INTC.EISR_bit.EN_SET_IDX = INT_ECAP; // enable ecap interrupt
    CT_INTC.GER_bit.EN_HINT_ANY = 1; // enable host interrupts
    /*
     * per prova. Da togliere
     */
    ecap_Init();
    edma_Init();
    ecap_Start();


    // TODO: gestire multiple istanze mpu e su canali i2c differenti
    uint8_t i2cInit = pru_i2c_driver_Init(2);
    testConnectionOk = pru_mpu6050_driver_TestConnection();
    while (1)
    {
        // Reset Ecap interrupt
        if (CT_ECAP.ECFLG & 0x0002)
        {
            CT_ECAP.ECCLR |= ECCLR_MSK; // remove EVT4 interrupt and INT
        } else

        // receive message from PRU0
        if (CT_INTC.SECR0_bit.ENA_STS_31_0 & (1<<INT_P0_TO_P1))
        {
            CT_INTC.SICR_bit.STS_CLR_IDX = INT_P0_TO_P1;
            __xin(SP_BANK_0, 3, 0, pru0_data);
            switch(pru0_data_struct->message_type) {
            case MPU_ENABLE_MSG_TYPE: {
                pru_mpu6050_driver_Initialize();
                active_sensors |= (1 << MPU_SENSOR_NUM);
                break;
            }
            case MPU_DISABLE_MSG_TYPE: {
                // TODO: eseguire stop del sensore
                active_sensors &= ~(1 << MPU_SENSOR_NUM);
                break;
            }
            case RC_ENABLE_MSG_TYPE: {
                ecap_Init();
                edma_Init();
                ecap_Start();
                active_sensors |= (1 << RC_SENSOR_NUM);
                break;
            }
            case RC_DISABLE_MSG_TYPE: {
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
                CT_INTC.SRSR0_bit.RAW_STS_31_0 |= (1<<INT_P1_TO_P0);
            }
        }
    }
}
