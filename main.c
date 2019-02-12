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
#define MPU_SENSOR_NUM          0
#define COMPASS_SENSOR_NUM      1
#define BAROMETER_SENSOR_NUM    2
#define RC_SENSOR_NUM           3

/*
 * Cycle Counter
 */
#define PRU_CTRL_CTR_EN 0x8
uint32_t* PRU_CTRL = (uint32_t*) 0x00024000;
uint32_t* PRU_CYCLE = (uint32_t*) 0x0002400C;

/*
 * Variables
 */
volatile register uint32_t __R30;
volatile register uint32_t __R31;

unsigned char pru0_data[sizeof(PrbMessageType)] = { '\0' };
PrbMessageType* pru0_data_struct = (PrbMessageType*) pru0_data;

uint32_t active_sensors = 0;
uint8_t testConnectionOk = 0;
uint8_t rc_receiver_newData = 0;
uint8_t counter8 = 0;

uint32_t BUFF_DEBUG[4] = { 0 };
uint32_t RC_BUFFER[9] = { 0 };

//uint32_t* CM_PER_PWMCSS_CLKCTRL[3] = { (uint32_t*) 0x44E000D4,
//                                       (uint32_t*) 0x44E000CC,
//                                       (uint32_t*) 0x44E000D8 };
//uint32_t* PWMSS_CTRL_REG = (uint32_t*) 0x44E10664;
//uint32_t* ECAP_CTRL_PIN = (uint32_t*) 0x44E10964;

inline uint8_t is_enabled_RC() {
    return active_sensors & (1 << RC_SENSOR_NUM);
}

void enable_RC()
{
    if(!is_enabled_RC()) {
        rc_receiver_Init();
        rc_receiver_Start();
        active_sensors |= (1 << RC_SENSOR_NUM);
    }
}

void disable_RC()
{
    if(is_enabled_RC()) {
        rc_receiver_Stop();
        active_sensors &= ~(1 << RC_SENSOR_NUM);
    }
}

inline uint32_t is_enabled_Barometer()
{
    return active_sensors & (1 << BAROMETER_SENSOR_NUM);
}

inline uint32_t is_enabled_Compass()
{
    return active_sensors & (1 << COMPASS_SENSOR_NUM);
}

inline uint32_t is_enabled_MPU()
{
    return active_sensors & (1 << MPU_SENSOR_NUM);
}
void enable_MPU()
{
    if(!is_enabled_MPU()) {
        pru_mpu6050_driver_Initialize();
        active_sensors |= (1 << MPU_SENSOR_NUM);
    }
}

void disable_MPU()
{
    // TODO: eseguire stop del sensore
    active_sensors &= ~(1 << MPU_SENSOR_NUM);
}

inline void send_interrupt_to_pru0()
{
    CT_INTC.SRSR0_bit.RAW_STS_31_0 |= (1 << INT_P1_TO_P0);
}

void send_data_to_pru0()
{
    __xout(SP_BANK_1, 6, 0, pru0_data);
    send_interrupt_to_pru0();
}

uint8_t exists_new_message_from_pru0()
{
    if(CT_INTC.SECR0_bit.ENA_STS_31_0 & (1 << INT_P0_TO_P1)) {
        return 1;
    }
    return 0;
}

uint8_t receive_data_from_pru0()
{
    uint8_t result = 0;
    if(CT_INTC.SECR0_bit.ENA_STS_31_0 & (1 << INT_P0_TO_P1)) {
        CT_INTC.SICR_bit.STS_CLR_IDX = INT_P0_TO_P1;
        __xin(SP_BANK_0, 3, 0, pru0_data);
        result = 1;
    }
    return result;
}

void clean_rc_buffer()
{
    for (counter8 = 0; counter8 < 9; counter8++)
    {
        RC_BUFFER[counter8] = 0;
    }
}

int main(void)
{
// TODO: eliminare. Assegnati già da PRU0
    CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;
    // clear interrupts
    CT_INTC.SICR_bit.STS_CLR_IDX = INT_P1_TO_P0;
    CT_INTC.SICR_bit.STS_CLR_IDX = INT_P0_TO_P1;

    CT_INTC.EISR_bit.EN_SET_IDX = INT_P0_TO_P1; // enable interrupt from PRU0
    CT_INTC.EISR_bit.EN_SET_IDX = INT_P1_TO_P0;

    /*
     * Forzo RC attivo
     */
    enable_RC();

    // TODO: gestire multiple istanze mpu e su canali i2c differenti
    uint8_t i2cInit = pru_i2c_driver_Init(2);
    testConnectionOk = pru_mpu6050_driver_TestConnection();

    while (1)
    {
        BUFF_DEBUG[0] = 0xF;
        BUFF_DEBUG[1] = 0xF0;

        // Rules: ordered by priority
        if(is_enabled_RC()) {
            /* Check for New Data from RC
             * TX not connected <=> rc_receiver_newData & RC_RECEIVER_TX_NOT_PRESENT
             * New Data <=> rc_receiver_newData & RC_RECEIVER_TX_COMPLETE
             */
            if (rc_receiver_newData = rc_receiver_PulseNewData())
            {
                // TODO: inviare l'intero buffer a pru0
                if(rc_receiver_newData & RC_RECEIVER_TX_NOT_PRESENT) {
                    clean_rc_buffer();
                } else {
                    rc_receiver_extract_Data(RC_BUFFER);
                }
            }
        }
        else
        // receive message from PRU0
        if (receive_data_from_pru0())
        {
            switch (pru0_data_struct->message_type)
            {
            case MPU_ENABLE_MSG_TYPE:
            {
                BUFF_DEBUG[0] = 0xFFFF;
                enable_MPU();
                break;
            }
            case MPU_DISABLE_MSG_TYPE:
            {
                disable_MPU();
                break;
            }
            case RC_ENABLE_MSG_TYPE:
            {
                enable_RC();
                break;
            }
            case RC_DISABLE_MSG_TYPE:
            {
                disable_RC();
                break;
            }
            }
            // TODO: interpret message
        }
        else if (is_enabled_Barometer())
        {
            // TODO: read data from baro
            // TODO: send data to pru0
        }
        else if (is_enabled_Compass())
        {
            // TODO: read data from compass
            // TODO: send data to pru0
        }
        else if (is_enabled_MPU())
        {
            BUFF_DEBUG[1] = 0xFFFF0000;

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
                send_data_to_pru0();
            }
        }
    }
}
