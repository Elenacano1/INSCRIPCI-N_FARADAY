/*
 * icm42688.c
 *
 *  Created on: 2 oct 2026
 *      Author: elena
 */


#include "icm42688.h"

extern SPI_HandleTypeDef hspi1;


/* Registros */
#define ICM42688_WHO_AM_I       0x75
#define ICM42688_PWR_MGMT0      0x4E
#define ICM42688_ACCEL_CONFIG0  0x50
#define ICM42688_GYRO_CONFIG0   0x4F

#define ICM42688_ACCEL_DATA_X1  0x1F
#define ICM42688_GYRO_DATA_X1   0x25


static HAL_StatusTypeDef ICM42688_WriteRegister(
    uint8_t reg,
    uint8_t data)
{
    uint8_t txData[2];

    txData[0] = reg & 0x7F;
    txData[1] = data;

    HAL_GPIO_WritePin(
        CS_ICM_GPIO_Port,
        CS_ICM_Pin,
        GPIO_PIN_RESET
    );

    HAL_StatusTypeDef status = HAL_SPI_Transmit(
        &hspi1,
        txData,
        2,
        HAL_MAX_DELAY
    );

    HAL_GPIO_WritePin(
        CS_ICM_GPIO_Port,
        CS_ICM_Pin,
        GPIO_PIN_SET
    );

    return status;
}


static HAL_StatusTypeDef ICM42688_ReadRegisters(
    uint8_t reg,
    uint8_t *data,
    uint16_t length)
{
    uint8_t command = reg | 0x80;

    HAL_GPIO_WritePin(
        CS_ICM_GPIO_Port,
        CS_ICM_Pin,
        GPIO_PIN_RESET
    );

    HAL_StatusTypeDef status = HAL_SPI_Transmit(
        &hspi1,
        &command,
        1,
        HAL_MAX_DELAY
    );

    if (status == HAL_OK)
    {
        status = HAL_SPI_Receive(
            &hspi1,
            data,
            length,
            HAL_MAX_DELAY
        );
    }

    HAL_GPIO_WritePin(
        CS_ICM_GPIO_Port,
        CS_ICM_Pin,
        GPIO_PIN_SET
    );

    return status;
}


HAL_StatusTypeDef ICM42688_ReadWhoAmI(uint8_t *value)
{
    return ICM42688_ReadRegisters(
        ICM42688_WHO_AM_I,
        value,
        1
    );
}


HAL_StatusTypeDef ICM42688_Init(void)
{
    HAL_StatusTypeDef status;

    /*
     * Habilitar acelerómetro y giroscopio
     */
    status = ICM42688_WriteRegister(
        ICM42688_PWR_MGMT0,
        0x0F
    );

    if (status != HAL_OK)
    {
        return status;
    }

    HAL_Delay(50);

    /*
     * Acelerómetro:
     * ±16 g
     */
    status = ICM42688_WriteRegister(
        ICM42688_ACCEL_CONFIG0,
        0x06
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * Giroscopio:
     * ±2000 °/s
     */
    status = ICM42688_WriteRegister(
        ICM42688_GYRO_CONFIG0,
        0x06
    );

    return status;
}


HAL_StatusTypeDef ICM42688_ReadAccel(
    int16_t *ax,
    int16_t *ay,
    int16_t *az)
{
    uint8_t data[6];

    HAL_StatusTypeDef status;

    status = ICM42688_ReadRegisters(
        ICM42688_ACCEL_DATA_X1,
        data,
        6
    );

    if (status != HAL_OK)
    {
        return status;
    }

    *ax = (int16_t)((data[0] << 8) | data[1]);
    *ay = (int16_t)((data[2] << 8) | data[3]);
    *az = (int16_t)((data[4] << 8) | data[5]);

    return HAL_OK;
}


HAL_StatusTypeDef ICM42688_ReadGyro(
    int16_t *gx,
    int16_t *gy,
    int16_t *gz)
{
    uint8_t data[6];

    HAL_StatusTypeDef status;

    status = ICM42688_ReadRegisters(
        ICM42688_GYRO_DATA_X1,
        data,
        6
    );

    if (status != HAL_OK)
    {
        return status;
    }

    *gx = (int16_t)((data[0] << 8) | data[1]);
    *gy = (int16_t)((data[2] << 8) | data[3]);
    *gz = (int16_t)((data[4] << 8) | data[5]);

    return HAL_OK;
}
