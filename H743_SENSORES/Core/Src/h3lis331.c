/*
 * h3lis331.c
 *
 *  Created on: 3 oct 2026
 *      Author: elena
 */

#include "h3lis331.h"

extern SPI_HandleTypeDef hspi1;


/* Registros */
#define H3LIS331_WHO_AM_I     0x0F
#define H3LIS331_CTRL_REG1    0x20
#define H3LIS331_CTRL_REG4    0x23
#define H3LIS331_OUT_X_L      0x28


static HAL_StatusTypeDef H3LIS331_WriteRegister(uint8_t reg, uint8_t data)
{
    uint8_t txData[2];

    txData[0] = reg;
    txData[1] = data;

    HAL_GPIO_WritePin(
        CS_H3LIS_GPIO_Port,
        CS_H3LIS_Pin,
        GPIO_PIN_RESET
    );

    HAL_StatusTypeDef status = HAL_SPI_Transmit(
        &hspi1,
        txData,
        2,
        HAL_MAX_DELAY
    );

    HAL_GPIO_WritePin(
        CS_H3LIS_GPIO_Port,
        CS_H3LIS_Pin,
        GPIO_PIN_SET
    );

    return status;
}


static HAL_StatusTypeDef H3LIS331_ReadRegisters(
    uint8_t reg,
    uint8_t *data,
    uint16_t length)
{
    uint8_t command = reg | 0x80;

    if (length > 1)
    {
        command |= 0x40;
    }

    HAL_GPIO_WritePin(
        CS_H3LIS_GPIO_Port,
        CS_H3LIS_Pin,
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
        CS_H3LIS_GPIO_Port,
        CS_H3LIS_Pin,
        GPIO_PIN_SET
    );

    return status;
}


HAL_StatusTypeDef H3LIS331_ReadWhoAmI(uint8_t *value)
{
    return H3LIS331_ReadRegisters(
        H3LIS331_WHO_AM_I,
        value,
        1
    );
}


HAL_StatusTypeDef H3LIS331_Init(void)
{
    HAL_StatusTypeDef status;

    /*
     * CTRL_REG1:
     * 100 Hz + XYZ habilitados
     */
    status = H3LIS331_WriteRegister(
        H3LIS331_CTRL_REG1,
        0x57
    );

    if (status != HAL_OK)
    {
        return status;
    }

    /*
     * CTRL_REG4:
     * ±400 g
     */
    status = H3LIS331_WriteRegister(
        H3LIS331_CTRL_REG4,
        0x30
    );

    return status;
}


HAL_StatusTypeDef H3LIS331_ReadAccel(
    int16_t *ax,
    int16_t *ay,
    int16_t *az)
{
    uint8_t data[6];

    HAL_StatusTypeDef status;

    status = H3LIS331_ReadRegisters(
        H3LIS331_OUT_X_L,
        data,
        6
    );

    if (status != HAL_OK)
    {
        return status;
    }

    *ax = (int16_t)((data[1] << 8) | data[0]);
    *ay = (int16_t)((data[3] << 8) | data[2]);
    *az = (int16_t)((data[5] << 8) | data[4]);

    return HAL_OK;
}
