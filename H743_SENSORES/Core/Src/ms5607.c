/*
 * ms5607.c
 *
 *  Created on: 3 oct 2026
 *      Author: elena
 */

#include "ms5607.h"

extern I2C_HandleTypeDef hi2c1;


/* Dirección I2C del MS5607 */
#define MS5607_ADDRESS          (0x76 << 1)


/* Comandos */
#define MS5607_RESET            0x1E

#define MS5607_CONVERT_D1       0x48
#define MS5607_CONVERT_D2       0x58

#define MS5607_ADC_READ         0x00

#define MS5607_PROM_C1          0xA2
#define MS5607_PROM_C2          0xA4
#define MS5607_PROM_C3          0xA6
#define MS5607_PROM_C4          0xA8
#define MS5607_PROM_C5          0xAA
#define MS5607_PROM_C6          0xAC


static uint16_t C1;
static uint16_t C2;
static uint16_t C3;
static uint16_t C4;
static uint16_t C5;
static uint16_t C6;


static HAL_StatusTypeDef MS5607_SendCommand(uint8_t command)
{
    return HAL_I2C_Master_Transmit(
        &hi2c1,
        MS5607_ADDRESS,
        &command,
        1,
        HAL_MAX_DELAY
    );
}


static HAL_StatusTypeDef MS5607_ReadPROM(
    uint8_t command,
    uint16_t *value)
{
    uint8_t data[2];

    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(
        &hi2c1,
        MS5607_ADDRESS,
        &command,
        1,
        HAL_MAX_DELAY
    );

    if (status != HAL_OK)
        return status;

    status = HAL_I2C_Master_Receive(
        &hi2c1,
        MS5607_ADDRESS,
        data,
        2,
        HAL_MAX_DELAY
    );

    if (status != HAL_OK)
        return status;

    *value = ((uint16_t)data[0] << 8) | data[1];

    return HAL_OK;
}


static HAL_StatusTypeDef MS5607_ReadADC(uint32_t *value)
{
    uint8_t command = MS5607_ADC_READ;
    uint8_t data[3];

    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(
        &hi2c1,
        MS5607_ADDRESS,
        &command,
        1,
        HAL_MAX_DELAY
    );

    if (status != HAL_OK)
        return status;

    status = HAL_I2C_Master_Receive(
        &hi2c1,
        MS5607_ADDRESS,
        data,
        3,
        HAL_MAX_DELAY
    );

    if (status != HAL_OK)
        return status;

    *value =
        ((uint32_t)data[0] << 16) |
        ((uint32_t)data[1] << 8) |
        data[2];

    return HAL_OK;
}


HAL_StatusTypeDef MS5607_Init(void)
{
    HAL_StatusTypeDef status;

    status = MS5607_SendCommand(MS5607_RESET);

    if (status != HAL_OK)
        return status;

    HAL_Delay(3);

    status = MS5607_ReadPROM(MS5607_PROM_C1, &C1);
    if (status != HAL_OK)
        return status;

    status = MS5607_ReadPROM(MS5607_PROM_C2, &C2);
    if (status != HAL_OK)
        return status;

    status = MS5607_ReadPROM(MS5607_PROM_C3, &C3);
    if (status != HAL_OK)
        return status;

    status = MS5607_ReadPROM(MS5607_PROM_C4, &C4);
    if (status != HAL_OK)
        return status;

    status = MS5607_ReadPROM(MS5607_PROM_C5, &C5);
    if (status != HAL_OK)
        return status;

    status = MS5607_ReadPROM(MS5607_PROM_C6, &C6);
    if (status != HAL_OK)
        return status;

    return HAL_OK;
}


HAL_StatusTypeDef MS5607_Read(
    uint32_t *pressure,
    int32_t *temperature)
{
    uint32_t D1;
    uint32_t D2;

    int32_t dT;
    int64_t OFF;
    int64_t SENS;

    int32_t TEMP;
    int32_t P;

    HAL_StatusTypeDef status;


    /* Conversión de presión D1 */
    status = MS5607_SendCommand(MS5607_CONVERT_D1);

    if (status != HAL_OK)
        return status;

    HAL_Delay(10);

    status = MS5607_ReadADC(&D1);

    if (status != HAL_OK)
        return status;


    /* Conversión de temperatura D2 */
    status = MS5607_SendCommand(MS5607_CONVERT_D2);

    if (status != HAL_OK)
        return status;

    HAL_Delay(10);

    status = MS5607_ReadADC(&D2);

    if (status != HAL_OK)
        return status;


    /*
     * Cálculo de temperatura
     */

    dT = (int32_t)D2 - ((int32_t)C5 << 8);

    TEMP = 2000 + ((int64_t)dT * C6) / 8388608;


    /*
     * Cálculo de OFF y SENS
     */

    OFF =
        ((int64_t)C2 << 17) +
        ((int64_t)C4 * dT) / 64;

    SENS =
        ((int64_t)C1 << 16) +
        ((int64_t)C3 * dT) / 128;


    /*
     * Corrección de temperatura baja
     */

    int64_t T2 = 0;
    int64_t OFF2 = 0;
    int64_t SENS2 = 0;

    if (TEMP < 2000)
    {
        T2 =
            (int64_t)3 *
            ((int64_t)dT * dT) /
            8589934592LL;

        OFF2 =
            (int64_t)61 *
            ((int64_t)(TEMP - 2000) *
             (TEMP - 2000)) /
            16;

        SENS2 =
            (int64_t)2 *
            ((int64_t)(TEMP - 2000) *
             (TEMP - 2000));
    }

    TEMP -= T2;
    OFF -= OFF2;
    SENS -= SENS2;


    /*
     * Presión en mbar
     */

    P =
        (int32_t)(
            (((int64_t)D1 * SENS) / 2097152 - OFF)
            / 32768
        );


    *pressure = (uint32_t)P;
    *temperature = TEMP;

    return HAL_OK;
}
