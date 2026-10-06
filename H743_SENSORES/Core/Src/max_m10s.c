/*
 * max_m10s.c
 *
 *  Created on: 3 oct 2026
 *      Author: elena
 */


#include "max_m10s.h"
#include <string.h>

extern UART_HandleTypeDef huart3;


/* Buffer para recibir datos del GNSS */
#define GNSS_BUFFER_SIZE 256

static uint8_t rxByte;

static uint8_t gnssBuffer[GNSS_BUFFER_SIZE];
static uint16_t gnssIndex = 0;


/* Última trama NMEA completa recibida */
static char gnssMessage[GNSS_BUFFER_SIZE];


/* --------------------------------------------------------- */
/* Inicialización                                            */
/* --------------------------------------------------------- */

HAL_StatusTypeDef MAX_M10S_Init(void)
{
    gnssIndex = 0;

    memset(gnssBuffer, 0, sizeof(gnssBuffer));
    memset(gnssMessage, 0, sizeof(gnssMessage));

    return HAL_OK;
}


/* --------------------------------------------------------- */
/* Comenzar recepción UART                                   */
/* --------------------------------------------------------- */

HAL_StatusTypeDef MAX_M10S_StartReception(void)
{
    return HAL_UART_Receive_IT(
        &huart3,
        &rxByte,
        1
    );
}


/* --------------------------------------------------------- */
/* Procesar cada byte recibido                               */
/* --------------------------------------------------------- */

void MAX_M10S_ProcessByte(uint8_t byte)
{
    /*
     * Comenzamos una nueva trama cuando aparece '$'
     */
    if (byte == '$')
    {
        gnssIndex = 0;

        gnssBuffer[gnssIndex++] = byte;

        return;
    }


    /*
     * Si todavía no ha comenzado una trama,
     * ignoramos el byte.
     */
    if (gnssIndex == 0)
    {
        return;
    }


    /*
     * Guardamos el byte
     */
    if (gnssIndex < GNSS_BUFFER_SIZE - 1)
    {
        gnssBuffer[gnssIndex++] = byte;
    }
    else
    {
        /*
         * Si el buffer se llena,
         * descartamos la trama.
         */
        gnssIndex = 0;

        return;
    }


    /*
     * Una trama NMEA termina en \n
     */
    if (byte == '\n')
    {
        gnssBuffer[gnssIndex] = '\0';

        memcpy(
            gnssMessage,
            gnssBuffer,
            gnssIndex + 1
        );

        gnssIndex = 0;
    }
}


/* --------------------------------------------------------- */
/* Callback UART                                             */
/* --------------------------------------------------------- */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        MAX_M10S_ProcessByte(rxByte);

        HAL_UART_Receive_IT(
            &huart3,
            &rxByte,
            1
        );
    }
}
