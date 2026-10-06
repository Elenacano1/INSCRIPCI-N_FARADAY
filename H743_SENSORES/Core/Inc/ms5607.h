/*
 * ms5607.h
 *
 *  Created on: 3 oct 2026
 *      Author: elena
 */

#ifndef INC_MS5607_H_
#define INC_MS5607_H_

#include "main.h"

HAL_StatusTypeDef MS5607_Init(void);

HAL_StatusTypeDef MS5607_Read(
    uint32_t *pressure,
    int32_t *temperature
);

#endif /* INC_MS5607_H_ */
