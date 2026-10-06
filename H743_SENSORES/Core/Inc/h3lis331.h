/*
 * h3lis331.h
 *
 *  Created on: 3 oct 2026
 *      Author: elena
 */

#ifndef INC_H3LIS331_H_
#define INC_H3LIS331_H_

#include "main.h"

HAL_StatusTypeDef H3LIS331_Init(void);

HAL_StatusTypeDef H3LIS331_ReadWhoAmI(
    uint8_t *value
);

HAL_StatusTypeDef H3LIS331_ReadAccel(
    int16_t *ax,
    int16_t *ay,
    int16_t *az
);

#endif /* INC_H3LIS331_H_ */
