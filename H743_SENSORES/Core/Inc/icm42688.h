/*
 * icm42688.h
 *
 *  Created on: 2 oct 2026
 *      Author: elena
 */

#ifndef INC_ICM42688_H_
#define INC_ICM42688_H_

#include "main.h"

HAL_StatusTypeDef ICM42688_Init(void);

HAL_StatusTypeDef ICM42688_ReadWhoAmI(uint8_t *value);

HAL_StatusTypeDef ICM42688_ReadAccel(
    int16_t *ax,
    int16_t *ay,
    int16_t *az
);

HAL_StatusTypeDef ICM42688_ReadGyro(
    int16_t *gx,
    int16_t *gy,
    int16_t *gz
);

#endif /* INC_ICM42688_H_ */
