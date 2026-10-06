/*
 * max_m10s.h
 *
 *  Created on: 3 oct 2026
 *      Author: elena
 */

#ifndef INC_MAX_M10S_H_
#define INC_MAX_M10S_H_

#include "main.h"

HAL_StatusTypeDef MAX_M10S_Init(void);

HAL_StatusTypeDef MAX_M10S_StartReception(void);

void MAX_M10S_ProcessByte(uint8_t byte);


#endif /* INC_MAX_M10S_H_ */
