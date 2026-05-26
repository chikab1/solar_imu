#ifndef __LSM6DS_H
#define __LSM6DS_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lsm6ds3tr-c_reg.h"
#include "i2c.h"

/* Exported functions prototypes ---------------------------------------------*/
uint8_t LSM6DS_Init(I2C_HandleTypeDef *hi2c);
uint8_t LSM6DS_Read_Storage(float *acc_mg, float *gyro_dps);
void LSM6DS_Config_Wakeup(uint8_t threshold, uint8_t duration);

#ifdef __cplusplus
}
#endif

#endif /* __LSM6DS_H */
