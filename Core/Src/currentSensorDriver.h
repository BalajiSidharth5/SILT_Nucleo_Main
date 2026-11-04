/*
 * currentSensorDriver.h
 *
 *  Created on: Nov 3, 2026
 *      Author: jdalt
 *
 *
 *  Adresses assigned as 0x40 and 0x4F using A0 and A1 jumper wires
 */

#ifndef SRC_CURRENTSENSORDRIVER_H_
#define SRC_CURRENTSENSORDRIVER_H_

#include "main.h"

#define CS_I2C_TIMEOUT_MS   3u   /* short, so a bad cable can't freeze the loop   */
#define CS_READ_PERIOD_MS   5u   /* matches the ~5 ms conversion set in CS_Init   */
#define CS_MAX_ERRORS       3u   /* failed reads in a row before marked unhealthy */

typedef struct {
    /* ---- set these before CS_Init (see winchSystem.c) ---- */
    uint8_t  addr;          /* 7-bit address, 0x40 - 0x4F                      */
    uint16_t stallmA;       /* current at or above this counts as "too high"   */
    uint16_t stallMs;       /* must stay too high this long to count as stall  */
    uint16_t blankMs;       /* ignore startup inrush for this long after start */

    /* ---- filled in by the driver, read-only for everyone else ---- */
    I2C_HandleTypeDef *hi2c;
    int32_t  currentmA;     /* last reading, signed                            */
    uint32_t busmV;         /* last reading                                    */
    uint8_t  present;       /* 1 = found and configured at init                */
    uint8_t  healthy;       /* 1 = recent reads are succeeding                 */
    uint8_t  errorCount;
    uint8_t  over;          /* 1 = currently above stallmA                     */
    uint32_t overSinceMs;
    uint32_t lastReadMs;
} CurrentSensor;

/* Checks the chip ID and configures it. Call once after MX_I2C1_Init(). */
HAL_StatusTypeDef CS_Init(CurrentSensor *s, I2C_HandleTypeDef *hi2c);

/* Reads current and voltage, at most every CS_READ_PERIOD_MS. Call every loop. */
void CS_Update(CurrentSensor *s);

/* Returns 1 once the current has stayed above stallmA for stallMs.
 * runStartMs = when the motor started driving (used for the blanking window). */
uint8_t CS_CheckStall(CurrentSensor *s, uint32_t runStartMs);

/* Clears the over-current timer. Call whenever the motor is not driving. */
void CS_ClearStall(CurrentSensor *s);

#endif /* SRC_CURRENTSENSORDRIVER_H_ */
