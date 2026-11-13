/*
 * currentSensorDriver.c
 *
 *  Created on: Nov 3, 2026
 *      Author: jdalt
 */

/*
 *  INA260 driver + stall detection. See currentSensor.h.
 *
 *  INA260 notes:
 *   - 16-bit registers, sent MSB first
 *   - current: signed, 1.25 mA per bit
 *   - bus voltage: unsigned, 1.25 mV per bit
 *   - HAL wants the 7-bit address shifted left by one
 */

#include "currentSensorDriver.h"

/* Registers */
#define INA260_REG_CONFIG     0x00
#define INA260_REG_CURRENT    0x01
#define INA260_REG_BUS_V      0x02
#define INA260_REG_MFG_ID     0xFE

#define INA260_MFG_ID         0x5449   /* "TI" */
#define INA260_RESET          0x8000

/*
 * Config register:
 *   bits 14-12 : 110 (fixed)
 *   AVG    001 : average 4 samples
 *   VBUSCT 000 : 140 us   (voltage is only telemetry, keep it fast)
 *   ISHCT  100 : 1.1 ms   (current is what we care about)
 *   MODE   111 : current + voltage, continuous
 * New value about every 4 x (1.1 ms + 0.14 ms) = ~5 ms.
 */
#define INA260_CONFIG  (0x6000u | (0x1u << 9) | (0x0u << 6) | (0x4u << 3) | 0x7u)

static HAL_StatusTypeDef reg_read(CurrentSensor *s, uint8_t reg, uint16_t *val)
{
    uint8_t buf[2];
    HAL_StatusTypeDef st = HAL_I2C_Mem_Read(s->hi2c, (uint16_t)(s->addr << 1), reg,
                                            I2C_MEMADD_SIZE_8BIT, buf, 2,
                                            CS_I2C_TIMEOUT_MS);
    if (st == HAL_OK) *val = (uint16_t)((buf[0] << 8) | buf[1]);
    return st;
}

static HAL_StatusTypeDef reg_write(CurrentSensor *s, uint8_t reg, uint16_t val)
{
    uint8_t buf[2] = { (uint8_t)(val >> 8), (uint8_t)(val & 0xFF) };
    return HAL_I2C_Mem_Write(s->hi2c, (uint16_t)(s->addr << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, buf, 2, CS_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef CS_Init(CurrentSensor *s, I2C_HandleTypeDef *hi2c)
{
    uint16_t id = 0;

    s->hi2c        = hi2c;
    s->currentmA   = 0;
    s->busmV       = 0;
    s->present     = 0;
    s->healthy     = 0;
    s->errorCount  = 0;
    s->over        = 0;
    s->overSinceMs = 0;
    s->lastReadMs  = HAL_GetTick();

    /* Is the right chip at this address? */
    if (reg_read(s, INA260_REG_MFG_ID, &id) != HAL_OK) return HAL_ERROR;
    if (id != INA260_MFG_ID)                           return HAL_ERROR;

    /* Reset, then configure */
    if (reg_write(s, INA260_REG_CONFIG, INA260_RESET)  != HAL_OK) return HAL_ERROR;
    if (reg_write(s, INA260_REG_CONFIG, INA260_CONFIG) != HAL_OK) return HAL_ERROR;

    s->present = 1;
    s->healthy = 1;
    return HAL_OK;
}

void CS_Update(CurrentSensor *s)
{
    if (!s->present) return;                     /* never found: stays unhealthy */

    uint32_t now = HAL_GetTick();
    if (now - s->lastReadMs < CS_READ_PERIOD_MS) return;
    s->lastReadMs = now;

    uint16_t rawI, rawV;
    if (reg_read(s, INA260_REG_CURRENT, &rawI) == HAL_OK &&
        reg_read(s, INA260_REG_BUS_V,   &rawV) == HAL_OK)
    {
        s->currentmA  = ((int32_t)(int16_t)rawI * 5) / 4;   /* x 1.25 mA */
        s->busmV      = ((uint32_t)rawV * 5) / 4;           /* x 1.25 mV */
        s->errorCount = 0;
        s->healthy    = 1;
    }
    else
    {
        if (s->errorCount < 255) s->errorCount++;
        if (s->errorCount >= CS_MAX_ERRORS) s->healthy = 0;
    }
}

uint8_t CS_CheckStall(CurrentSensor *s, uint32_t runStartMs)
{
    uint32_t now = HAL_GetTick();

    /* Startup inrush looks like a stall, so ignore it */
    if (now - runStartMs < s->blankMs)
    {
        s->over = 0;
        return 0;
    }

    /* Use the size only, so it works whichever way the sensor is wired */
    int32_t mag = (s->currentmA < 0) ? -s->currentmA : s->currentmA;

    if (mag >= (int32_t)s->stallmA)
    {
        if (!s->over)
        {
            s->over        = 1;
            s->overSinceMs = now;
        }
        return (now - s->overSinceMs) >= s->stallMs;
    }

    s->over = 0;                                 /* dipped below: start over */
    return 0;
}

void CS_ClearStall(CurrentSensor *s)
{
    s->over = 0;
}




