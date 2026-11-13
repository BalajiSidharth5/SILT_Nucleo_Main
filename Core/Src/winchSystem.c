/*
 * winchSystem.c
 *
 *  Created on: Nov 3, 2026
 *      Author: jdalt
 *
 *  Handles communication logic between currentSensorDriver and winchDriver
 *
 */


#include "winchSystem.h"

/* ------------------------------------------------------------------------
 * Winch pins (CubeMX user labels)
 * Winch 2 is only built once the W2_* labels exist in CubeMX/main.h.
 * ---------------------------------------------------------------------- */
static Winch winches[] = {
    { .rh = {Right_High_GPIO_Port, Right_High_Pin}, .rl = {Right_Low_GPIO_Port, Right_Low_Pin},
      .lh = {Left_High_GPIO_Port,  Left_High_Pin},  .ll = {Left_Low_GPIO_Port,  Left_Low_Pin} },
#if defined(W2_Right_High_Pin)
    { .rh = {W2_Right_High_GPIO_Port, W2_Right_High_Pin}, .rl = {W2_Right_Low_GPIO_Port, W2_Right_Low_Pin},
      .lh = {W2_Left_High_GPIO_Port,  W2_Left_High_Pin},  .ll = {W2_Left_Low_GPIO_Port,  W2_Left_Low_Pin} },
#endif
};


/*
 * so winches[0] winch is associated with sensors[0] sensor
 */
static CurrentSensor sensors[] = {
    { .addr = 0x40, .stallmA = 2000, .stallMs = 100, .blankMs = 300 },
#if defined(W2_Right_High_Pin)
    { .addr = 0x41, .stallmA = 2000, .stallMs = 100, .blankMs = 300 },
#endif
};

#define WINCH_COUNT  ((uint8_t)(sizeof(winches) / sizeof(winches[0])))


static uint8_t commands[sizeof(winches) / sizeof(winches[0])];



void WinchSys_Init(I2C_HandleTypeDef *hi2c)
{
    for (uint8_t i = 0; i < WINCH_COUNT; i++)
    {
        commands[i] = 0;
        Winch_Init(&winches[i]);

        /* No working sensor = no stall protection = not allowed to run */
        if (CS_Init(&sensors[i], hi2c) != HAL_OK)
            Winch_Fault(&winches[i]);
    }
}

void WinchSys_SetCommand(uint8_t index, uint8_t cmd)
{
    if (index < WINCH_COUNT) commands[index] = cmd;
}

void WinchSys_Update(void)
{
    for (uint8_t i = 0; i < WINCH_COUNT; i++)
    {
        Winch         *w = &winches[i];
        CurrentSensor *s = &sensors[i];

        CS_Update(s);

        /* If the sensor stops answering, treat the command as stop */
        Winch_Update(w, s->healthy ? commands[i] : 0);

        if (Winch_IsDriving(w))
        {
            if (CS_CheckStall(s, w->stateEnteredMs))
                Winch_Stall(w);          /* stops now; re-arms after cmd 0 */
        }
        else
        {
            CS_ClearStall(s);
        }
    }
}

void WinchSys_StopAll(void)
{
    for (uint8_t i = 0; i < WINCH_COUNT; i++)
        Winch_Stop(&winches[i]);
}

uint8_t WinchSys_Count(void) { return WINCH_COUNT; }

WinchState WinchSys_GetState(uint8_t index)
{
    return (index < WINCH_COUNT) ? winches[index].state : WINCH_FAULT;
}

int32_t WinchSys_GetCurrentmA(uint8_t index)
{
    return (index < WINCH_COUNT) ? sensors[index].currentmA : 0;
}

uint32_t WinchSys_GetBusmV(uint8_t index)
{
    return (index < WINCH_COUNT) ? sensors[index].busmV : 0;
}

