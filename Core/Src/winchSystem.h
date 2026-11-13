/*
 * winchSystem.h
 *
 *  Created on: Nov 3, 2026
 *      Author: jdalt
 */

#ifndef SRC_WINCHSYSTEM_H_
#define SRC_WINCHSYSTEM_H_

/*
Owns every winch and its current sensor, so main.c only needs:
 *
 *    USER CODE BEGIN 2   WinchSys_Init(&hi2c1);
 *    while (1)           WinchSys_SetCommand(0, winch1Cmd);
 *                        WinchSys_SetCommand(1, winch2Cmd);
 *                        WinchSys_Update();
 *    Error_Handler       WinchSys_StopAll();
 *
 *  Pins, sensor addresses and stall limits live in winchSystem.c.
 */

#include "main.h"
#include "winchDriver.h"
#include "currentSensorDriver.h"


/* Call once after MX_GPIO_Init() and MX_I2C1_Init(). */
void WinchSys_Init(I2C_HandleTypeDef *hi2c);

/* cmd: 0 = stop, 1 = down, 2 = up (same as the Pi packet). Bad index is ignored. */
void WinchSys_SetCommand(uint8_t index, uint8_t cmd);

/* Runs every winch state machine and stall check. Call every loop. */
void WinchSys_Update(void);

/* All bridge outputs off. GPIO only, safe from Error_Handler. */
void WinchSys_StopAll(void);

/* For telemetry back to the Pi */
uint8_t    WinchSys_Count(void);
WinchState WinchSys_GetState(uint8_t index);
int32_t    WinchSys_GetCurrentmA(uint8_t index);
uint32_t   WinchSys_GetBusmV(uint8_t index);



#endif /* SRC_WINCHSYSTEM_H_ */
