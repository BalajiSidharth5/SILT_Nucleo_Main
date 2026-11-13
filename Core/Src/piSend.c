/*
 * piSend.c
 *
 *  Created on: Nov 12, 2026
 *      Author: jdalt
 */


#include "piSend.h"
#include "winchSystem.h"

#include <stdio.h>

static UART_HandleTypeDef *uart = NULL;
static uint32_t lastSendTick = 0;

void PiSend_Init(UART_HandleTypeDef *huart)
{
    uart = huart;
    lastSendTick = HAL_GetTick();
}

uint8_t PiSend_Checksum(const char *line)
{
    uint8_t total = 0;
    uint16_t i = 1;

    while (line[i] != '*' && line[i] != '\0')
    {
        total ^= (uint8_t)line[i];
        i++;
    }
    return total;
}

uint16_t PiSend_BuildLine(char *buf, uint16_t size)
{
    /* Step 1: data part, "$I,w1mA,w1mV,w2mA,w2mV,s1,s2*" */
	//I assume s1 and s2 are states of motor but need to confirm

    int len = snprintf(buf, size, "$I,%ld,%lu,%ld,%lu,%u,%u*",
                       (long)WinchSys_GetCurrentmA(0), (unsigned long)WinchSys_GetBusmV(0),
                       (long)WinchSys_GetCurrentmA(1), (unsigned long)WinchSys_GetBusmV(1),
                       (unsigned)WinchSys_GetState(0), (unsigned)WinchSys_GetState(1));
    if (len < 0 || len >= size) return 0;

    /* Step 2: tack the checksum and newline onto the end, "3A\n" */
    int tail = snprintf(buf + len, size - len, "%02X\n", PiSend_Checksum(buf));
    if (tail < 0 || len + tail >= size) return 0;

    return (uint16_t)(len + tail);
}

void PiSend_Update(void)
{
    if (uart == NULL) return;
    if (HAL_GetTick() - lastSendTick < PISEND_INTERVAL_MS) return;
    lastSendTick = HAL_GetTick();

    char line[PISEND_MAX_LEN];
    uint16_t len = PiSend_BuildLine(line, sizeof(line));
    if (len == 0) return;

    HAL_UART_Transmit(uart, (uint8_t *)line, len, PISEND_TIMEOUT_MS);
}
