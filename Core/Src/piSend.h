/*
 * piSend.h
 *
 *  Created on: Nov 12, 2026
 *      Author: jdalt
 */

#ifndef SRC_PISEND_H_
#define SRC_PISEND_H_

/*
 * Sends winch telemetry to the Pi, so main.c only needs:
 *
 *    USER CODE BEGIN 2   PiSend_Init(&PI_UART);
 *    while (1)           PiSend_Update();
 *
 * Line sent every PISEND_INTERVAL_MS:
 *
 *    $I,w1mA,w1mV,w2mA,w2mV,s1,s2*CS\n      e.g.  $I,350,12480,0,12470,2,1*43\n
 *
 *    w#mA : winch current in mA (signed, can be negative)
 *    w#mV : winch bus voltage in mV
 *    s#   : WinchState number (0 START, 1 IDLE, 2 UP, 3 DOWN,
 *                              4 COOLDOWN, 5 STALLED, 6 FAULT)
 *    CS   : XOR of every char between '$' and '*', as 2 hex digits
 *           (same checksum as the packets coming from the Pi)
 *
 * Values come from WinchSys_Get*(), so WinchSys_Update() must run in the
 * same loop to keep them fresh.
 */

#include "main.h"

#define PISEND_INTERVAL_MS   100u   /* 100 ms between sends = 10 per second  */
#define PISEND_TIMEOUT_MS    10u    /* ~30 bytes at 115200 takes ~3 ms        */
#define PISEND_MAX_LEN       64u    /* buffer size, including '\n' and '\0'  */

/* Call once after MX_USARTx_UART_Init() and WinchSys_Init(). */
void PiSend_Init(UART_HandleTypeDef *huart);

/* Builds and sends one line if PISEND_INTERVAL_MS has passed. Call every loop. */
void PiSend_Update(void);

/* Fills buf with one complete line (with checksum and '\n').
 * Returns the length, or 0 if it didn't fit. Used by PiSend_Update,
 * public so it can be printed/tested on its own. */
uint16_t PiSend_BuildLine(char *buf, uint16_t size);

/* XOR of the chars after '$' up to '*' (or end of string). */
uint8_t PiSend_Checksum(const char *line);


#endif /* SRC_PISEND_H_ */
