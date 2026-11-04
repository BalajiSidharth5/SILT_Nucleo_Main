/*
 * winchDriver.h
 *
 *  Created on: Oct 16, 2026
 *      Author: jdalt
 * Instructions in winchDriver.c
 */

#ifndef SRC_WINCHDRIVER_H_
#define SRC_WINCHDRIVER_H_

#include "main.h"

#define WINCH_COOLDOWN_MS 500u

typedef enum {
    WINCH_START = 0,
    WINCH_IDLE,
    WINCH_UP,
    WINCH_DOWN,
    WINCH_COOLDOWN,
    WINCH_STALLED,
    WINCH_FAULT
} WinchState;

typedef struct {
    GPIO_TypeDef *port;
    uint16_t      pin;
} WinchPin;

typedef struct {
    WinchPin rh;
    WinchPin rl;
    WinchPin lh;
    WinchPin ll;

    WinchState state;
    uint32_t   stateEnteredMs;
} Winch;

void Winch_Init(Winch *w);

/* cmd: 0 = stop, 1 = down, 2 = up */
void Winch_Update(Winch *w, int cmd);

/* Turns all four bridge outputs off. Safe to call from Error_Handler. Needs to be called individually for each winch*/
void Winch_Stop(Winch *w);

/* Stall seen by the current sensor: stop now, run again only after cmd 0. */
void Winch_Stall(Winch *w);

/* Stop and stay stopped until reset (current sensor missing or accidental shoo thro). */
void Winch_Fault(Winch *w);

int Winch_IsDriving(const Winch *w);

#endif /* SRC_WINCHDRIVER_H_ */
