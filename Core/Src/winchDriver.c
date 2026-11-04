/*
 * winchDriver.c
 *
 *  Created on: Oct 16, 2026
 *      Author: jdalt
 */
#include "winchDriver.h"

#include "main.h"

/*
 * SETUP
 * requires 8 outpins configured with userlabels
 * {Right_High,Right_Low,Left_High,Left_Low}
 * {W2_Right_High,W2_Right_Low, W2_Left_High, W2_Left_Low}
 *
 * The winches[] table, Winch_Init/Winch_Update calls and current-sensor
 * stall checks all live in winchSystem.c. main.c only calls WinchSys_*.
 */

/*
 * WINCH_START - init position, waits for a low value for on/off before defaulting to WINCH_IDLE
 * WINCH_IDLE - not driven, waiting for a commmand
 * WINCH_UP - up pattern on, if in WINCH_DOWN before starts 500 ms cooldown for mechanical/electrical stress reduction
 * WINCH_DOWN - down pattern on, if in WINCH_DOWN before starts 500 ms cooldown for mechanical/electrical stress reduction
 * WINCH_COOLDOWN - prementioned cooldown state, utizilzes HAL_tick to wait 500ms without halting code.
 * WINCH_STALLED - current sensor saw a stall; stopped until the command goes back to 0, then cooldown
 * WINCH_FAULT - emergency stop until program reset
 */

static void write_pin(WinchPin p, int on)
{
    HAL_GPIO_WritePin(p.port, p.pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void enter(Winch *w, WinchState s)
{
    w->state = s;
    w->stateEnteredMs = HAL_GetTick();
}

static void send_signal(Winch *w, int rh, int rl, int lh, int ll)
{
    if ((rh && rl) || (lh && ll)) {
        /* Shoot through protection */
        Winch_Stop(w);
        enter(w, WINCH_FAULT);
        return;
    }

    write_pin(w->rh, rh);
    write_pin(w->lh, lh);
    write_pin(w->rl, rl);
    write_pin(w->ll, ll);
}

/* No PWM; full-on H-bridge drive.
 *  RH-        -LH
 *        ||
 *        ||
 *  RL-        -LL
 */
static void drive(Winch *w, int up)
{
    if (up) send_signal(w, 1, 0, 0, 1);
    else    send_signal(w, 0, 1, 1, 0);
}



void Winch_Stop(Winch *w)
{
    write_pin(w->rh, 0);
    write_pin(w->lh, 0);
    write_pin(w->rl, 0);
    write_pin(w->ll, 0);
}

void Winch_Init(Winch *w)
{
    Winch_Stop(w);
    enter(w, WINCH_START);
}

void Winch_Stall(Winch *w)
{
    Winch_Stop(w);
    if (w->state != WINCH_FAULT)        /* never downgrade a fault */
        enter(w, WINCH_STALLED);
}

void Winch_Fault(Winch *w)
{
    Winch_Stop(w);
    enter(w, WINCH_FAULT);
}

int Winch_IsDriving(const Winch *w)
{
    return (w->state == WINCH_UP) || (w->state == WINCH_DOWN);
}

void Winch_Update(Winch *w, int cmd)
{
	int runReq, dir;
	if(cmd == 1 || cmd == 2){runReq = 1, dir = (cmd==2);}else{runReq=0;dir=0;}

    switch (w->state) {

    case WINCH_START:
        if (!runReq) enter(w, WINCH_IDLE);
        break;

    case WINCH_IDLE:
        if (runReq) {
            drive(w, dir);
            if (w->state == WINCH_IDLE)
                enter(w, dir ? WINCH_UP : WINCH_DOWN);
        }
        break;

    case WINCH_UP:
        if (!runReq || !dir) {
            Winch_Stop(w);
            enter(w, WINCH_COOLDOWN);
        }
        break;

    case WINCH_DOWN:
        if (!runReq || dir) {
            Winch_Stop(w);
            enter(w, WINCH_COOLDOWN);
        }
        break;

    case WINCH_COOLDOWN:
        if (HAL_GetTick() - w->stateEnteredMs >= WINCH_COOLDOWN_MS)
            enter(w, WINCH_IDLE);
        break;

    case WINCH_STALLED:
        Winch_Stop(w);
        if (!runReq) enter(w, WINCH_COOLDOWN);   /* operator released: re-arm */
        break;

    case WINCH_FAULT:
    default:
        Winch_Stop(w);
        break;
    }
}
