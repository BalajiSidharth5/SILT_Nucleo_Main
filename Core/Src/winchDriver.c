/*
 * winchDriver.c
 *
 *  Created on: Oct 16, 2026
 *      Author: jdalt
 */
#include "winchDriver.h"

#include "main.h"

/*
 * REQUIRED CODE IN main.c
 * requires 8 outpins configured with userlabels
 * {Right_High,Right_Low,Left_High,Left_Low}
 * {W2_Right_High,W2_Right_Low, W2_Left_High, W2_Left_Low}
 *
 * requires winch1Cmd/winch2Cmd from recieve function
 *
 *USER CODE PV
 *static Winch winches[] = {
	//Winch one pins
  { .rh = {Right_High_GPIO_Port, Right_High_Pin}, .rl = {Right_Low_GPIO_Port, Right_Low_Pin},
    .lh = {Left_High_GPIO_Port,  Left_High_Pin},  .ll = {Left_Low_GPIO_Port,  Left_Low_Pin} },
	//Winch two pins (placeholders currently)
  { .rh = {W2_Right_High_GPIO_Port, W2_Right_High_Pin}, .rl = {W2_Right_Low_GPIO_Port, W2_Right_Low_Pin},
    .lh = {W2_Left_High_GPIO_Port,  W2_Left_High_Pin},  .ll = {W2_Left_Low_GPIO_Port,  W2_Left_Low_Pin} },
};
 *
 *USER CODE BEGIN 2
 *Winch_Init(&winches[0]);
  Winch_Init(&winches[1]);
 *
 *while(1)
 *Winch_Update(&winches[0], winch1Cmd);
 *Winch_Update(&winches[0], Winch_Update(&winches[1], winch2Cmd);
 *
 *USER CODE BEGIN ERROR HANDLER
 *Winch_Stop(&winches[0]);
 *Winch_Stop(&winches[1]);
 */

/*
 * WINCH_START - init position, waits for a low value for on/off before defaulting to WINCH_IDLE
 * WINCH_IDLE - not driven, waiting for a commmand
 * WINCH_UP - up pattern on, if in WINCH_DOWN before starts 500 ms cooldown for mechanical/electrical stress reduction
 * WINCH_DOWN - down pattern on, if in WINCH_DOWN before starts 500 ms cooldown for mechanical/electrical stress reduction
 * WINCH_COOLDOWN - prementioned cooldown state, utizilzes HAL_tick to wait 500ms without halting code.
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
        /* Shoo through protection */
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

    case WINCH_FAULT:
    default:
        Winch_Stop(w);
        break;
    }
}



