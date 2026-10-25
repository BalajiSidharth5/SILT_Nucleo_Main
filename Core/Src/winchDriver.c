/*
 * mylib.c
 *
 *  Created on: Oct 16, 2026
 *      Author: jdalt
 */
#include "winchDriver.h"

#include "main.h"


/*
 * WINCH_IDLE - not driven, waiting for a commmand
 * WINCH_UP - up pattern on, if in WINCH_DOWN before starts 500 ms cooldown for mechanical/electrical stress reduction
 * WINCH_DOWN - down pattern on, if in WINCH_DOWN before starts 500 ms cooldown for mechanical/electrical stress reduction
 * WINCH_COOLDOWN - prementioned cooldown state, utizilzes HAL_tick to wait 500ms without halting code.
 * WINCH_FAULT - emergency stop until program reset
 */

WinchState state = WINCH_START;
uint32_t stateEnteredMs;

void enter(WinchState s) {
    state = s;
    stateEnteredMs = HAL_GetTick();
}

void Winch_Stop(){
	HAL_GPIO_WritePin(Right_High_GPIO_Port, Right_High_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Left_High_GPIO_Port, Left_High_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Right_Low_GPIO_Port, Right_Low_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(Left_Low_GPIO_Port, Left_Low_Pin, GPIO_PIN_RESET);
}
void Send_Winch_Signal(int RH, int RL, int LH, int LL){
	if((RH && RL) || (LH && LL)){
		//Error: output stopped due to potential short circuit (RH && RL || LH && LL
		//turn all off to eliminate sho throug
		Winch_Stop();
		enter(WINCH_FAULT);
		return;
	}

	HAL_GPIO_WritePin(Right_High_GPIO_Port, Right_High_Pin, RH);
	HAL_GPIO_WritePin(Left_High_GPIO_Port, Left_High_Pin, LH);
	HAL_GPIO_WritePin(Right_Low_GPIO_Port, Right_Low_Pin, RL);
	HAL_GPIO_WritePin(Left_Low_GPIO_Port, Left_Low_Pin, LL);

}
//no PWM control and controls h bridge logic
void Winch_Toggle(int direction){
	// 1 = up, 0 = down
	//  RH-        -LH
	//		  ||
	//	      ||
	//  RL-        -LL
	if(direction){
		Send_Winch_Signal(1,0,0,1);
	}
	else{
		Send_Winch_Signal(0,1,1,0);
	}

}

void Winch_Update(int runReq, int dir){
	switch(state){
	case WINCH_IDLE:
		if(runReq){
			Winch_Toggle(dir);
			if(state == WINCH_IDLE) enter(dir ? WINCH_UP : WINCH_DOWN);
		}
		break;
	case WINCH_UP:
		if(!runReq || !dir){
			Winch_Stop();
			enter(WINCH_COOLDOWN);
		}
		break;
	case WINCH_DOWN:
		if(!runReq || dir){
			Winch_Stop();
			enter(WINCH_COOLDOWN);
		}
		break;
	case WINCH_COOLDOWN:
		if(HAL_GetTick() - stateEnteredMs >= 500){
			enter(WINCH_IDLE);
		}
		break;
	case WINCH_FAULT:
		Winch_Stop();
		break;
	//ensures winch doesn't run on start and waits until a low input is red from runReq
	case WINCH_START:
	       if (!runReq) enter(WINCH_IDLE);
	       break;
	}
}

