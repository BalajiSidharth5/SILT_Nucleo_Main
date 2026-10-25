/*
 * mylib.h
 *
 *  Created on: Oct 16, 2026
 *      Author: jdalt
 */

#ifndef SRC_WINCHDRIVER_H_
#define SRC_WINCHDRIVER_H_

typedef enum { WINCH_IDLE, WINCH_UP, WINCH_DOWN, WINCH_COOLDOWN, WINCH_FAULT, WINCH_START } WinchState;

void Send_Winch_Signal(int, int, int, int);
void Winch_Toggle(int);
void Winch_Stop();

void Winch_Update(int, int);
void enter(WinchState);


#endif /* SRC_WINCHDRIVER_H_ */
