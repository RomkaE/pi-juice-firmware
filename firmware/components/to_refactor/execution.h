/*
 * execution.h
 *
 *  Created on: 26.03.2018.
 *      Author: milan
 */

#ifndef EXECUTION_H_
#define EXECUTION_H_


#define EXECUTION_STATE_COLD_START		0
#define EXECUTION_STATE_POR_RESET		0x11111111
#define EXECUTION_STATE_NORMAL			0xAAAAAAAA
//#define EXECUTION_STATE_UPDATE			0xA1A15151//
#define EXECUTION_STATE_CONFIG_RESET	0xA5A5A5A5

extern uint32_t executionState;

#endif /* EXECUTION_H_ */
