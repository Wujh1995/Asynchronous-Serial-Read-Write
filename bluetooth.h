#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

//#include "stm32f10x.h"
#include <stdio.h>
#include <stdint.h>

#define start_of_frame 0x7FFE
#define end_of_frame 0x7FFF

#define __PACKED__     __attribute__ ((__packed__))

struct bluetooth_frame //16 Bytes
{
	int16_t start;
	int16_t pwm_l;
	int16_t pwm_r;
	int16_t encoder_l;
	int16_t encoder_r;
	int16_t  speed_l;
	int16_t speed_r;
	int16_t end;
}__PACKED__;
typedef struct bluetooth_frame bluetooth_frame_t;
typedef struct bluetooth_frame* p_bluetooth_frame_t;

/*
void BLUETOOTH_Config(void);
void bluetooth_send_number(uint16_t x);
void bluetooth_send_buffer(void);
int8_t bluetooth_receive_buffer(void);
void bluetooth_send_frame(void);
void bluetooth_receive_frame(void);
*/
#endif

