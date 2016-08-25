#include "bluetooth.h"

uint8_t bluetooth_rx_byte;
uint8_t bluetooth_tx_buffer[16];
uint8_t bluetooth_rx_buffer[16];
bluetooth_frame_t bluetooth_tx_frame,bluetooth_rx_frame;
p_bluetooth_frame_t p_bluetooth_tx_frame,p_bluetooth_rx_frame;

void BLUETOOTH_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOD,&GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(UART5,&USART_InitStructure);
	
	USART_ITConfig(UART5,USART_IT_RXNE,ENABLE);
	
	USART_Cmd(UART5,ENABLE);

	USART_GetFlagStatus(UART5,USART_FLAG_TC);

	bluetooth_tx_frame.start = start_of_frame;
	bluetooth_tx_frame.end = end_of_frame;
	bluetooth_rx_frame = bluetooth_tx_frame;
	p_bluetooth_tx_frame = &bluetooth_tx_frame;
	p_bluetooth_rx_frame = &bluetooth_rx_frame;
}

int fputc(int ch,FILE *f)
{
	//USART_GetFlagStatus(UART5,USART_FLAG_TC);
	USART_SendData(UART5,(unsigned char)ch);
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC) == RESET);
	return(ch);
}

void bluetooth_send_number(uint16_t x)
{
	if(x < 0)
	{
		USART_SendData(UART5,'-');
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC) == RESET);
		x = -x;
	}
	
	USART_SendData(UART5,x / 10000 + 48);
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC) == RESET);
	
	USART_SendData(UART5,x % 10000 / 1000 + 48);
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC) == RESET);
	
	USART_SendData(UART5,x % 1000 / 100 + 48);
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC) == RESET);
	
	USART_SendData(UART5,x % 100 / 10 + 48);
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC) == RESET);
	
	USART_SendData(UART5,x % 10 + 48);
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC) == RESET);
	
	USART_SendData(UART5,'\r');
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC) == RESET);
	
	USART_SendData(UART5,'\n');
	while(USART_GetFlagStatus(UART5,USART_FLAG_TC) == RESET);
}

void bluetooth_send_buffer(void)
{
	uint8_t i;
	for(i = 0;i < 16;i++)
	{
		USART_SendData(UART5,bluetooth_tx_buffer[i]);
		while(USART_GetFlagStatus(UART5,USART_FLAG_TC) == RESET);
	}
}

int8_t bluetooth_receive_buffer(void)
{
	static uint8_t state;
	uint8_t i,error = 0;
	
	if(0x7F == bluetooth_rx_byte && 0 == state)
	{
		bluetooth_rx_buffer[0] = bluetooth_rx_byte;
	}
	else if(0 == state)
		error = 1;
	
	if(0xFE == bluetooth_rx_byte && 1 == state)
	{
		bluetooth_rx_buffer[1] = bluetooth_rx_byte;
	}
	else if(1 == state)
		error = 1;
	
	for(i = 2;i < 14;i++)
	{
		if(i == state)
		{
			bluetooth_rx_buffer[i] = bluetooth_rx_byte;
		}
	}
	
	if(0x7F == bluetooth_rx_byte && 14 == state)
	{
		bluetooth_rx_buffer[14] = bluetooth_rx_byte;
	}
	else if(14 == state)
		error = 1;
	
	if(0xFF == bluetooth_rx_byte && 15 == state)
	{
		bluetooth_rx_buffer[15] = bluetooth_rx_byte;
	}
	else if(15 == state)
		error = 1;
	
	state++;
	if(state >= 16 && 0 == error)
	{
		state = 0;
		return 0;
	}
	if(error)
	{
		state = 0;
		for(i = 0;i < 16;i++)
		{
			bluetooth_rx_buffer[i] = 0;
		}
		return -1;
	}
	return 1;
}

void bluetooth_send_frame(void)
{
	bluetooth_tx_buffer[0] = bluetooth_tx_frame.start >> 8;
	bluetooth_tx_buffer[1] = bluetooth_tx_frame.start & 0x00FF;
	bluetooth_tx_buffer[2] = bluetooth_tx_frame.pwm_l >> 8;
	bluetooth_tx_buffer[3] = bluetooth_tx_frame.pwm_l & 0x00FF;
	bluetooth_tx_buffer[4] = bluetooth_tx_frame.pwm_r >> 8;
	bluetooth_tx_buffer[5] = bluetooth_tx_frame.pwm_r & 0x00FF;
	bluetooth_tx_buffer[6] = bluetooth_tx_frame.encoder_l >> 8;
	bluetooth_tx_buffer[7] = bluetooth_tx_frame.encoder_l & 0x00FF;
	bluetooth_tx_buffer[8] = bluetooth_tx_frame.encoder_r >> 8;
	bluetooth_tx_buffer[9] = bluetooth_tx_frame.encoder_r & 0x00FF;
	bluetooth_tx_buffer[10] = bluetooth_tx_frame.speed_l >> 8;
	bluetooth_tx_buffer[11] = bluetooth_tx_frame.speed_l & 0x00FF;
	bluetooth_tx_buffer[12] = bluetooth_tx_frame.speed_r >> 8;
	bluetooth_tx_buffer[13] = bluetooth_tx_frame.speed_r & 0x00FF;
	bluetooth_tx_buffer[14] = bluetooth_tx_frame.end >> 8;
	bluetooth_tx_buffer[15] = bluetooth_tx_frame.end & 0x00FF;

	bluetooth_send_buffer();
}

void bluetooth_receive_frame(void)
{
	if(0 == bluetooth_receive_buffer())
	{
		bluetooth_rx_frame.start = (int16_t)bluetooth_rx_buffer[0] << 8 | (int16_t)bluetooth_rx_buffer[1];
		bluetooth_rx_frame.pwm_l = (int16_t)bluetooth_rx_buffer[2] << 8 | (int16_t)bluetooth_rx_buffer[3];
		bluetooth_rx_frame.pwm_r = (int16_t)bluetooth_rx_buffer[4] << 8 | (int16_t)bluetooth_rx_buffer[5];
		bluetooth_rx_frame.encoder_l = (int16_t)bluetooth_rx_buffer[6] << 8 | (int16_t)bluetooth_rx_buffer[7];
		bluetooth_rx_frame.encoder_r = (int16_t)bluetooth_rx_buffer[8] << 8 | (int16_t)bluetooth_rx_buffer[9];
		bluetooth_rx_frame.speed_l = (int16_t)bluetooth_rx_buffer[10] << 8 | (int16_t)bluetooth_rx_buffer[11];
		bluetooth_rx_frame.speed_r = (int16_t)bluetooth_rx_buffer[12] << 8 | (int16_t)bluetooth_rx_buffer[13];
		bluetooth_rx_frame.end = (int16_t)bluetooth_rx_buffer[14] << 8 | (int16_t)bluetooth_rx_buffer[15];
	}
}

