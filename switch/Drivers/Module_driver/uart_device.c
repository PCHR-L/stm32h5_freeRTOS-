// SPDX-License-Identifier: GPL-3.0-only
/*
 * Copyright (c) 2008-2023 100askTeam : Dongshan WEI <weidongshan@qq.com> 
 * Discourse:  https://forums.100ask.net
 */
 
/*  Copyright (C) 2008-2023 深圳百问网科技有限公司
 *  All rights reserved
 *
 * 免责声明: 百问网编写的文档, 仅供学员学习使用, 可以转发或引用(请保留作者信息),禁止用于商业用途！
 * 免责声明: 百问网编写的程序, 可以用于商业用途, 但百问网不承担任何后果！
 * 
 * 本程序遵循GPL V3协议, 请遵循协议
 * 百问网学习平台   : https://www.100ask.net
 * 百问网交流社区   : https://forums.100ask.net
 * 百问网官方B站    : https://space.bilibili.com/275908810
 * 本程序所用开发板 : DShanMCU-F103
 * 百问网官方淘宝   : https://100ask.taobao.com
 * 联系我们(E-mail): weidongshan@qq.com
 *
 *          版权所有，盗版必究。
 *  
 * 修改历史     版本号           作者        修改内容
 *-----------------------------------------------------
 * 2024.02.01      v01         百问科技      创建文件
 *-----------------------------------------------------
 */
#include <stdio.h>
#include <string.h>

#include "uart_device.h"

static struct UART_Data g_uart1_data = {
	&huart1,
	GPIOA,
	GPIO_PIN_8,
};

static int stm32_uart_init(struct UART_Device *pDev, int baud, char parity, int data_bit, int stop_bit);
static int stm32_uart_recv(struct UART_Device *pDev, uint8_t *pData, int timeout);
static int stm32_uart_send(struct UART_Device *pDev, uint8_t *datas, uint32_t len, int timeout);
static int stm32_uart_flush(struct UART_Device *pDev);

struct UART_Device g_uart1_dev = {"uart1", stm32_uart_init, stm32_uart_send, stm32_uart_recv, stm32_uart_flush, &g_uart1_data};

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{   
	if (huart == &huart1)
	{
		struct UART_Data *uart_data = g_uart1_dev.priv_data;
		xSemaphoreGiveFromISR(uart_data->xTxSem, NULL);
		HAL_GPIO_WritePin(uart_data->GPIOx_485, uart_data->GPIO_Pin_485, GPIO_PIN_RESET);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart1)
	{
		struct UART_Data *uart_data = g_uart1_dev.priv_data;
		/* write queue : g_uart4_rx_buf 100 bytes ==> queue */
		xQueueSendFromISR(uart_data->xRxqueue, (const void *)&uart_data->rxdata, NULL);
		HAL_UART_Receive_IT(uart_data->huart, &uart_data->rxdata, 1);
	}	
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
	{
		struct UART_Data *uart_data = g_uart1_dev.priv_data;
        HAL_UART_DeInit(uart_data->huart);
        HAL_UART_Init(uart_data->huart);
		HAL_UART_Receive_IT(uart_data->huart, &uart_data->rxdata, 1);
	}	
}

static int stm32_uart_init(struct UART_Device *pDev, int baud, char parity, int data_bit, int stop_bit)
{
	struct UART_Data *uart_data = pDev->priv_data;
	if (!uart_data->xRxqueue)
	{
		uart_data->xRxqueue = xQueueCreate(200, 1);
		uart_data->xTxSem = xSemaphoreCreateBinary( );
		/* 配置RS485转换芯片的方向引脚，设为0表示接收 */
		HAL_GPIO_WritePin(uart_data->GPIOx_485, uart_data->GPIO_Pin_485, GPIO_PIN_RESET);
		HAL_UART_Receive_IT(uart_data->huart, &uart_data->rxdata, 1);
	}
	return 0;
}

static int stm32_uart_recv(struct UART_Device *pDev, uint8_t *pData, int timeout)
{
	struct UART_Data *uart_data = pDev->priv_data;
	if (pdPASS == xQueueReceive(uart_data->xRxqueue, pData, timeout))
		return 0;
	else
		return -1;
}

static int stm32_uart_send(struct UART_Device *pDev, uint8_t *datas, uint32_t len, int timeout)
{
	struct UART_Data *uart_data = pDev->priv_data;
	/* 配置RS485转换芯片的方向引脚，设为1表示发送 */
	HAL_GPIO_WritePin(uart_data->GPIOx_485, uart_data->GPIO_Pin_485, GPIO_PIN_SET);
	HAL_UART_Transmit_IT(uart_data->huart, datas, len);
	if (pdTRUE == xSemaphoreTake(uart_data->xTxSem, timeout))
	{	
			/* 配置RS485转换芯片的方向引脚，设为0表示接收 */
		HAL_GPIO_WritePin(uart_data->GPIOx_485, uart_data->GPIO_Pin_485, GPIO_PIN_RESET);
		return 0;
	}
	else
		return -1;
}

static int stm32_uart_flush(struct UART_Device *pDev)
{
	int cnt = 0;
	uint8_t data;
	struct UART_Data *uart_data = pDev->priv_data;
	while (1)
	{
		if (pdPASS != xQueueReceive(uart_data->xRxqueue, &data, 0))
			break;
		cnt++;
	}
	return cnt;
}

static struct UART_Device *g_uart_devices[] = {&g_uart1_dev};

struct UART_Device *GetUARTDevice(char *name)
{
	int i = 0;
	for (i = 0; i < sizeof(g_uart_devices)/sizeof(g_uart_devices[0]); i++)
	{
		if (!strcmp(name, g_uart_devices[i]->name))
			return g_uart_devices[i];
	}
	
	return NULL;
}
