/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "uart_device.h"
#include "modbus.h"
#include "errno.h"
#include "modbus-rtu.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define USE_TMP_HUMI_SENSOR  0
#define USE_ENV_MONITOR_SENSOR  0
#define USE_SWITCH_SENSOR       1

#if USE_TMP_HUMI_SENSOR
extern ADC_HandleTypeDef hadc;
static void aht20_get_datas(uint16_t *h, uint16_t *t);
#define SLAVE_ADDR          3
#define NB_BITS             5
#define NB_INPUT_BITS       0
#define NB_REGISTERS        0
#define NB_INPUT_REGISTERS  2
#endif


#if USE_ENV_MONITOR_SENSOR
extern ADC_HandleTypeDef hadc;
#define SLAVE_ADDR          2
#define NB_BITS             5
#define NB_INPUT_BITS       0
#define NB_REGISTERS        0
#define NB_INPUT_REGISTERS  2
#endif

#if USE_SWITCH_SENSOR
#define SLAVE_ADDR          1
#define NB_BITS             5
#define NB_INPUT_BITS       3
#define NB_REGISTERS        0
#define NB_INPUT_REGISTERS  0
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
#if USE_TMP_HUMI_SENSOR
void ATH20Task(void *argument);
#endif
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
#if USE_TMP_HUMI_SENSOR
  osThreadNew(ATH20Task, NULL, &defaultTask_attributes);
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
#if USE_TMP_HUMI_SENSOR
//**********************************************************//
//CRC校验类型： CRC8
//多项式： X8+X5+X4+1
//Poly:0011 0001 0x31
unsigned char Calc_CRC8(unsigned char *message,unsigned char Num)
{
    unsigned char i;
    unsigned char byte;
    unsigned char crc =0xFF;
    for (byte = 0;byte<Num;byte++)
    {
        crc^=(message[byte]);
        for(i=8;i>0;--i)
        {
            if(crc&0x80)
                crc=(crc<<1)^0x31;
            else
                crc=(crc<<1);
        }
    }
    return crc;
}//**********************************************************//
static uint32_t g_humi, g_tmp;

static void aht20_get_datas(uint16_t *h, uint16_t *t)
{
    *h = g_humi;
    *t = g_tmp;
}

void ATH20Task(void *argument)
{
    uint8_t cmd[] = {0xac, 0x33, 0x00};
    uint8_t datas[7] = {0};
    unsigned char crc;
    extern I2C_HandleTypeDef hi2c1;
    vTaskDelay(10);              //上电后等待5ms
    while(1)
    {
        if(HAL_OK == HAL_I2C_Master_Transmit(&hi2c1, 0x70, cmd, 3, 100))
        {
            vTaskDelay(100);    //等待80ms以上
            if(HAL_OK == HAL_I2C_Master_Receive(&hi2c1, 0x71, datas, 7, 100))
            {
                //计算CRC
                crc = Calc_CRC8(datas, 6);
                if (crc == datas[6])
                {
                    /*ok*/
                    g_humi = ((uint32_t)datas[1] << 12) | ((uint32_t)datas[2] << 4) | ((uint32_t)datas[3] >> 4);
                    g_tmp = (((uint32_t)datas[3]&0x0f)  << 16) | ((uint32_t)datas[4] << 8) | ((uint32_t)datas[5]);
                    g_humi = (g_humi * 100 / 0x100000) * 10;    // 0.1%
                    g_tmp = (g_tmp * 200 / 0x100000 - 50) * 10; //0.1C  
                }
            }
        }
        vTaskDelay(20);
    }
}
#endif
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
    
    GPIO_PinState val;
    
    uint8_t *query;
	modbus_t *ctx;
	int rc;
	modbus_mapping_t *mb_mapping;
	
#if USE_ENV_MONITOR_SENSOR
    HAL_ADCEx_Calibration_Start(&hadc);
#endif
	ctx = modbus_new_st_rtu("uart1", 115200, 'N', 8, 1);
	modbus_set_slave(ctx, SLAVE_ADDR);
	query = pvPortMalloc(MODBUS_RTU_MAX_ADU_LENGTH);
    
	mb_mapping = modbus_mapping_new_start_address(0, NB_BITS, 0, NB_INPUT_BITS, 0, NB_REGISTERS, 0, NB_INPUT_REGISTERS);
	
	memset(mb_mapping->tab_bits, 0, mb_mapping->nb_bits);
	memset(mb_mapping->tab_registers, 0x55, mb_mapping->nb_registers*2);

	rc = modbus_connect(ctx);
	if (rc == -1) {
		//fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
		modbus_free(ctx);
		vTaskDelete(NULL);;
	}

	for (;;) {
		do {
			rc = modbus_receive(ctx, query);
			/* Filtered queries return 0 */
		} while (rc == 0);
 
		/* The connection is not closed on errors which require on reply such as
		   bad CRC in RTU. */
		if (rc == -1 && errno != EMBBADCRC) {
			/* Quit */
			continue;
		}

        /* updata value of registers
         * a.read gpio
         * b.updata registers
         */
#if USE_SWITCH_SENSOR
        /*key 1*/
        val = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3); 
        if(val ==  GPIO_PIN_RESET)
            mb_mapping->tab_input_bits[0] = 1;
        else
            mb_mapping->tab_input_bits[0] = 0;
        /*key 2*/
        val = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4); 
        if(val ==  GPIO_PIN_RESET)
            mb_mapping->tab_input_bits[1] = 1;
        else
            mb_mapping->tab_input_bits[1] = 0;
        /*key 3*/
        val = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5); 
        if(val ==  GPIO_PIN_RESET)
            mb_mapping->tab_input_bits[2] = 1;
        else
            mb_mapping->tab_input_bits[2] = 0;
#endif
#if USE_ENV_MONITOR_SENSOR
        /* read ADC to updata tab_input_registers */ 
        for (int i = 0; i < 2; i++)
        {
            HAL_ADC_Start(&hadc);
            if (HAL_OK == HAL_ADC_PollForConversion(&hadc, 100))
            {
                mb_mapping->tab_input_registers[i] = HAL_ADC_GetValue(&hadc);
            }
        }
#endif
#if USE_TMP_HUMI_SENSOR
        /* can not start iic to capture, it takes up  80ms，too long  */ 
        uint16_t tmp, humi;
        aht20_get_datas(&tmp, &humi);
        mb_mapping->tab_input_registers[0] = tmp;
        mb_mapping->tab_input_registers[1] = humi;
#endif
		rc = modbus_reply(ctx, query, rc, mb_mapping);
		if (rc == -1) {
			//break;
		}
#if USE_SWITCH_SENSOR
        /* switch 1 */
		if (mb_mapping->tab_bits[0])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); 
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET); 
        /* switch 2 */
		if (mb_mapping->tab_bits[1])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET); 
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET); 
        /* led 1*/
        if (mb_mapping->tab_bits[2])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); 
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); 
        /* led 2*/
        if (mb_mapping->tab_bits[3])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); 
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); 
        /* led 3*/
        if (mb_mapping->tab_bits[4])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); 
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); 
#endif
#if USE_ENV_MONITOR_SENSOR
        /* beep 1 */
		if (mb_mapping->tab_bits[0])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); 
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); 
        /* beep 2 */
		if (mb_mapping->tab_bits[1])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); 
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); 
        /* led 1*/
        if (mb_mapping->tab_bits[2])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); 
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); 
        /* led 2*/
        if (mb_mapping->tab_bits[3])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); 
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); 
        /* led 3*/
        if (mb_mapping->tab_bits[4])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); 
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); 
#endif
#if USE_TMP_HUMI_SENSOR
        /* can not start iic to capture, it takes up  80ms，too long  */ 
        /* beep 1 */
		if (mb_mapping->tab_bits[0])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); 
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); 
        /* beep 2 */
		if (mb_mapping->tab_bits[1])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); 
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); 
        /* led 1*/
        if (mb_mapping->tab_bits[2])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET); 
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET); 
        /* led 2*/
        if (mb_mapping->tab_bits[3])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); 
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); 
        /* led 3*/
        if (mb_mapping->tab_bits[4])
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); 
        else
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); 
#endif
	}

  modbus_mapping_free(mb_mapping);
	vPortFree(query);
	/* For RTU */
	modbus_close(ctx);
	modbus_free(ctx);

	vTaskDelete(NULL);
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

