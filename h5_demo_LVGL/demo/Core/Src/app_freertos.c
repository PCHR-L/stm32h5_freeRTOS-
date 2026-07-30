/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "cmsis_os2.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "draw.h"
#include "stdio.h"
#include "draw.h"
#include "ux_api.h"
#include "modbus.h"
#include "errno.h"
#include "uart_device.h"
#include "semphr.h"
#include "lv_init.h"
#include "lv_port_disp.h"
#include "ui.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;

extern lv_obj_t *led;
extern lv_obj_t *led1;
extern lv_obj_t *led2;

extern lv_obj_t *lab_temp;
extern lv_obj_t *lab_volt;
extern lv_obj_t *env_light;
extern lv_obj_t *env_press;

static SemaphoreHandle_t g_xBinarySemaphoreSwitch;
static SemaphoreHandle_t g_xBinarySemaphoreENV;
static SemaphoreHandle_t g_xBinarySemaphoreTempHumi;
static SemaphoreHandle_t g_ch1_lock;

modbus_mapping_t *g_mb_mapping;

static modbus_t *g_ch1_ctx;


/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

static void LibmodbusServerTask( void *pvParameters )	
{
	uint8_t *query;
	modbus_t *ctx;
	int rc;
	modbus_mapping_t *mb_mapping;
    uint8_t do_registers_backup[16];
	
	ctx = modbus_new_st_rtu("usb", 115200, 'N', 8, 1);
	modbus_set_slave(ctx, 1);
	query = pvPortMalloc(MODBUS_RTU_MAX_ADU_LENGTH);

	mb_mapping = modbus_mapping_new_start_address(0,
												  16,  /* DO, H5 1, switch has 5, env 5, temp&humi 5 */
												  0,
												  3,   /* DI, switch 3 */
												  0,
												  0,   /* AO, 0 */
												  0,
												  4);  /* AI, env 2, temp&humi 2 */
	g_mb_mapping = mb_mapping;
	memset(mb_mapping->tab_bits, 0, mb_mapping->nb_bits);
    memset(do_registers_backup, 0, sizeof(do_registers_backup));
	memset(mb_mapping->tab_registers, 0, mb_mapping->nb_registers*2);    

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
            vTaskDelay(10);
			continue;
		}

		rc = modbus_reply(ctx, query, rc, mb_mapping);
		if (rc == -1) {
			//break;
		}

        /* For DO register, wake up task2/3/4 */
        if (memcmp(&do_registers_backup[1], &mb_mapping->tab_bits[1], 5) != 0)
        {
            /* wakeup task2 */
            xSemaphoreGive(g_xBinarySemaphoreSwitch);
        }
        if (memcmp(&do_registers_backup[6], &mb_mapping->tab_bits[6], 5) != 0)
        {
            /* wakeup task3 */
            xSemaphoreGive(g_xBinarySemaphoreENV);
        }
        if (memcmp(&do_registers_backup[11], &mb_mapping->tab_bits[11], 5) != 0)
        {
            /* wakeup task3 */
            xSemaphoreGive(g_xBinarySemaphoreTempHumi);
        }

        memcpy(do_registers_backup, mb_mapping->tab_bits, 16);

		if (mb_mapping->tab_bits[0])
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET);
	}

	modbus_mapping_free(mb_mapping);
	vPortFree(query);
	/* For RTU */
	modbus_close(ctx);
	modbus_free(ctx);

	vTaskDelete(NULL);
}

static void LibmodbusCH1SwitchClientTask( void *pvParameters )	
{
	modbus_t *ctx;
	int rc;
	uint16_t vals[10];
	int nb = 1;
    uint8_t bits[10] = {0};
    uint8_t buf[100];
    int led_status = 1;

    ctx = g_ch1_ctx;

    xSemaphoreTake(g_ch1_lock, portMAX_DELAY);
    
    modbus_set_slave(ctx, 1);
    rc = modbus_read_bits(ctx, 0, 5, bits);
    if (rc == 5)
    {
        memcpy(&g_mb_mapping->tab_bits[1], bits, 5);
    }
    
    xSemaphoreGive(g_ch1_lock);
    

	for (;;) {

        /* read switch(ID=1), get status of key1,key2,key3 
         * display on lcd
         * blink led
         */
        xSemaphoreTake(g_ch1_lock, portMAX_DELAY);
        vTaskDelay(20);
        modbus_set_slave(ctx, 1);
        rc = modbus_read_input_bits(ctx, 0, 3, bits);
        if (rc == 3)
        {
            if(bits[0] != 0)
            {
                lv_lock();
                lv_led_on(led);
                lv_unlock();
            }else
            {
                lv_lock();
                lv_led_off(led);
                lv_unlock();
            }
            if(bits[1] != 0)
            {
                lv_lock();
                lv_led_on(led1);
                lv_unlock();
            }
            else 
            {
                lv_lock();
                lv_led_off(led1);
                lv_unlock();
            }
            if(bits[2] != 0)
            {
                lv_lock();
                lv_led_on(led2);
                lv_unlock();
            }
            else
            {
                lv_lock();
                lv_led_off(led2);
                lv_unlock();
            }
            /* update DI register */
            memcpy(g_mb_mapping->tab_input_bits, bits, 3);
        }
        xSemaphoreGive(g_ch1_lock);
        
        /* wait for PC update DO */
        if (xSemaphoreTake(g_xBinarySemaphoreSwitch, 500) == pdTRUE)
        {
            xSemaphoreTake(g_ch1_lock, portMAX_DELAY);
            vTaskDelay(20);
            modbus_set_slave(ctx, 1);
            rc = modbus_write_bits(ctx, 0, 5, &g_mb_mapping->tab_bits[1]);            
            xSemaphoreGive(g_ch1_lock);
        }
	}

	/* For RTU */
	modbus_close(ctx);
	modbus_free(ctx);

	vTaskDelete(NULL);
}

static void LibmodbusCH1ENVClientTask( void *pvParameters )	
{
	modbus_t *ctx;
	int rc;
	uint16_t vals[10];
	int nb = 1;
    uint8_t bits[10];
    uint8_t buf[100];
    int led_status = 1;

    ctx = g_ch1_ctx;

    xSemaphoreTake(g_ch1_lock, portMAX_DELAY);
    
    modbus_set_slave(ctx, 2);
    rc = modbus_read_bits(ctx, 0, 5, bits);
    if (rc == 5)
    {
        memcpy(&g_mb_mapping->tab_bits[6], bits, 5);
    }
    
    xSemaphoreGive(g_ch1_lock);

	for (;;) {

        /* read switch(ID=2), get status of key1,key2,key3 
         * display on lcd
         * blink led
         */
        xSemaphoreTake(g_ch1_lock, portMAX_DELAY);
        vTaskDelay(20);
        modbus_set_slave(ctx, 2);
		rc = modbus_read_input_registers(ctx, 0, 2, vals);
		if (rc == 2)
		{
            snprintf((char *)buf, sizeof(buf), "opti:0x%x", vals[0]);
            lv_lock();
            lv_label_set_text(env_light, (char *)buf);
            lv_unlock();
            snprintf((char *)buf, sizeof(buf), "res :0x%x", vals[1]);
            lv_lock();
            lv_label_set_text(env_press, (char *)buf);
            lv_unlock();

            /* update AI register */
            memcpy(g_mb_mapping->tab_input_registers, vals, 4);
		}
        xSemaphoreGive(g_ch1_lock);

        /* wait for PC update DO */
        if (xSemaphoreTake(g_xBinarySemaphoreENV, 500) == pdTRUE)
        {
            xSemaphoreTake(g_ch1_lock, portMAX_DELAY);
            vTaskDelay(20);
            modbus_set_slave(ctx, 2);
            rc = modbus_write_bits(ctx, 0, 5, &g_mb_mapping->tab_bits[6]);            
            xSemaphoreGive(g_ch1_lock);
        }

	}

	/* For RTU */
	modbus_close(ctx);
	modbus_free(ctx);

	vTaskDelete(NULL);
}

static void LibmodbusCH2TempHumiClientTask( void *pvParameters )	
{
	modbus_t *ctx;
	int rc;
	uint16_t vals[10];
	int nb = 1;
    uint8_t bits[10];
    uint8_t buf[100];
    int led_status = 1;
	
	ctx = modbus_new_st_rtu("uart4", 115200, 'N', 8, 1);
	modbus_set_slave(ctx, 3);
	
	rc = modbus_connect(ctx);
	if (rc == -1) {
		//fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
		modbus_free(ctx);
		vTaskDelete(NULL);;
	}

    rc = modbus_read_bits(ctx, 0, 5, bits);
    if (rc == 5)
    {
        memcpy(&g_mb_mapping->tab_bits[11], bits, 5);
    }

	for (;;) {

        /* read tem_hum(ID=3), get temp, humi
         * display on lcd
         * blink led
         */
		rc = modbus_read_input_registers(ctx, 0, 2, vals);
		if (rc == 2)
		{
            snprintf((char *)buf, sizeof(buf), "temp:%d.%dC", vals[1]/10, vals[1]%10);
            lv_lock();
            lv_label_set_text(lab_temp, (char *)buf);
            lv_unlock();
            snprintf((char *)buf, sizeof(buf), "humi:%d.%d", vals[0]/10, vals[0]%10);
            lv_lock();
            lv_label_set_text(lab_volt, (char *)buf);
            lv_unlock();

            /* update AI register */
            memcpy(&g_mb_mapping->tab_input_registers[2], vals, 4);
		}

        /* wait for PC update DO */
        if (xSemaphoreTake(g_xBinarySemaphoreTempHumi, 500) == pdTRUE)
        {
            rc = modbus_write_bits(ctx, 0, 5, &g_mb_mapping->tab_bits[11]);            
        }
	}

	/* For RTU */
	modbus_close(ctx);
	modbus_free(ctx);

	vTaskDelete(NULL);
}


static void lvgl_task(void *pvParameters)
{
    lv_port_disp_init();
    ui_init();
    while(1)
    {
        lv_task_handler();   // LVGL后台处理
        vTaskDelay(10);         // 10ms刷新一次，100Hz刷新率
    }
}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void vApplicationTickHook(void);

/* USER CODE BEGIN 3 */
void vApplicationTickHook( void )
{
   /* This function will be called by each tick interrupt if
   configUSE_TICK_HOOK is set to 1 in FreeRTOSConfig.h. User code can be
   added here, but the tick hook is called from an interrupt context, so
   code must not attempt to block, and only the interrupt safe FreeRTOS API
   functions can be used (those that end in FromISR()). */
    //lv_tick_inc(1);lv_tick_set_cb() 是 LVGL v9 引入的"查询式" Tick 机制，用来替代旧版的 lv_tick_inc() 周期性调用
}
/* USER CODE END 3 */

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
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  g_xBinarySemaphoreSwitch = xSemaphoreCreateBinary( );
  g_xBinarySemaphoreENV    = xSemaphoreCreateBinary( );
  g_xBinarySemaphoreTempHumi = xSemaphoreCreateBinary( );
  g_ch1_lock = xSemaphoreCreateMutex();

  xTaskCreate(lvgl_task, "lvgl_task", 2000, NULL, osPriorityNormal, NULL);

  /* Task1 */
  xTaskCreate(
      LibmodbusServerTask, // 函数指针, 任务函数
      "LibmodbusServerTask", // 任务的名�?
      300, // 栈大�?,单位为word,10表示40字节
      NULL, // 调用任务函数时传入的参数
      osPriorityNormal, // 优先�?
      NULL); // 任务句柄, 以后使用它来操作这个任务

      g_ch1_ctx = modbus_new_st_rtu("uart2", 115200, 'N', 8, 1);
      modbus_set_slave(g_ch1_ctx, 1);
      
      if (modbus_connect(g_ch1_ctx) == -1) {
          //fprintf(stderr, "Unable to connect %s\n", modbus_strerror(errno));
          modbus_free(modbus_connect(g_ch1_ctx));
          return;
      }


  /* Task2 */
  xTaskCreate(
      LibmodbusCH1SwitchClientTask, // 函数指针, 任务函数
      "LibmodbusCH1SwitchClientTask", // 任务的名�?
      300, // 栈大�?,单位为word,10表示40字节
      NULL, // 调用任务函数时传入的参数
      osPriorityNormal, // 优先�?
      NULL); // 任务句柄, 以后使用它来操作这个任务

  /* Task3 */
  xTaskCreate(
      LibmodbusCH1ENVClientTask, // 函数指针, 任务函数
      "LibmodbusCH1ENVClientTask", // 任务的名�?
      300, // 栈大�?,单位为word,10表示40字节
      NULL, // 调用任务函数时传入的参数
      osPriorityNormal, // 优先�?
      NULL); // 任务句柄, 以后使用它来操作这个任务

  /* Task4 */
  xTaskCreate(
      LibmodbusCH2TempHumiClientTask, // 函数指针, 任务函数
      "LibmodbusCH2TempHumiClientTask", // 任务的名�?
      300, // 栈大�?,单位为word,10表示40字节
      NULL, // 调用任务函数时传入的参数
      osPriorityNormal, // 优先�?
      NULL); // 任务句柄, 以后使用它来操作这个任务

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN defaultTask */
  /* Infinite loop */
  for(;;)
  {
	  ux_system_tasks_run();
      vTaskDelay(100);         // 10ms刷新一次，100Hz刷新率
  }
  /* USER CODE END defaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

