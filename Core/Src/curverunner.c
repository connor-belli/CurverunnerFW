#include "curverunner.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

#include "stm32f1xx_hal_gpio.h"
#include "usbd_cdc_if.h"
#include <stdint.h>
#include <assert.h>

extern ADC_HandleTypeDef hadc1;

extern I2C_HandleTypeDef hi2c2;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

#define GPIO_STS1 GPIOB;
#define PIN_STS1 GPIO_PIN_0;
#define GPIO_STS2 GPIOB;
#define PIN_STS2 GPIO_PIN_1;
#define GPIO_STS3 GPIOB;
#define PIN_STS3 GPIO_PIN_2;

#define GPIO_MOT1A GPIOB;
#define PIN_MOT1A GPIO_PIN_8;
#define GPIO_MOT1B GPIOB;
#define PIN_MOT1B GPIO_PIN_9;
#define GPIO_MOT2A GPIOB;
#define PIN_MOT2A GPIO_PIN_6;
#define GPIO_MOT2B GPIOB;
#define PIN_MOT2B GPIO_PIN_7;

#define GPIO_SCL GPIOB;
#define PIN_SDA GPIO_PIN_0;

struct __attribute__((packed)) ConfigData
{
	uint8_t i2c_address;

	uint8_t aux_mode;

	uint8_t m1_mode;
	uint16_t m1_rads_per_pulse;
	uint16_t m1_p;
	uint16_t m1_i;
	uint16_t m1_d;
	uint16_t m1_ff;
};

struct __attribute__((packed)) ComRegisters
{
	uint8_t aux_mode;
	uint16_t aux1;
	uint16_t aux2;
	uint16_t aux3;

	uint16_t servo1;
	uint16_t servo2;
	uint16_t servo3;

	uint8_t m1_mode;
	uint16_t m1_rads_per_pulse;
	uint16_t m1_p;
	uint16_t m1_i;
	uint16_t m1_d;
	uint16_t m1_ff;
	int16_t m1_target;
	int16_t m1_vel;
	int32_t m1_pos;

	uint8_t command;
	uint16_t param1;
	uint16_t param2;

	short a;
	int b;
};

static void blink_task(void *pvParameters)
{
	for (;;)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		vTaskDelay(500); // portTICK_PERIOD_MS);
	}
}

static void serial_task(void *pvParameters)
{
	struct SerialCommand command;
	char response[8];
	for (;;)
	{
		xQueueReceive(serial_comm.command_queue, (void *)&command, portMAX_DELAY);

		if (command.status == false)
		{
			CDC_Transmit_FS((uint8_t *)"?\n", 2);
		}
		else
		{
			itoa(command.param1 + command.param2, response, 10);
			CDC_Transmit_FS((uint8_t*)response, strlen(response));
		}
	}

}

int curverunner_main()
{
	BaseType_t retval;
	serialcomm_init(&serial_comm);

	retval = xTaskCreate(blink_task, "LED_BLINK", 128, NULL, tskIDLE_PRIORITY + 1, NULL);
	if (retval != pdPASS)
	{
		assert(0);
	}

	retval = xTaskCreate(serial_task, "SERIAL", 128, NULL, tskIDLE_PRIORITY + 50, NULL);
	if (retval != pdPASS)
	{
		assert(0);
	}

	// HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	// HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	// HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	// HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	uint8_t buffer[] = "Hello, World!\r\n";
	CDC_Transmit_FS(buffer, sizeof(buffer));
	for (;;)
	{
		//		TIM4->CCR1 = 60000;
		//		TIM4->CCR2 = 0;
		//		TIM4->CCR3 = 60000;
		//		TIM4->CCR4 = 0;


		vTaskDelay(2000);
	}
}
