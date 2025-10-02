#include "curverunner.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

#include "stm32f1xx_hal_gpio.h"
#include "usbd_cdc_if.h"
#include <stdint.h>
#include <assert.h>
#include <ctype.h>
#include <string.h>

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
};

struct ComRegisters registers;

static void blink_task(void *pvParameters)
{
	for (;;)
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
		vTaskDelay(500); // portTICK_PERIOD_MS);
	}
}

static void handle_write_command(uint32_t addr, uint32_t param2, uint32_t size)
{
	int i;
	volatile uint8_t *reg_data = (volatile uint8_t *)&registers;

	if (addr + size > sizeof(struct ComRegisters))
	{
		CDC_Transmit_FS((uint8_t *)"!\n", 2);
	}
	else
	{
		// NOTE: cant use memcpy because reg_data is volatile
		for(i = 0; i < size; i++) {
			reg_data[addr + i] = ((uint8_t *)&param2)[i];
		}
		CDC_Transmit_FS((uint8_t *)"S\n", 2);
	}
}

static void handle_read_command(uint32_t param1, uint32_t param2, uint32_t size)
{
	uint32_t data;
	char buffer[64];
	int i = 0;

	if (param1 + size > sizeof(struct ComRegisters))
	{
		CDC_Transmit_FS((uint8_t *)"!\n", 2);
	}
	else
	{
		// NOTE: cant use memcpy because reg_data is volatile
		for (i = 0; i < size; i++) {
			((uint8_t *)&data)[i] = *((volatile uint8_t *)&registers + param1 + i);
		}

		snprintf(buffer, sizeof(buffer), "D%lu\n", data);
		CDC_Transmit_FS((uint8_t *)buffer, strlen(buffer));
	}
}

static void serial_task(void *pvParameters)
{
	struct SerialCommand command;
	int count = 0;

	for (;;)
	{
		xQueueReceive(serial_comm.command_queue, (void *)&command, portMAX_DELAY);

		if (command.status == false)
		{
			CDC_Transmit_FS((uint8_t *)"?\n", 2);
		}
		else if (command.command[0] == 'W')
		{
			if (isdigit((unsigned char)command.command[1]) && command.command[2] == '\0') {
				count = command.command[1] - '0';
				handle_write_command(command.param1, command.param2, count);
			}
		}
		else if (command.command[0] == 'R')
		{
			if (isdigit((unsigned char)command.command[1]) && command.command[2] == '\0') {
				count = command.command[1] - '0';
				handle_read_command(command.param1, command.param2, count);
			}
		}
	}
}

void servo_write(volatile uint32_t *pwm_reg, uint16_t value)
{
	uint32_t min_servo_pwm = 1000 * 72 / 22;
	uint32_t servo_slope;
	
	if(value > 1800)
		value = 1800;

	servo_slope = value * (1000 * 72 / 1800) / 22;
	*pwm_reg = servo_slope + min_servo_pwm;
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

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);


	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	uint8_t buffer[] = "Hello, World!\r\n";
	CDC_Transmit_FS(buffer, sizeof(buffer));
	for (;;)
	{
		servo_write(&TIM1->CCR1, registers.aux1);
		servo_write(&TIM1->CCR2, registers.aux2);
		servo_write(&TIM1->CCR3, registers.aux3);

		vTaskDelay(1);
	}
}
