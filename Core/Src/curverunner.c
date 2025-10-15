#include "curverunner.h"

#include <assert.h>
#include <ctype.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "task.h"
#include "usbd_cdc_if.h"

#include "curverunner_io_map.h"
#include "main.h"
#include "motor.h"

static struct CRMotor motor1 = {0};
static struct CRMotor motor2 = {0};

struct __attribute__((packed)) ConfigData {
  uint8_t i2c_address;

  uint8_t aux_mode;

  uint8_t m1_mode;
  uint16_t m1_rads_per_pulse;
  uint16_t m1_p;
  uint16_t m1_i;
  uint16_t m1_d;
  uint16_t m1_ff;
};

struct __attribute__((packed)) MotorRegisters {
  uint8_t motor_type;
  uint16_t pulses_per_rev;
  bool motor_inverted;
  bool encoder_inverted;
  uint16_t slew_rate;
  uint16_t p;
  uint16_t i;
  uint16_t d;
  uint16_t ff;
  int32_t target;
  uint8_t control_mode;
  int16_t vel;
  int32_t pos;
  int16_t percent_output;
};

struct __attribute__((packed)) ComRegisters {
  uint8_t aux_mode;
  uint16_t aux1;
  uint16_t aux2;
  uint16_t aux3;

  uint16_t servo1;
  uint16_t servo2;
  uint16_t servo3;

  struct MotorRegisters m1;

  uint16_t param1;
  uint16_t param2;
  uint8_t command;
};

#define REG_PID_SCALE 1000

struct ComRegisters registers;

static void blink_task(void *pvParameters) {
  for (;;) {
    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
    vTaskDelay(500);
  }
}

static void handle_write_command(uint32_t addr, uint32_t param2,
                                 uint32_t size) {
  int i = 0;
  volatile uint8_t *reg_data = (volatile uint8_t *)&registers;

  if (addr + size > sizeof(struct ComRegisters)) {
    CDC_Transmit_FS((uint8_t *)"!\n", 2);
  } else {
    // NOTE: cant use memcpy because reg_data is volatile
    for (i = 0; i < size; i++) {
      reg_data[addr + i] = ((uint8_t *)&param2)[i];
    }
    CDC_Transmit_FS((uint8_t *)"S\n", 2);
  }
}

static void handle_read_command(uint32_t param1, uint32_t param2,
                                uint32_t size) {
  uint32_t data = 0;
  char buffer[32];
  size_t len = 0;
  int i = 0;

  if (param1 + size > sizeof(struct ComRegisters)) {
    CDC_Transmit_FS((uint8_t *)"!\n", 2);
  } else {
    // NOTE: cant use memcpy because reg_data is volatile
    for (i = 0; i < size; i++) {
      ((uint8_t *)&data)[i] = *((volatile uint8_t *)&registers + param1 + i);
    }

    itoa(data, buffer, 10);
    len = strlen(buffer);
    buffer[len] = '\n';
    buffer[len + 1] = '\0';
    CDC_Transmit_FS((uint8_t *)buffer, len + 1);
  }
}

static void serial_task(void *pvParameters) {
  struct SerialCommand command;
  int count = 0;

  for (;;) {
    xQueueReceive(serial_comm.command_queue, (void *)&command, portMAX_DELAY);

    if (command.status == false) {
      CDC_Transmit_FS((uint8_t *)"?\n", 2);
    } else if (command.command[0] == 'W') {
      if (isdigit((unsigned char)command.command[1]) &&
          command.command[2] == '\0') {
        count = command.command[1] - '0';
        handle_write_command(command.param1, command.param2, count);
      }
    } else if (command.command[0] == 'R') {
      if (isdigit((unsigned char)command.command[1]) &&
          command.command[2] == '\0') {
        count = command.command[1] - '0';
        handle_read_command(command.param1, command.param2, count);
      }
    }
  }
}

void servo_write(volatile uint32_t *pwm_reg, uint16_t value) {
  uint32_t min_servo_pwm = 1000 * 72 / 22;
  uint32_t servo_slope;

  if (value > 1800)
    value = 1800;

  servo_slope = value * (1000 * 72 / 1800) / 22;
  *pwm_reg = servo_slope + min_servo_pwm;
}

static void init_motors() {
  struct CRMotorIO motor1_io = {.gpio_m = GPIO_MOT1A,
                                .pin_m1 = PIN_MOT1A,
                                .pin_m2 = PIN_MOT1B,
                                .gpio_enc = GPIO_ENC1A,
                                .pin_a = PIN_ENC1A,
                                .pin_b = PIN_ENC1B,
                                .tim_enc = &htim2,
                                .tim_motor = &htim4,
                                .chan_m1 = TIM_CHANNEL_3,
                                .chan_m2 = TIM_CHANNEL_4};

  struct CRMotorIO motor2_io = {.gpio_m = GPIO_MOT2A,
                                .pin_m1 = PIN_MOT2A,
                                .pin_m2 = PIN_MOT2B,
                                .gpio_enc = GPIO_ENC2A,
                                .pin_a = PIN_ENC2A,
                                .pin_b = PIN_ENC2B,
                                .tim_enc = &htim3,
                                .tim_motor = &htim4,
                                .chan_m1 = TIM_CHANNEL_1,
                                .chan_m2 = TIM_CHANNEL_2};

  registers.m1.p = 100;
  registers.m1.i = 100;
  registers.m1.d = 0;
  registers.m1.ff = 470;
  registers.m1.pulses_per_rev = 2800;
  registers.m1.motor_inverted = false;
  registers.m1.encoder_inverted = true;
  registers.m1.slew_rate = 0;

  motor_init(&motor1, motor1_io, MOTORTYPE_CLOSED_LOOP_BDC);
  motor_init(&motor2, motor2_io, MOTORTYPE_OPEN_LOOP_BDC);
  registers.m1.motor_type = MOTORTYPE_CLOSED_LOOP_BDC;

  motor1.closed_loop_bdc_config.pulses_per_rev = 2800;
  motor1.closed_loop_bdc_config.encoder_inverted = true;
  motor1.closed_loop_bdc_config.motor_inverted = false;
}

void read_motor_registers(struct MotorRegisters *reg, struct CRMotor *motor,
                          float dt) {

  taskENTER_CRITICAL();

  if (reg->motor_type > MOTORTYPE_FIT0441) {
    // Invalid motor type, reset
    reg->motor_type = 0xFF;
    reg->control_mode = MOTORCONTROL_DISABLED;
    reg->target = 0;
  }

  // Change motor type if needed
  if (reg->motor_type != motor->motor_type) {
    motor_set_percent_out(motor, 0);
    motor_init(motor, motor->io, reg->motor_type);
    reg->control_mode = MOTORCONTROL_DISABLED;
    reg->target = 0;
  }

  // Update motor specific settings

  if (reg->motor_type == MOTORTYPE_OPEN_LOOP_BDC) {
    motor->open_loop_bdc_config.motor_inverted = reg->motor_inverted;
  } else if (reg->motor_type == MOTORTYPE_CLOSED_LOOP_BDC) {
    pid_create_params(&motor->closed_loop_bdc_config.pid_params,
                      (float)reg->p / REG_PID_SCALE,
                      (float)reg->i / REG_PID_SCALE,
                      (float)reg->d / REG_PID_SCALE);
    motor->closed_loop_bdc_config.ff = (float)reg->ff / REG_PID_SCALE;
    motor->closed_loop_bdc_config.pulses_per_rev = reg->pulses_per_rev;
    motor->closed_loop_bdc_config.encoder_inverted = reg->encoder_inverted;
    motor->closed_loop_bdc_config.motor_inverted = reg->motor_inverted;
  } else if (reg->motor_type == MOTORTYPE_FIT0441) {
    // No settings to change
  }

  // Apply control mode changes
  if (reg->control_mode == MOTORCONTROL_POSITION) {
    motor_set_target_position(motor, reg->target);
    reg->control_mode = 0xFF;
  } else if (reg->control_mode == MOTORCONTROL_VELOCITY) {
    motor_set_target_velocity(motor, reg->target);
    reg->control_mode = 0xFF;
  } else if (reg->control_mode == MOTORCONTROL_PERCENT_OUTPUT) {

    motor_set_percent_out(motor, ((float)reg->target - INT16_MAX) / INT16_MAX);
    reg->control_mode = 0xFF;
  } else if (reg->control_mode == MOTORCONTROL_DISABLED) {
    motor_set_disabled(motor);
    reg->control_mode = 0xFF;
  } else {
    // Invalid control mode, reset
    reg->control_mode = 0xFF;
  }

  taskEXIT_CRITICAL();
}

void update_motor_registers(struct MotorRegisters *reg, struct CRMotor *motor,
                            float dt) {
  taskENTER_CRITICAL();
  reg->pos = motor_get_position_raw(motor);
  reg->vel = motor_get_velocity_raw(motor);
  reg->percent_output = (int16_t)(motor->percent_output * INT16_MAX);
  taskEXIT_CRITICAL();
}

int curverunner_main() {
  BaseType_t retval;
  serialcomm_init(&serial_comm);

  retval = xTaskCreate(blink_task, "LED_BLINK", 128, NULL, tskIDLE_PRIORITY + 1,
                       NULL);
  if (retval != pdPASS) {
    assert(0);
  }

  retval = xTaskCreate(serial_task, "SERIAL", 256, NULL, tskIDLE_PRIORITY + 50,
                       NULL);
  if (retval != pdPASS) {
    assert(0);
  }

  init_motors();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

  for (;;) {
    servo_write(&TIM1->CCR1, registers.aux1);
    servo_write(&TIM1->CCR2, registers.aux2);
    servo_write(&TIM1->CCR3, registers.aux3);

    read_motor_registers(&registers.m1, &motor1, 0.001f);
    motor_update(&motor1, 0.001f);
    update_motor_registers(&registers.m1, &motor1, 0.001f);

    vTaskDelay(1);
  }
}
