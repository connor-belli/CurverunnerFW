#include "curverunner.h"

#include <assert.h>
#include <ctype.h>
#include <stdint.h>
#include <string.h>

#include "FreeRTOS.h"
#include "cr_config.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_i2c.h"
#include "task.h"
#include "usbd_cdc_if.h"

#include "curverunner_io_map.h"
#include "i2c_slave.h"
#include "main.h"
#include "motor.h"
#include "registers.h"

static struct CRMotor motor1 = { 0 };
static struct CRMotor motor2 = { 0 };

static void blink_task(__attribute__((unused)) void *pvParameters)
{
    for (;;) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
        vTaskDelay(500);
    }
}

static void handle_write_command(uint32_t addr, uint32_t param2, uint32_t size)
{
    size_t i = 0;
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

static void handle_read_command(uint32_t param1, uint32_t size)
{
    uint32_t data = 0;
    char buffer[32];
    size_t len = 0;
    size_t i = 0;

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

static void serial_task(__attribute__((unused)) void *pvParameters)
{
    struct SerialCommand command;
    int count = 0;

    for (;;) {
        xQueueReceive(serial_comm.command_queue, (void *)&command, portMAX_DELAY);

        if (command.status == false) {
            CDC_Transmit_FS((uint8_t *)"?\n", 2);
        } else if (command.command[0] == 'W') {
            if (isdigit((unsigned char)command.command[1]) && command.command[2] == '\0') {
                count = command.command[1] - '0';
                handle_write_command(command.param1, command.param2, count);
            }
        } else if (command.command[0] == 'R') {
            if (isdigit((unsigned char)command.command[1]) && command.command[2] == '\0') {
                count = command.command[1] - '0';
                handle_read_command(command.param1, count);
            }
        }
    }
}

void servo_write(volatile uint32_t *pwm_reg, uint16_t value)
{
    uint32_t min_servo_pwm = 1000 * 72 / 22;
    uint32_t servo_slope;

    if (value > 1800)
        value = 1800;

    servo_slope = value * (1000 * 72 / 1800) / 22;
    *pwm_reg = servo_slope + min_servo_pwm;
}

static void init_motors()
{
    struct CRMotorIO motor1_io = { .gpio_m = GPIO_MOT1A,
                                   .pin_m1 = PIN_MOT1A,
                                   .pin_m2 = PIN_MOT1B,
                                   .gpio_enc = GPIO_ENC1A,
                                   .pin_a = PIN_ENC1A,
                                   .pin_b = PIN_ENC1B,
                                   .tim_enc = &htim2,
                                   .tim_motor = &htim4,
                                   .chan_m1 = TIM_CHANNEL_3,
                                   .chan_m2 = TIM_CHANNEL_4 };

    struct CRMotorIO motor2_io = { .gpio_m = GPIO_MOT2A,
                                   .pin_m1 = PIN_MOT2A,
                                   .pin_m2 = PIN_MOT2B,
                                   .gpio_enc = GPIO_ENC2A,
                                   .pin_a = PIN_ENC2A,
                                   .pin_b = PIN_ENC2B,
                                   .tim_enc = &htim3,
                                   .tim_motor = &htim4,
                                   .chan_m1 = TIM_CHANNEL_1,
                                   .chan_m2 = TIM_CHANNEL_2 };

    motor_init(&motor1, motor1_io, MOTORTYPE_CLOSED_LOOP_BDC);
    motor_init(&motor2, motor2_io, MOTORTYPE_OPEN_LOOP_BDC);
    registers.m1.motor_type = MOTORTYPE_CLOSED_LOOP_BDC;

    motor1.pulses_per_rev = 2800;
    motor1.encoder_inverted = true;
    motor1.motor_inverted = false;
}

void read_motor_registers(struct MotorRegisters *reg, struct CRMotor *motor)
{
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
        motor_set_type(motor, reg->motor_type);
        reg->control_mode = MOTORCONTROL_DISABLED;
        reg->target = 0;
    }

    // Update common motor settings
    pid_create_params(&motor->pid_params, (float)reg->p / REG_PID_SCALE, (float)reg->i / REG_PID_SCALE,
                      (float)reg->d / REG_PID_SCALE);
    motor->ff = (float)reg->ff / REG_PID_SCALE;
    if (reg->pulses_per_rev == 0) {
        reg->pulses_per_rev = 1;
    }
    motor->pulses_per_rev = reg->pulses_per_rev;
    motor->encoder_inverted = reg->encoder_inverted;
    motor->motor_inverted = reg->motor_inverted;
    motor->slew_rate = (float)reg->slew_rate / REG_PID_SCALE;

    // Apply control mode changes
    if (reg->control_mode == MOTORCONTROL_POSITION) {
        motor_set_target_position(motor, reg->target);
        reg->control_mode = 0xFF;
    } else if (reg->control_mode == MOTORCONTROL_VELOCITY) {
        motor_set_target_velocity(motor, reg->target);
        reg->control_mode = 0xFF;
    } else if (reg->control_mode == MOTORCONTROL_PERCENT_OUTPUT) {
        motor_set_percent_out(motor, ((int16_t) reg->target) / (float)INT16_MAX);
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

void update_motor_registers(struct MotorRegisters *reg, struct CRMotor *motor)
{
    taskENTER_CRITICAL();
    reg->pos = motor_get_position_raw(motor);
    reg->vel = motor_get_velocity_raw(motor);
    reg->percent_output = (int16_t)(motor->percent_output * INT16_MAX);
    taskEXIT_CRITICAL();
}

void handle_commands()
{
    switch (registers.command) {
    case COMMAND_SAVE_CONFIG:
        save_registers_to_config();
        save_config_data();
        registers.command = 0;
        break;
    case COMMAND_RELOAD_CONFIG:
        reload_config_data();
        load_config_to_registers();
        registers.command = 0;
        break;
    case COMMAND_FACTORY_RESET:
        load_default_config();
        save_config_data();
        load_config_to_registers();
        registers.command = 0;
        break;
    case COMMAND_SET_DEVICE_ID:
        if (registers.device_id != 0) {
            config_data.device_id = registers.device_id;
            i2c_set_slave_addr(config_data.device_id);
            save_config_data();
        } else {
            registers.device_id = config_data.device_id;
        }
        registers.command = 0;
        break;
    default:
        break;
    }
}

int curverunner_main(__attribute__((unused)) void *argument)
{
    BaseType_t retval;
    reload_config_data();
    load_config_to_registers();
    serialcomm_init(&serial_comm);
    i2c_init_slave();

    retval = xTaskCreate(blink_task, "LED_BLINK", 128, NULL, tskIDLE_PRIORITY + 1, NULL);
    if (retval != pdPASS) {
        assert(0);
    }

    retval = xTaskCreate(serial_task, "SERIAL", 256, NULL, tskIDLE_PRIORITY + 50, NULL);
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

        read_motor_registers(&registers.m1, &motor1);
        motor_update(&motor1, 0.001f);
        update_motor_registers(&registers.m1, &motor1);

        handle_commands();

        vTaskDelay(1);
    }
}
