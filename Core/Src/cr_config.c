#include "cr_config.h"

#include "main.h"
#include "registers.h"
#include "motor.h"

#include <FreeRTOS.h>
#include <task.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_flash.h"

#define FLASH_DATA_PAGE 63
#define FLASH_START_ADDRESS 0x0800FC00

struct ConfigData config_data;

uint32_t calc_crc(void *start, size_t length)
{
    uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t *)start, length / 4);
    return crc;
}


void load_motor_config_to_registers(struct MotorConfigData *cfg, struct MotorRegisters *reg)
{
    reg->motor_type = cfg->motor_type;
    reg->pulses_per_rev = cfg->pulses_per_rev;
    reg->motor_inverted = cfg->motor_inverted;
    reg->encoder_inverted = cfg->encoder_inverted;
    reg->slew_rate = cfg->slew_rate;
    reg->p = cfg->p;
    reg->i = cfg->i;
    reg->d = cfg->d;
    reg->ff = cfg->ff;

    reg->control_mode = MOTORCONTROL_DISABLED;
    reg->target = 0;
}

void load_motor_registers_to_config(struct MotorRegisters *reg, struct MotorConfigData *cfg)
{
    cfg->motor_type = reg->motor_type;
    cfg->pulses_per_rev = reg->pulses_per_rev;
    cfg->motor_inverted = reg->motor_inverted;
    cfg->encoder_inverted = reg->encoder_inverted;
    cfg->slew_rate = reg->slew_rate;
    cfg->p = reg->p;
    cfg->i = reg->i;
    cfg->d = reg->d;
    cfg->ff = reg->ff;
}

void load_config_to_registers()
{
    taskENTER_CRITICAL();
    registers.aux_mode = config_data.aux_mode;
    registers.device_id = config_data.device_id;
    registers.device_version = CONFIG_SCHEMA_VERSION;

    load_motor_config_to_registers(&config_data.m1, &registers.m1);
    load_motor_config_to_registers(&config_data.m2, &registers.m2);

    taskEXIT_CRITICAL();
}

void save_registers_to_config()
{
    taskENTER_CRITICAL();
    config_data.aux_mode = registers.aux_mode;
    config_data.device_id = registers.device_id;

    load_motor_registers_to_config(&registers.m1, &config_data.m1);
    load_motor_registers_to_config(&registers.m2, &config_data.m2);
    taskEXIT_CRITICAL();
}

uint32_t save_config_data()
{

    uint32_t num_words = sizeof(struct ConfigData) / 4;

    FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError;
    uint32_t *data = (uint32_t *)&config_data;
    size_t i = 0;

    taskENTER_CRITICAL();

    // Update CRC
    config_data.crc32 = calc_crc((void *)&config_data + 4, sizeof(struct ConfigData) - 4);

    HAL_FLASH_Unlock();

    /* Fill EraseInit structure*/
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = FLASH_START_ADDRESS;
    EraseInitStruct.NbPages = 1;
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
    {
        return HAL_FLASH_GetError();
    }

    while (i < num_words)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, FLASH_START_ADDRESS + 4 * i, data[i]) == HAL_OK)
        {
            i++;
        }
        else
        {
            /* Error occurred while writing data in Flash memory*/
            return HAL_FLASH_GetError();
        }
    }

    HAL_FLASH_Lock();

    taskEXIT_CRITICAL();
    return FLASH_ERROR_NONE;
}

void load_default_motor_config(struct MotorConfigData *cfg)
{
    cfg->motor_type = MOTORTYPE_DISABLED;
    cfg->pulses_per_rev = 2800;
    cfg->motor_inverted = false;
    cfg->encoder_inverted = true;
    cfg->slew_rate = 0;
    cfg->p = 100;
    cfg->i = 100;
    cfg->d = 0;
    cfg->ff = 470;
}

void load_default_config()
{
    taskENTER_CRITICAL();
    config_data.last_initialized_version = CONFIG_SCHEMA_VERSION;
    config_data.device_id = DEFAULT_DEVICE_ID;
    config_data.aux_mode = 0;

    load_default_motor_config(&config_data.m1);
    load_default_motor_config(&config_data.m2);

    taskEXIT_CRITICAL();
}

void reload_config_data()
{
    __IO uint32_t *start_addr = (__IO uint32_t *)FLASH_START_ADDRESS;
    __IO uint32_t *end_addr = (__IO uint32_t *)(FLASH_START_ADDRESS + sizeof(struct ConfigData));
    __IO uint32_t *addr;
    taskENTER_CRITICAL();

    uint32_t crc = 0;
    for (addr = start_addr; addr < end_addr; addr++)
    {
        *((uint32_t *)&config_data + (addr - start_addr)) = *addr;
    }

    crc = calc_crc((void *)&config_data + 4, sizeof(struct ConfigData) - 4);

    if (crc != config_data.crc32 || config_data.last_initialized_version != CONFIG_SCHEMA_VERSION)
    {
        // CRC mismatch, load defaults
        load_default_config();
        save_config_data();
    }
    taskEXIT_CRITICAL();
}