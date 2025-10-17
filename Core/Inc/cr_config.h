#ifndef __CR_CONFIG_H
#define __CR_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define CONFIG_SCHEMA_VERSION 1
#define DEFAULT_DEVICE_ID 21

struct __attribute__((packed, aligned(4))) MotorConfigData {
    uint8_t motor_type;
    uint16_t pulses_per_rev;
    bool motor_inverted;
    bool encoder_inverted;
    uint16_t slew_rate;
    uint16_t p;
    uint16_t i;
    uint16_t d;
    uint16_t ff;
};

struct __attribute__((packed, aligned(4))) ConfigData {
    uint32_t crc32;
    uint8_t last_initialized_version;
    uint8_t device_id;

    uint8_t aux_mode;

    struct MotorConfigData m1;
    struct MotorConfigData m2;
};

extern struct ConfigData config_data;

void load_config_to_registers();
void save_registers_to_config();

void load_default_config();
uint32_t save_config_data();
void reload_config_data();

#ifdef __cplusplus
}
#endif

#endif /* __CR_CONFIG_H */