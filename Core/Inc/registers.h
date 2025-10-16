#ifndef __REGISTERS_H
#define __REGISTERS_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdbool.h>
#include <stdint.h>

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
    uint8_t device_version;
    uint8_t device_id;
    uint8_t aux_mode;
    uint16_t aux1;
    uint16_t aux2;
    uint16_t aux3;

    uint16_t servo1;
    uint16_t servo2;
    uint16_t servo3;

    struct MotorRegisters m1;
    struct MotorRegisters m2;

    uint16_t param1;
    uint16_t param2;
    uint8_t command;
};

#define REG_PID_SCALE 1000

extern struct ComRegisters registers;

#ifdef __cplusplus
}
#endif

#endif /* __REGISTERS_H */