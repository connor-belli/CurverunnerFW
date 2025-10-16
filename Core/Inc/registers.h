#ifndef __REGISTERS_H
#define __REGISTERS_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdbool.h>

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

extern struct ComRegisters registers;

#ifdef __cplusplus
}
#endif

#endif /* __REGISTERS_H */