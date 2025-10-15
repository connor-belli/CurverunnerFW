#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include "pid_control.h"
#include "stm32f1xx_hal.h"

#define MOTOR_MA_SIZE 20

enum CRMotorType
{
    MOTORTYPE_DISABLED = 0,
    MOTORTYPE_OPEN_LOOP_BDC = 1,
    MOTORTYPE_CLOSED_LOOP_BDC = 2,
    MOTORTYPE_FIT0441 = 3
};

enum MotorControlMode
{
    MOTORCONTROL_DISABLED = 0,
    MOTORCONTROL_PERCENT_OUTPUT = 1,
    MOTORCONTROL_VELOCITY = 2,
    MOTORCONTROL_POSITION = 3
};

struct MotorConfig
{

};


struct OpenLoopBDCConfig
{
    bool motor_inverted;
};

struct OpenLoopBDC
{
};

struct ClosedLoopBDCConfig
{
    int pulses_per_rev;
    bool encoder_inverted;
    bool motor_inverted;

    struct PIDParams pid_params;
    float ff;
};

struct ClosedLoopBDC
{
    struct PIDController pid;
    int32_t prev_encoder_count;
    int32_t encoder_count;
    enum MotorControlMode prev_control_mode;

    int prev_count;
    float ma_buff[MOTOR_MA_SIZE];
    int ma_index;

    float cur_vel;
};

struct FIT0441Config
{

};

struct FIT0441
{
};

struct CRMotorIO {
    GPIO_TypeDef *gpio_m;
    uint16_t pin_m1;
    uint16_t pin_m2;
    GPIO_TypeDef *gpio_enc;
    uint16_t pin_a;
    uint16_t pin_b;

    TIM_HandleTypeDef* tim_enc;
    TIM_HandleTypeDef* tim_motor;

    uint8_t chan_m1;
    uint8_t chan_m2;
};

struct CRMotor
{
    struct CRMotorIO io;

    enum MotorControlMode control_mode;
    int32_t target_position_raw;
    int32_t target_velocity_raw;
    float percent_output;

    enum CRMotorType motor_type;
    union
    {
        struct ClosedLoopBDC closed_loop_bdc_data;
        struct OpenLoopBDC open_loop_bdc_data;
        struct FIT0441 fit0441_data;
    };

    union
    {
        struct ClosedLoopBDCConfig closed_loop_bdc_config;
        struct OpenLoopBDCConfig open_loop_bdc_config;
        struct FIT0441Config fit0441_config;
    };
    
};

void motor_init(struct CRMotor* motor, struct CRMotorIO io, enum CRMotorType motor_type);

void motor_set_type(struct CRMotor* motor, enum CRMotorType type);

bool motor_set_control(struct CRMotor* motor, enum MotorControlMode mode);

void motor_set_disabled(struct CRMotor* motor);
void motor_set_percent_out(struct CRMotor* motor, float power);
bool motor_set_target_position(struct CRMotor* motor, int32_t position_raw);
bool motor_set_target_velocity(struct CRMotor* motor, int16_t velocity_raw);

int32_t motor_get_position_raw(struct CRMotor* motor);
int16_t motor_get_velocity_raw(struct CRMotor* motor);
int16_t motor_get_percent_output(struct CRMotor* motor);

void motor_update(struct CRMotor* motor, float dt);

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */