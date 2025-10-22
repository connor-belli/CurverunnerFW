#include "motor.h"

#include "stm32f1xx.h"
#include <string.h>

void open_loop_bdc_init(struct CRMotor *motor);
void open_loop_bdc_deinit(struct CRMotor *motor);
void open_loop_set_percent_out(struct CRMotor *motor, float power);
void open_loop_update_sensors(struct CRMotor *motor, float dt);

void closed_loop_bdc_init(struct CRMotor *motor);
void closed_loop_bdc_deinit(struct CRMotor *motor);
void closed_loop_set_percent_out(struct CRMotor *motor, float power);
void closed_loop_update_sensors(struct CRMotor *motor, float dt);

struct MotorInterface {
    void (*init)(struct CRMotor *motor);
    void (*deinit)(struct CRMotor *motor);
    void (*set_percent_out)(struct CRMotor *motor, float power);
    void (*update_sensors)(struct CRMotor *motor, float dt);

    bool supports_position;
    bool supports_velocity;
};

static struct MotorInterface open_loop_bdc_interface = {
    .init = open_loop_bdc_init,
    .deinit = open_loop_bdc_deinit,
    .set_percent_out = open_loop_set_percent_out,
    .update_sensors = open_loop_update_sensors,
    .supports_position = false,
    .supports_velocity = false,
};

static struct MotorInterface closed_loop_bdc_interface = {
    .init = closed_loop_bdc_init,
    .deinit = closed_loop_bdc_deinit,
    .set_percent_out = closed_loop_set_percent_out,
    .update_sensors = closed_loop_update_sensors,
    .supports_position = true,
    .supports_velocity = true,
};

/// @brief Unwrap a 16-bit encoder reading to a 32-bit value
/// @param in Current 16-bit reading
/// @param prev Previous unwrapped value
/// @return Unwrapped 32-bit encoder value
int32_t unwrap_encoder(uint16_t in, int32_t *prev)
{
    const int ONE_PERIOD = 65536;
    const int HALF_PERIOD = 32768;
    int32_t c32 = (int32_t)in - HALF_PERIOD; // remove half period to determine (+/-) sign of the wrap
    int32_t dif = (c32 - *prev); // core concept: prev + (current - prev) = current

    // wrap difference from -HALF_PERIOD to HALF_PERIOD. modulo prevents differences after the wrap from having an incorrect result
    int32_t mod_dif = ((dif + HALF_PERIOD) % ONE_PERIOD) - HALF_PERIOD;
    if (dif < -HALF_PERIOD)
        mod_dif += ONE_PERIOD; // account for mod of negative number behavior in C

    int32_t unwrapped = *prev + mod_dif;
    *prev = unwrapped; // load previous value

    return unwrapped + HALF_PERIOD; // remove the shift we applied at the beginning, and return
}

void open_loop_bdc_init(struct CRMotor *motor)
{
    struct CRMotorIO *io = &motor->io;
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    // Enable PWM on motor gpio pins
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = io->pin_m1 | io->pin_m2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(io->gpio_m, &GPIO_InitStruct);

    __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m1, 0);
    __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m2, 0);

    HAL_TIM_PWM_Start(io->tim_motor, io->chan_m1);
    HAL_TIM_PWM_Start(io->tim_motor, io->chan_m2);
}

void open_loop_bdc_deinit(struct CRMotor *motor)
{
    struct CRMotorIO *io = &motor->io;
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m1, 0);
    __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m2, 0);

    HAL_TIM_PWM_Stop(io->tim_motor, io->chan_m1);
    HAL_TIM_PWM_Stop(io->tim_motor, io->chan_m2);

    // Disable PWM on motor gpio pins
    GPIO_InitStruct.Pin = io->pin_m1 | io->pin_m2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void open_loop_set_percent_out(struct CRMotor *motor, float power)
{
    struct CRMotorIO *io = &motor->io;
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(io->tim_motor);

    if (power > 1.0f)
        power = 1.0f;
    else if (power < -1.0f)
        power = -1.0f;

    if (motor->motor_inverted)
        power = -power;

    if (power >= 0.0f) {
        __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m1, (uint32_t)(power * arr));
        __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m1, 0);
        __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m2, (uint32_t)(-power * arr));
    }
}

void open_loop_update_sensors(__attribute__((unused)) struct CRMotor *motor, __attribute__((unused)) float dt)
{
}

void closed_loop_bdc_init(struct CRMotor *motor)
{
    struct CRMotorIO *io = &motor->io;
    struct ClosedLoopBDC *data = &motor->closed_loop_bdc_data;

    pid_reset(&data->pid);
    motor->prev_control_mode = MOTORCONTROL_DISABLED;
    data->prev_encoder_count = 0;
    motor->encoder_count = 0;

    data->prev_count = 0;
    memset(data->ma_buff, 0, sizeof(data->ma_buff));
    data->ma_index = 0;
    motor->cur_vel = 0.0f;

    TIM_Encoder_InitTypeDef sConfig = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };

    // Initialize Motor channels
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    // Enable PWM on motor gpio pins
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitStruct.Pin = io->pin_m1 | io->pin_m2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m1, 0);
    __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m2, 0);

    HAL_TIM_PWM_Start(io->tim_motor, io->chan_m1);
    HAL_TIM_PWM_Start(io->tim_motor, io->chan_m2);

    /* USER CODE BEGIN TIM2_Init 1 */

    /* USER CODE END TIM2_Init 1 */
    io->tim_enc->Init.Prescaler = 0;
    io->tim_enc->Init.CounterMode = TIM_COUNTERMODE_UP;
    io->tim_enc->Init.Period = 65535;
    io->tim_enc->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    io->tim_enc->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 10;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 10;
    HAL_TIM_Encoder_Init(io->tim_enc, &sConfig);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(io->tim_enc, &sMasterConfig);

    HAL_TIM_Encoder_Start(io->tim_enc, TIM_CHANNEL_ALL);
    io->tim_enc->Instance->CNT = 0;
}

void closed_loop_update_encoder_count(struct CRMotor *motor)
{
    struct CRMotorIO *io = &motor->io;
    struct ClosedLoopBDC *data = &motor->closed_loop_bdc_data;

    uint16_t enc = __HAL_TIM_GET_COUNTER(io->tim_enc);

    if (motor->encoder_inverted)
        motor->encoder_count = -unwrap_encoder(enc, &data->prev_encoder_count);
    else
        motor->encoder_count = unwrap_encoder(enc, &data->prev_encoder_count);
}

void closed_loop_bdc_deinit(struct CRMotor *motor)
{
    struct CRMotorIO *io = &motor->io;
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m1, 0);
    __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m2, 0);

    HAL_TIM_PWM_Stop(io->tim_motor, io->chan_m1);
    HAL_TIM_PWM_Stop(io->tim_motor, io->chan_m2);

    HAL_TIM_Encoder_Stop(io->tim_enc, TIM_CHANNEL_ALL);

    // Disable PWM on motor gpio pins
    GPIO_InitStruct.Pin = io->pin_m1 | io->pin_m2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void closed_loop_set_percent_out(struct CRMotor *motor, float power)
{
    struct CRMotorIO *io = &motor->io;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(io->tim_motor);

    if (power > 1.0f)
        power = 1.0f;
    else if (power < -1.0f)
        power = -1.0f;

    if (motor->motor_inverted)
        power = -power;

    if (power >= 0.0f) {
        __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m1, (uint32_t)(power * arr));
        __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m2, 0);
    } else {
        __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m1, 0);
        __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m2, (uint32_t)(-power * arr));
    }
}

void closed_loop_update_velocity(struct CRMotor *motor, float dt)
{
    struct ClosedLoopBDC *data = &motor->closed_loop_bdc_data;
    int cur_count = motor->encoder_count;
    int i = 0;
    float raw_vel = 0;
    float vel = 0;

    raw_vel = (cur_count - data->prev_count) / dt;
    data->prev_count = cur_count;
    data->ma_buff[data->ma_index] = raw_vel;
    data->ma_index = (data->ma_index + 1) % MOTOR_MA_SIZE;
    for (i = 0; i < MOTOR_MA_SIZE; i++) {
        vel += data->ma_buff[i];
    }
    vel /= MOTOR_MA_SIZE;
    motor->cur_vel = vel;
}

void closed_loop_update_sensors(struct CRMotor *motor, float dt)
{
    closed_loop_update_encoder_count(motor);
    closed_loop_update_velocity(motor, dt);
}

void fit0441_init(__attribute__((unused)) struct CRMotor *motor)
{
}

void fit0441_deinit(__attribute__((unused)) struct CRMotor *motor)
{
}

void motor_apply_slew_rate(struct CRMotor *motor, float dt)
{
    float max_delta = 0.0f;
    float delta = 0.0f;

    if (motor->slew_rate > 0.0f) {
        max_delta = motor->slew_rate * dt;
        delta = motor->target_percent_output - motor->percent_output;
        if (delta > max_delta)
            delta = max_delta;
        else if (delta < -max_delta)
            delta = -max_delta;
        motor->percent_output += delta;
    } else {
        motor->percent_output = motor->target_percent_output;
    }
}

void motor_init(struct CRMotor *motor, struct CRMotorIO io, enum CRMotorType motor_type)
{
    motor->io = io;
    motor->motor_type = MOTORTYPE_DISABLED;
    motor->control_mode = MOTORCONTROL_DISABLED;
    motor->target_position_raw = 0;
    motor->target_velocity_raw = 0;
    motor->percent_output = 0.0f;

    motor_set_type(motor, motor_type);
}

void motor_set_type(struct CRMotor *motor, enum CRMotorType motor_type)
{
    if (motor->motor_type == motor_type)
        return;

    if (motor->interface != NULL) {
        motor->interface->deinit(motor);
        motor->interface = NULL;
    }

    motor->motor_type = motor_type;

    switch (motor->motor_type) {
    case MOTORTYPE_OPEN_LOOP_BDC:
        motor->interface = &open_loop_bdc_interface;
        break;
    case MOTORTYPE_CLOSED_LOOP_BDC:
        motor->interface = &closed_loop_bdc_interface;
        break;
    case MOTORTYPE_FIT0441:
        motor->interface = NULL; // To be implemented
        break;
    case MOTORTYPE_DISABLED:
        motor->interface = NULL;
        break;
    default:
        motor->interface = NULL;
        break;
    }

    motor->control_mode = MOTORCONTROL_DISABLED;
    if (motor->interface != NULL) {
        motor->interface->init(motor);
    }
}

void motor_set_disabled(struct CRMotor *motor)
{
    motor->control_mode = MOTORCONTROL_DISABLED;
    motor->percent_output = 0.0f;
}

void motor_set_percent_out(struct CRMotor *motor, float power)
{
    if (motor->motor_type != MOTORTYPE_OPEN_LOOP_BDC && motor->motor_type != MOTORTYPE_CLOSED_LOOP_BDC)
        return;

    motor->control_mode = MOTORCONTROL_PERCENT_OUTPUT;
    motor->target_percent_output = power;
}

bool motor_set_target_position(struct CRMotor *motor, int32_t position_raw)
{
    if (motor->motor_type != MOTORTYPE_CLOSED_LOOP_BDC)
        return false;

    motor->control_mode = MOTORCONTROL_POSITION;
    motor->target_position_raw = position_raw;
    return true;
}

bool motor_set_target_velocity(struct CRMotor *motor, int16_t velocity_raw)
{
    if (motor->motor_type != MOTORTYPE_CLOSED_LOOP_BDC && motor->motor_type != MOTORTYPE_FIT0441)
        return false;
    motor->control_mode = MOTORCONTROL_VELOCITY;
    motor->target_velocity_raw = velocity_raw;
    return true;
}

int32_t motor_get_position_raw(struct CRMotor *motor)
{
    if (motor->interface == NULL || !motor->interface->supports_position)
        return 0;

    return motor->encoder_count;
}

int16_t motor_get_velocity_raw(struct CRMotor *motor)
{
    if (motor->interface == NULL || !motor->interface->supports_velocity)
        return 0;

    return (int16_t)motor->cur_vel;
}

int16_t motor_get_percent_out(struct CRMotor *motor)
{
    return (int16_t)(motor->percent_output * INT16_MAX);
}

void motor_update(struct CRMotor *motor, float dt)
{
    if (motor == NULL)
        return;
    if (motor->interface == NULL)
        return;

    motor->interface->update_sensors(motor, dt);

    if ((motor->control_mode == MOTORCONTROL_POSITION || motor->control_mode == MOTORCONTROL_VELOCITY) &&
        motor->prev_control_mode != motor->control_mode) {
        // Reset PID when changing to a closed loop control mode
        pid_reset(&motor->pid);
    }

    if (motor->control_mode == MOTORCONTROL_POSITION && motor->interface->supports_position) {
        motor->pid.setpoint = ((float)motor->target_position_raw) / motor->pulses_per_rev;
        pid_controller_update(&motor->pid, (float)motor->encoder_count / motor->pulses_per_rev, dt);
        motor->target_percent_output = motor->pid.output;
    } else if (motor->control_mode == MOTORCONTROL_VELOCITY && motor->interface->supports_velocity) {
        motor->pid.setpoint = ((float)motor->target_velocity_raw) / motor->pulses_per_rev;
        pid_controller_update(&motor->pid, motor->cur_vel / motor->pulses_per_rev, dt);
        motor->target_percent_output = motor->pid.output + motor->pid.setpoint * motor->ff;
    } else if (motor->control_mode == MOTORCONTROL_PERCENT_OUTPUT) {
        // Do nothing, just use the percent output set by the user
    } else
        motor->target_percent_output = 0.0f; // Unsupported control mode

    motor_apply_slew_rate(motor, dt);

    motor->interface->set_percent_out(motor, motor->percent_output);
    motor->prev_control_mode = motor->control_mode;
}
