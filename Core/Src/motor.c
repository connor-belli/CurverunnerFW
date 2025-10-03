#include "motor.h"

#include "stm32f1xx.h"

void open_loop_bdc_init(struct CRMotor *motor)
{
    struct CRMotorIO *io = &motor->io;
    GPIO_InitTypeDef GPIO_InitStruct = {0};

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
    GPIO_InitTypeDef GPIO_InitStruct = {0};

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

    if (power >= 0.0f)
    {
        __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m1, (uint32_t)(power * arr));
        __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m2, 0);
    }
    else
    {
        __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m1, 0);
        __HAL_TIM_SET_COMPARE(io->tim_motor, io->chan_m2, (uint32_t)(-power * arr));
    }
}

void open_loop_update(struct CRMotor *motor, float dt)
{
    if (motor->control_mode == MOTORCONTROL_PERCENT_OUTPUT)
        open_loop_set_percent_out(motor, motor->percent_output);
    else
        open_loop_set_percent_out(motor, 0.0f); // Unsupported control mode
}

void closed_loop_bdc_init(struct CRMotor *motor)
{
    struct CRMotorIO *io = &motor->io;
    struct ClosedLoopBDC *data = &motor->closed_loop_bdc_data;
    TIM_Encoder_InitTypeDef sConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    // Initialize Motor channels
    GPIO_InitTypeDef GPIO_InitStruct = {0};

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
    io->tim_enc->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    HAL_TIM_Encoder_Init(io->tim_enc, &sConfig);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(io->tim_enc, &sMasterConfig);
}

uint32_t closed_loop_get_encoder_count(struct CRMotor *motor)
{
    struct CRMotorIO *io = &motor->io;
    // TODO: handle overflow
    return __HAL_TIM_GET_COUNTER(io->tim_enc);
}

void closed_loop_bdc_deinit(struct CRMotor *motor)
{
}

void closed_loop_update(struct CRMotor *motor, float dt)
{
    struct ClosedLoopBDC *data = &motor->closed_loop_bdc_data;
    if (motor->control_mode == MOTORCONTROL_PERCENT_OUTPUT)
        open_loop_set_percent_out(motor, motor->percent_output);
    else
        open_loop_set_percent_out(motor, 0.0f); // Unsupported control mode
}

void fit0441_init(struct CRMotor *motor)
{
}

void fit0441_deinit(struct CRMotor *motor)
{
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

    switch (motor->motor_type)
    {
    case MOTORTYPE_OPEN_LOOP_BDC:
        open_loop_bdc_deinit(motor);
        break;
    case MOTORTYPE_CLOSED_LOOP_BDC:
        closed_loop_bdc_deinit(motor);
        break;
    case MOTORTYPE_FIT0441:
        fit0441_deinit(motor);
        break;
    case MOTORTYPE_DISABLED:
        break;
    }

    motor->motor_type = motor_type;

    switch (motor->motor_type)
    {
    case MOTORTYPE_OPEN_LOOP_BDC:
        open_loop_bdc_init(motor);
        break;
    case MOTORTYPE_CLOSED_LOOP_BDC:
        closed_loop_bdc_init(motor);
        break;
    case MOTORTYPE_FIT0441:
        fit0441_init(motor);
        break;
    case MOTORTYPE_DISABLED:
        break;
    }
}

void motor_set_percent_out(struct CRMotor *motor, float power)
{
    if (motor->motor_type != MOTORTYPE_OPEN_LOOP_BDC)
        return;

    motor->control_mode = MOTORCONTROL_PERCENT_OUTPUT;
    motor->percent_output = power;
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
    if (motor->motor_type != MOTORTYPE_CLOSED_LOOP_BDC)
        return 0;

    return motor->target_position_raw;
}

int16_t motor_get_velocity_raw(struct CRMotor *motor)
{
    if (motor->motor_type != MOTORTYPE_CLOSED_LOOP_BDC && motor->motor_type != MOTORTYPE_FIT0441)
        return 0;

    return motor->target_velocity_raw;
}

void motor_update(struct CRMotor *motor, float dt)
{
    switch (motor->motor_type)
    {
    case MOTORTYPE_DISABLED:
        return;
    case MOTORTYPE_OPEN_LOOP_BDC:
        open_loop_update(motor, dt);
        break;
    case MOTORTYPE_CLOSED_LOOP_BDC:
        closed_loop_update(motor, dt);
        break;
    case MOTORTYPE_FIT0441:
        return; // Not implemented
        break;
    default:
        return;
    }
}
