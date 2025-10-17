#include "i2c_slave.h"

/* Include system header files -----------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

/* Include user header files -------------------------------------------------*/
#include "main.h"
#include "registers.h"
// Peripherals
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_i2c.h"
/* Exported macros -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
// I2C direction
#define I2C_DIR_SLAVE_TO_MASTER I2C_DIRECTION_RECEIVE
#define I2C_DIR_MASTER_TO_SLAVE I2C_DIRECTION_TRANSMIT

/* Private types -------------------------------------------------------------*/
/* Private enum tag ----------------------------------------------------------*/
enum {
    I2C_STATE_NONE,
    I2C_STATE_READY,
    I2C_STATE_SLAVE_ADDR_MATCHED,
    I2C_STATE_REG_ADDR_RECEIVED,
    I2C_STATE_SLAVE_TO_MASTER,
    I2C_STATE_MASTER_TO_SLAVE,
    I2C_STATE_LISTEN_MODE_END,
    I2C_STATE_ERROR_OCCURED,
};

/* Private struct/union tag --------------------------------------------------*/
/* Imported variables --------------------------------------------------------*/
#define I2C_SLAVE_HANDLE_PTR (&hi2c2) // I2C_HandleTypeDef
/* Exported variables --------------------------------------------------------*/
/* Imported variables --------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static uint8_t i2c_slave_addr = 21;
static int i2c_state = I2C_STATE_NONE;
static uint8_t i2c_transfer_direction; // I2C_DIR_MASTER_TO_SLAVE or I2C_DIR_SLAVE_TO_MASTER
static uint8_t i2c_reg_addr = 0;
static uint8_t i2c_wdata = 0;

/* Exported functions prototypes----------------------------------------------*/
/* Imported function prototypes ----------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
static uint8_t i2c_read_byte_slave(uint8_t);
static void i2c_write_byte_slave(uint8_t, uint8_t);

/* Exported functions --------------------------------------------------------*/
void i2c_init_slave()
{
    HAL_I2C_StateTypeDef i2cStatus;

    // Reinitialize I2C
    i2cStatus = HAL_I2C_GetState(I2C_SLAVE_HANDLE_PTR);
    if (i2cStatus == HAL_I2C_STATE_LISTEN || i2cStatus == HAL_I2C_STATE_BUSY_TX_LISTEN ||
        i2cStatus == HAL_I2C_STATE_BUSY_RX_LISTEN) {
        HAL_I2C_DisableListen_IT(I2C_SLAVE_HANDLE_PTR);
    }
    HAL_I2C_DeInit(I2C_SLAVE_HANDLE_PTR);
    I2C_SLAVE_HANDLE_PTR->Init.OwnAddress1 = i2c_slave_addr << 1;
    HAL_I2C_Init(I2C_SLAVE_HANDLE_PTR);
    i2c_state = I2C_STATE_READY;
    if (HAL_I2C_GetState(I2C_SLAVE_HANDLE_PTR) != HAL_I2C_STATE_LISTEN) {
        HAL_I2C_EnableListen_IT(I2C_SLAVE_HANDLE_PTR);
    }
}

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t direction, uint16_t addr)
{
    static uint8_t i2cBuf;
    if (addr != (i2c_slave_addr << 1) || hi2c != I2C_SLAVE_HANDLE_PTR) {
        return;
    }
    i2c_transfer_direction = direction;
    switch (i2c_state) {
    case I2C_STATE_READY:
        i2c_state = I2C_STATE_SLAVE_ADDR_MATCHED;
        if (i2c_transfer_direction == I2C_DIR_SLAVE_TO_MASTER) {
            // In the initial state, the next communication direction must be Tx mode ??
            return;
        }

        // Read register address
        HAL_I2C_Slave_Seq_Receive_IT(I2C_SLAVE_HANDLE_PTR, &i2c_reg_addr, 1, I2C_FIRST_FRAME);
        break;

    case I2C_STATE_REG_ADDR_RECEIVED:
        // start MemRead
        i2c_state = I2C_STATE_SLAVE_TO_MASTER;

        // When AddressCallback is called after receiving a register address, it must be Rx mode for restart ??
        if (i2c_transfer_direction == I2C_DIR_MASTER_TO_SLAVE) {
            return;
        }

        i2cBuf = i2c_read_byte_slave(i2c_reg_addr);
        HAL_I2C_Slave_Seq_Transmit_IT(I2C_SLAVE_HANDLE_PTR, &i2cBuf, 1, I2C_NEXT_FRAME);
        i2c_reg_addr++;
        break;

    default:
        break;
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c != I2C_SLAVE_HANDLE_PTR) {
        return;
    }

    switch (i2c_state) {
    case I2C_STATE_SLAVE_ADDR_MATCHED:
        // Complete to read register address
        i2c_state = I2C_STATE_REG_ADDR_RECEIVED;

        // From here, branch to MemWrite mode and MemRead mode

        // Waiting to receive data for MemWrite mode
        HAL_I2C_Slave_Seq_Receive_IT(I2C_SLAVE_HANDLE_PTR, &i2c_wdata, 1, I2C_NEXT_FRAME);
        break;

    case I2C_STATE_REG_ADDR_RECEIVED:
        // MemWrite mode: first byte
        i2c_state = I2C_STATE_MASTER_TO_SLAVE;
        i2c_write_byte_slave(i2c_reg_addr, i2c_wdata);
        i2c_reg_addr++;

        // Wait for second byte to be read
        HAL_I2C_Slave_Seq_Receive_IT(I2C_SLAVE_HANDLE_PTR, &i2c_wdata, 1, I2C_NEXT_FRAME);
        break;

    case I2C_STATE_MASTER_TO_SLAVE:
        // MemWrite mode:　After 2nd bytes
        i2c_write_byte_slave(i2c_reg_addr, i2c_wdata);
        i2c_reg_addr++;

        // Wait for 3rd and subsequent bytes to be read
        HAL_I2C_Slave_Seq_Receive_IT(I2C_SLAVE_HANDLE_PTR, &i2c_wdata, 1, I2C_NEXT_FRAME);
        break;

    default:
        break;
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    static uint8_t i2cBuf;

    if (hi2c != I2C_SLAVE_HANDLE_PTR) {
        return;
    }
    switch (i2c_state) {
    case I2C_STATE_SLAVE_TO_MASTER:
        // MemRead mode: After 2nd bytes
        i2cBuf = i2c_read_byte_slave(i2c_reg_addr);
        HAL_I2C_Slave_Seq_Transmit_IT(I2C_SLAVE_HANDLE_PTR, &i2cBuf, 1, I2C_NEXT_FRAME);
        i2c_reg_addr++;
        break;

    default:
        break;
    }
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2c_state = I2C_STATE_READY;
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    i2c_state = I2C_STATE_READY;
    HAL_I2C_EnableListen_IT(hi2c);
}

uint8_t i2c_get_slave_addr()
{
    return i2c_slave_addr;
}

void i2c_set_slave_addr(uint8_t addr)
{
    i2c_slave_addr = addr;
    i2c_init_slave();
}

/* Private functions ---------------------------------------------------------*/
static void i2c_write_byte_slave(uint8_t regAddr, uint8_t data)
{
    volatile uint8_t *reg_data = (volatile uint8_t *)&registers;
    if (regAddr < sizeof(struct ComRegisters)) {
        reg_data[regAddr] = data;
    }
}

static uint8_t i2c_read_byte_slave(uint8_t regAddr)
{
    volatile uint8_t *reg_data = (volatile uint8_t *)&registers;
    uint8_t data = 0;
    if (regAddr < sizeof(struct ComRegisters)) {
        data = reg_data[regAddr];
    }
    return data;
}
/***************************************************************END OF FILE****/