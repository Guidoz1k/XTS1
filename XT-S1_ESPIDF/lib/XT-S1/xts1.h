#ifndef __XTS1_H
#define __XTS1_H

#include <stdio.h>
#include <driver/uart.h>
#include <esp_err.h>
#include <esp_log.h>
#include <stdbool.h>
#include <stdint.h>

// ========== DEFINITIONS ==========

#define TIMEOUT 12 // modbus comms timeout in system tick (12ms for tickrate of 1000Hz)

// Modbus / CRC
#define MODBUS_DEVICE_ID           0x01
#define MODBUS_FC_READ_HOLDING     0x03
#define MODBUS_FC_READ_INPUT       0x04
#define MODBUS_FC_WRITE_SINGLE     0x06
#define CRC_INIT                   0xFFFF
#define CRC_POLY                   0xA001

// UART / timing
#define UART_BAUD_RATE             115200
#define UART_TX_PIN                4
#define UART_RX_PIN                5
#define UART_DRV_RX_BUF_SIZE       2048
#define UART_DRV_TX_BUF_SIZE       0
#define UART_EVENT_QUEUE_SIZE      0
#define UART_RX_FLOW_CTRL_THRESH   122
#define READ_TIMEOUT_MS            10

// Modbus frame sizes (bytes)
#define MODBUS_FRAME_SIZE_WRITE    8
#define MODBUS_FRAME_SIZE_READ     7
#define MODBUS_FRAME_SIZE_SYSERR   9

// Other constants
#define BYTE_MASK                  0xFF
#define CRC_DATA_LENGTH            6  // bytes before CRC in Modbus frame


#define XTS1_IS_READ_HOLDING(idx) \
    ( (idx) == XT_S1_IDX_DISTANCE_CURRENT_SELECT || \
      (idx) == XT_S1_IDX_INTEGRATION_TIME || \
      (idx) == XT_S1_IDX_EXTRA_DELAY_TIMING || \
      (idx) == XT_S1_IDX_DISTANCE_MEASURE_COMMAND || \
      (idx) == XT_S1_IDX_CHIP_REGISTER_OPERATION || \
      (idx) == XT_S1_IDX_CHIP_REGISTER_ADDRESS || \
      (idx) == XT_S1_IDX_FLASH_OPERATION_COMMAND || \
      (idx) == XT_S1_IDX_REGISTER_WRITE_PROTECTION || \
      (idx) == XT_S1_IDX_IIC_UART_ADDRESS || \
      (idx) == XT_S1_IDX_MODULE_OPERATION_MODE || \
      (idx) == XT_S1_IDX_CONTINUOUS_MEASUREMENT_PERIOD || \
      (idx) == XT_S1_IDX_AMP_VALUE_UFS_MODE || \
      (idx) == XT_S1_IDX_BEST_AMP_VALUE_UFS_MODE )

#define XTS1_IS_READ_INPUT(idx) \
    ( (idx) == XT_S1_IDX_CURRENT_INTEGRATION_TIME || \
      (idx) == XT_S1_IDX_DISTANCE_INFORMATION || \
      (idx) == XT_S1_IDX_TEMPERATURE || \
      (idx) == XT_S1_IDX_SIGNAL_AMPLITUDE || \
      (idx) == XT_S1_IDX_BACKGROUND_LIGHT || \
      (idx) == XT_S1_IDX_WAFER_ID || \
      (idx) == XT_S1_IDX_CHIP_ID || \
      (idx) == XT_S1_IDX_SERIAL_OUTPUT_MODE )


// UART config macro
#define UART_CONFIG_DEFAULT { \
    .baud_rate = UART_BAUD_RATE, \
    .data_bits = UART_DATA_8_BITS, \
    .parity = UART_PARITY_DISABLE, \
    .stop_bits = UART_STOP_BITS_1, \
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, \
    .rx_flow_ctrl_thresh = UART_RX_FLOW_CTRL_THRESH, \
    .source_clk = UART_SCLK_DEFAULT \
}

typedef struct {
    uint8_t device_id;         // device ID (1)
    uint8_t function;       // function code (1)
    uint8_t address_hi;   // register address (2)
    uint8_t address_lo;   // register address (2)
    uint8_t data_hi;  // number of registers / data (2)
    uint8_t data_lo;  // number of registers / data (2)
    uint8_t crc_hi;       // CRC16 (2)
    uint8_t crc_lo;       // CRC16 (2)
} __attribute__((packed)) modbus_request_t;

typedef enum
{
    /* ---------------- Holding Registers (R/W) ---------------- */

    XT_S1_IDX_DISTANCE_CURRENT_SELECT      = 0,   // 0x00
    XT_S1_IDX_INTEGRATION_TIME              = 1,   // 0x02
    XT_S1_IDX_EXTRA_DELAY_TIMING            = 2,   // 0x04
    XT_S1_IDX_DISTANCE_MEASURE_COMMAND      = 3,   // 0x06
    XT_S1_IDX_CHIP_REGISTER_OPERATION       = 4,   // 0x08
    XT_S1_IDX_CHIP_REGISTER_ADDRESS         = 5,   // 0x0A
    XT_S1_IDX_FLASH_OPERATION_COMMAND       = 6,   // 0x0C
    XT_S1_IDX_REGISTER_WRITE_PROTECTION     = 7,   // 0x0E

    /* Reserved block */
    XT_S1_IDX_RESERVED_BLOCK_1              = 8,   // 0x10 (length 16)

    /* ---------------- Input Registers (Read-only) ---------------- */

    XT_S1_IDX_SYSTEM_ERROR_CODE              = 16,  // 0x20
    XT_S1_IDX_CURRENT_INTEGRATION_TIME       = 22,  // 0x2C
    XT_S1_IDX_DISTANCE_INFORMATION           = 23,  // 0x2E
    XT_S1_IDX_TEMPERATURE                    = 24,  // 0x30
    XT_S1_IDX_SIGNAL_AMPLITUDE               = 25,  // 0x32
    XT_S1_IDX_BACKGROUND_LIGHT               = 26,  // 0x34
    XT_S1_IDX_WAFER_ID                       = 59,  // 0x76
    XT_S1_IDX_CHIP_ID                        = 60,  // 0x78
    XT_S1_IDX_SERIAL_OUTPUT_MODE             = 61,  // 0x7A

    /* ---------------- Holding Registers (R/W) ---------------- */

    XT_S1_IDX_IIC_UART_ADDRESS               = 64,  // 0x80
    XT_S1_IDX_MODULE_OPERATION_MODE          = 65,  // 0x82
    XT_S1_IDX_CONTINUOUS_MEASUREMENT_PERIOD  = 66,  // 0x84
    XT_S1_IDX_AMP_VALUE_UFS_MODE             = 86,  // 0xAC
    XT_S1_IDX_BEST_AMP_VALUE_UFS_MODE        = 87,  // 0xAE

    /* ---------------- Factory / Version ---------------- */

    XT_S1_IDX_FACTORY_CODE                   = 304, // 0x260
    XT_S1_IDX_CUSTOMER_CODE                  = 318, // 0x27C
    XT_S1_IDX_VERSION_NUMBER                 = 319  // 0x27E

} xts1_register_index_t;

void xts1_setup();

esp_err_t xts1_write_register( uint16_t address_index, uint16_t value );

esp_err_t xts1_read_register( uint16_t address_index, uint16_t *value );

esp_err_t xts1_sys_error( uint32_t *value );

uint16_t xts1_measure_distance();

#endif /* __XTS1_H */