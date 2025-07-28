#include "xts1.h"

// ========== DEFINITIONS ==========

#define TIMEOUT 12 // modbus comms timeout in system tick (12ms for tickrate of 1000Hz)

// ========== GLOBAL VARIABLES ==========

static const char *TAG = "UART2";   // esp_err variable
static uint8_t read_timeout = pdMS_TO_TICKS( 10 );    // UART reading timeout

// ============ INTERNAL FUNCTIONS ============

uint16_t crc16( const uint8_t *data, size_t length ) {
    uint16_t crc = 0xFFFF; // Initial CRC value as per Modbus standard [ 1 ]

    for( size_t i = 0; i < length; ++i ){
        crc ^= ( uint16_t )data[  i  ]; // XOR with current byte [ 1 ]
        for( size_t j = 0; j < 8; ++j ){
            if( crc & 0x0001 ) // If LSB (Least Significant Bit) is 1 [ 1 ]
                crc = ( crc >> 1 ) ^ 0xA001; // Shift right and XOR with polynomial 0xA001 [ 1 ]
            else
                crc >>= 1; // Shift right [ 1 ]
        }
    }
    return crc;
}

void modbus_flush(){
    uart_flush( UART_NUM_2 );
}

uint8_t modbus_check_buffer(){
    size_t size_IDF = 0;
    uint8_t size = 0;

    uart_get_buffered_data_len( UART_NUM_2, &size_IDF );
    size = ( uint8_t )size_IDF;

    return size;
}

/* important info:
    word_count parameter:
        number of words (2 bytes) to read from register
        normal registers should be 1 word long, divide the number of bytes from the datasheet by 2
    command parameter should be:
        0x03 = functional code for "read holding register"
        0x04 = functional code for "read input register"
*/
void modbus_read_register_command( uint16_t address_index, uint8_t word_count, uint8_t command ){
    uint8_t buffer[ 8 ] = { 0 };
    uint16_t crc_calc = 0;

    buffer[ 0 ] = 0x01;   // device ID
    buffer[ 1 ] = command;   // functional code

    buffer[ 2 ] = ( address_index >> 8 ) & 0xFF;
    buffer[ 3 ] = address_index & 0xFF;
    buffer[ 4 ] = ( word_count >> 8 ) & 0xFF;
    buffer[ 5 ] = word_count & 0xFF;

    crc_calc = crc16( buffer, 6 );
    buffer[ 6 ] = crc_calc & 0xFF;
    buffer[ 7 ] = ( crc_calc >> 8 ) & 0xFF;

    uart_write_bytes( UART_NUM_2, buffer, 8 );
}

void modbus_write_register_command( uint16_t address_index, uint16_t value ){
    uint8_t buffer[ 16 ] = {0};
    uint16_t crc_calc = 0;

    buffer[ 0 ] = 0x01;   // device ID
    buffer[ 1 ] = 0x06;   // functional code for "write single register"

    buffer[ 2 ] = ( address_index >> 8 ) & 0xFF;
    buffer[ 3 ] = address_index & 0xFF;
    buffer[ 4 ] = ( value >> 8 ) & 0xFF;
    buffer[ 5 ] = value & 0xFF;

    crc_calc = crc16( buffer, 6 );
    buffer[ 6 ] = crc_calc & 0xFF;
    buffer[ 7 ] = ( crc_calc >> 8 ) & 0xFF;

    uart_write_bytes( UART_NUM_2, buffer, 8 );
}

void modbus_read_buffer( uint8_t *buffer, uint8_t size ){
    uart_read_bytes( UART_NUM_2, buffer, size, read_timeout );
}

// ============ EXTERNAL FUNCTIONS ============

esp_err_t xts1_write_register( uint16_t address_index, uint16_t value ){
    esp_err_t operation = ESP_OK;
    uint8_t wait = 0;

    if( ( address_index == 0 ) ||
        ( address_index == 1 ) ||
        ( address_index == 2 ) ||
        ( address_index == 3 ) ||
        ( address_index == 4 ) ||
        ( address_index == 5 ) ||
        ( address_index == 6 ) ||
        ( address_index == 7 ) ||
        ( address_index == 64 ) ||
        ( address_index == 65 ) ||
        ( address_index == 66 ) ||
        ( address_index == 86 ) ||
        ( address_index == 87 )
        ){
        modbus_write_register_command( address_index, value );
        while( ( wait < TIMEOUT ) && ( modbus_check_buffer() != 8 ) ){
            wait++;
            vTaskDelay( 1 );
        }
        modbus_flush();
        if( wait == TIMEOUT )
            operation = ESP_ERR_TIMEOUT;
    }
    else
        operation = ESP_ERR_INVALID_ARG;

    return operation;
}

esp_err_t xts1_read_register( uint16_t address_index, uint16_t *value ){
    esp_err_t operation = ESP_OK;
    uint8_t wait = 0;
    uint8_t buffer[ 7 ] = {0};

    if( ( address_index == 0 ) ||
        ( address_index == 1 ) ||
        ( address_index == 2 ) ||
        ( address_index == 3 ) ||
        ( address_index == 4 ) ||
        ( address_index == 5 ) ||
        ( address_index == 6 ) ||
        ( address_index == 7 ) ||
        ( address_index == 64 ) ||
        ( address_index == 65 ) ||
        ( address_index == 66 ) ||
        ( address_index == 86 ) ||
        ( address_index == 87 )
        )
        modbus_read_register_command( address_index, 1, 0x03 );
    else{
        if( ( address_index == 22 ) ||
            ( address_index == 23 ) ||
            ( address_index == 24 ) ||
            ( address_index == 25 ) ||
            ( address_index == 26 ) ||
            ( address_index == 59 ) ||
            ( address_index == 60 ) ||
            ( address_index == 61 )
            ){
            modbus_read_register_command( address_index, 1, 0x04 );
        }
        else{
            operation = ESP_ERR_INVALID_ARG;
        }
    }

    if( operation != ESP_ERR_INVALID_ARG ){
        while( ( wait < TIMEOUT ) && ( modbus_check_buffer() != 7 ) ){
            wait++;
            vTaskDelay( 1 );
        }
        if( wait == TIMEOUT )
            operation = ESP_ERR_TIMEOUT;
        else{
            modbus_read_buffer( buffer, 7 );
            *value = ( buffer[ 3 ] << 8 ) + buffer[  4 ];
        }
    }

    return operation;
}

esp_err_t xts1_sys_error( uint32_t *value ){ 
    esp_err_t operation = ESP_OK;
    uint8_t wait = 0;
    uint8_t buffer[ 9 ] = {0};

    modbus_read_register_command( 16, 2, 0x04 );
    while( ( wait < TIMEOUT ) && ( modbus_check_buffer() != 9 ) ){
        wait++;
        vTaskDelay( 1 );
    }
    if( wait == TIMEOUT )
        operation = ESP_ERR_TIMEOUT;
    else{
        modbus_read_buffer( buffer, 9 );
        *value = ( buffer[ 3 ] << 24 ) + ( buffer[ 4 ] << 16 ) + ( buffer[ 5 ] << 8 ) + buffer[ 6 ];
    }

    return operation;
}

/* Will return distance in mm or error code:
    -13: overexposure
    -12: no object detected
    -11: abnormal TOF image
    -10: abnormal temperature image
    -9: abnormal grey scale image
    -8: reserve
    -7: signal too weak
    -6: signal too strong
    -5: reserve
    -4: sample data below min value
    -3: sample data beyond max value
    -2: pixel saturation
    -1: SPI communication error
*/
uint16_t xts1_measure_distance(){
    uint16_t value = 0;

    ESP_ERROR_CHECK( xts1_read_register( 23, &value ) );

    return value;
}

void xts1_setup(){
    uart_config_t configs = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK( uart_set_pin( UART_NUM_2, 4, 5, -1, -1 ) );
    ESP_ERROR_CHECK( uart_param_config( UART_NUM_2, &configs ) );
    ESP_ERROR_CHECK( uart_driver_install( UART_NUM_2, 2048, 0, 0, NULL, 0 ) );

    ESP_ERROR_CHECK( xts1_write_register( 66, 20 ) );       // measurement period of 20ms (the minimum, for the maximum sample rate is 50Hz)
    ESP_ERROR_CHECK( xts1_write_register( 65, 0x0101 ) );   // active measurement (by default), median filter activated
    ESP_ERROR_CHECK( xts1_write_register( 3, 1 ) );         // start active measurement
    ESP_LOGI( TAG, "XT-S1 Modbus Serial  initialized" );
}
     