#ifndef __XTS1_H
#define __XTS1_H

#include <stdio.h>
#include <driver/uart.h>
#include <esp_err.h>
#include <esp_log.h>
#include <stdbool.h>
#include <stdint.h>

void xts1_setup();

esp_err_t xts1_write_register( uint16_t address_index, uint16_t value );

esp_err_t xts1_read_register( uint16_t address_index, uint16_t *value );

esp_err_t xts1_sys_error( uint32_t *value );

uint16_t xts1_measure_distance();

#endif /* __XTS1_H */