#ifndef __DELAY_H
#define __DELAY_H

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <esp_timer.h>
#include <stdbool.h>
#include <stdint.h>

void delay_tick();

void delay_milli( uint16_t period );

void delay_micro( uint32_t microseconds );

#endif /* __DELAY_H */