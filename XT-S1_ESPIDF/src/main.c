/* IMPORTANT INFO ABOUT DEMO:

    CONFIG_FREERTOS_HZ=100 changed to CONFIG_FREERTOS_HZ=1000 in the sdkconfig file

    HARDWARE CONFI:
    ESP32-S3 pin    |   XT-S1 pin
    4               |   RX
    5               |   TX
    +5V             |   +5V
    GND             |   GND
*/

// ========== IDF LIBRARIES ==========

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// ========== INTERNAL LIBRARIES ==========

#include "delay.h"
#include "xts1.h"

// ========== DEFINITIONS ==========

#define DATASET 50

// ========== MAIN PROGRAM (DEMO) ==========

void app_main(){
    xts1_setup();

    // all the registers you could read/write are 0, 1, 2, 3, 4, 5, 6, 7, 64, 65, 66 ,86, 87, 22, 23, 24, 25, 26, 59, 60, 61
    char *distance_diag[ 13 ] = { // possible error results from measurements
        "ERROR -13: overexposure",
        "ERROR -12: no object detected",
        "ERROR -11: abnormal TOF image",
        "ERROR -10: abnormal temperature image",
        "ERROR -9: abnormal grey scale image",
        "ERROR -8: reserve",
        "ERROR -7: signal too weak",
        "ERROR -6: signal too strong",
        "ERROR -5: reserve",
        "ERROR -4: sample data below min value",
        "ERROR -3: sample data beyond max value",
        "ERROR -2: pixel saturation",
        "ERROR -1: SPI communication error",
    };
    uint8_t i;
    // statistical data variables
    int16_t measurements[ DATASET ] = {0}, minimum, maximum;
    float variance, standard_deviation, mean;

    printf( "\n" );
    while( true ){
        minimum = 30000; // the maximum distance in mm
        maximum = 0;
        mean = 0;
        variance = 0;
        standard_deviation = 0;
        printf( "starting measurements.\n" );
        for( i = 0; i < DATASET; i++ ){
            // acquire data and check for error messages
            measurements[ i ] = 0;
            while( measurements[ i ] < 1 ){ // distance register returned error message
                measurements[ i ] = xts1_measure_distance(); // actually reads sensor
                if( measurements[ i ] < 0 )
                    printf( "  --> %s\n", distance_diag[ measurements[ i ] + 13 ] );
            }
            // check for minimum
            if( measurements[ i ] < minimum )
                minimum = measurements[ i ];
            // check for maximum
            if( measurements[ i ] > maximum )
                maximum = measurements[ i ];
            delay_milli( 20 ); // maximum sample rate = 50Hz
        }
        // calculating mean
        for( i = 0; i < DATASET; i++ )
            mean += measurements[ i ];
        mean /= DATASET;
        // calculating variance
        for( i = 0; i < DATASET; i++ )
            variance +=  pow( measurements[ i ] - mean, 2 );
        variance /= DATASET - 1; // -1 due to Bessel's Correction
        // calculating standard deviation
        standard_deviation = sqrt( variance );

        printf( " minimum = %5d \n", minimum );
        printf( " maximum = %5d \n", maximum );
        printf( " mean = %f \n", mean );
        printf( " variance = %f \n", variance );
        printf( " standard deviation = %f \n\n", standard_deviation );
    }
}