//should be defined before including delays
#define F_CPU 18432000UL

#include <avr/io.h>
#include <avr/interrupt.h>


#define MESSAGEID_INDEX 0

#define ORDER_ID 0b00000101

//message 1
#define IRSIGNATURE_INDEX 3
#define IRSENSOR_INDEX 6
#define TAPESENSOR1_INDEX 7
#define TAPESENSOR2_INDEX 8

//message 2
#define ULTRASONICSENSOR1_INDEX 3
#define LASER_INDEX 8

//message 3
#define ULTRASONICSENSOR2_INDEX 3

//message 4
#define LOWERBITSGYRO_INDEX 3

//message 5
#define HIGHERBITSGYRO_INDEX 3

//message 6
#define ORDER_INDEX 3


//UBRR constants for UART
const uint8_t UBRR_SENSOR_MALSOKNING = 119;
const uint8_t UBRR_STYR_MALSOKNING = 9;

//sensor unit constants
const unsigned NUMBER_OF_ADC_SENSORS = 3;
const uint8_t NUMBER_OF_MESSAGES = 5;