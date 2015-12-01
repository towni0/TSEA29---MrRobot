//should be defined before including delays
#define F_CPU 18432000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#define MESSAGEID_INDEX 0

#define ORDER_ID 0b00000101

#define LASER_AKTIVERA_PORT 4

//left 800
uint16_t tape1Threshold = 650; //Limit that determines what's tape and what's not
uint16_t tape1CurrentValue = 0;
//right 500
uint16_t tape2Threshold = 400; //Limit that determines what's tape and what's not
uint16_t tape2CurrentValue = 0;

const uint8_t TAPE_ERROR_MARGIN = 200;

//message 1
#define IRSIGNATURE_INDEX 3
#define LASER_INDEX 6
#define IRSENSOR_INDEX 7

//message 2
#define ULTRASONICSENSOR1_INDEX 3

//message 3
#define ULTRASONICSENSOR2_INDEX 3

//message 4
#define LOWERBITSGYRO_INDEX 3

//message 5
#define HIGHERBITSGYRO_INDEX 3
#define TAPESENSOR1_INDEX 6
#define TAPESENSOR2_INDEX 7

//message 6
#define ORDER_INDEX 3

//Ultrasonic
#define ECHO_PIN1 PINB0
#define PULSE_TRIGGER_PIN1 PINB1

#define ECHO_PIN2 PINB2
#define PULSE_TRIGGER_PIN2 PINB3

#define MICRO_SEC_PER_TICK 42
// ## Styrenhet definitions ##

#define PWM1 PIND5
#define PWM2 PIND4
#define DIR1 PINB5
#define DIR2 PINA0
#define DIR_PWM_PORT PORTD

#define LASER_PIN PINB4
#define LASER_PORT PORTB

// #define IR_PWM PINB5
// #define IR_PORT PINB

#define LED1_PIN PINB0
#define LED2_PIN PINB1
#define LED3_PIN PINB2
#define INVISIBLE_LED_PIN PINB3
#define LASER_LED_PIN PINB4
#define LED_PORT PORTB

#define SHOOT_SWEEP_DEGREES 46

//Orders
#define DO_NOTHING 						0
#define MOVE_FORWARD 					1
#define TURN_LEFT 						2
#define TURN_RIGHT 						3
#define ACTIVATE_LASER 					4
#define DEACTIVATE_LASER 				5
#define TURN_OFF_IR_SIG					6
#define TURN_ON_IR_SIG 					7
#define STOP_MOVING 					8
#define DECREMENT_LED_LIVES				9
#define TURN_INVISIBLE_AND_DEC_LIFE_LED 10
#define ACTIVATE_LASER_AND_TURN_RIGHT 	11
#define MOVE_BACKWARDS				 	12


#define RESET_SE						20
										
#define MOVEMENT_SPEED					1175
//1175
#define ROTATION_SPEED					2350


//## Målsökning definitions ##

#define HighSpeed 255
#define MediumSpeed 128
#define LowSpeed 64
#define ANGULAR_RATE_IDLE 128


//Rotation offset
// offset seems to be linear with y(t) = x*333.333 + 2500
#define OFFSET_ROTATE_22POINT5	10000
#define OFFSET_ROTATE_45		17000
#define OFFSET_ROTATE_90		33000
#define OFFSET_ROTATE_135		48000
#define OFFSET_ROTATE_180		62000

//UBRR constants for UART
const uint8_t UBRR_SENSOR_MALSOKNING = 119;
const uint8_t UBRR_STYR_MALSOKNING = 9;

//sensor unit constants
const unsigned NUMBER_OF_ADC_SENSORS = 3;
const uint8_t NUMBER_OF_MESSAGES = 5;


//Wait for press on activation button
void waitForActivation(){
	DDRD &= ~(1<<PIND7); //Aktiveringsknapp (in)
	_delay_ms(10);
	while((PIND>>PIND7) == 0){
		//Do nothing, wait for activation
	}
	return;
}


/* NEW SCHEME
Meddelande 1:
Bit 0-2:	Meddelande ID (000)
Bit 3-5:	IR-signaturen
Bit 6:      Laser (1 för träff)
Bit 7:		Aktiv IR-signatur (robot framför oss)

Meddelande 2:
Bit 0-2:	Meddelande ID (001)
Bit 3-7:	Främre avståndssensorn (ca 1 dm precision)

Meddelande 3:
Bit 0-2:	Meddelande ID (010)
Bit 3-7:	Bakre avståndssensorn (ca 1 dm precision)

Meddelande 4:
Bit 0-2:	Meddelande ID (011)
Bit 3-7:	5 LSB Gyro (grader rotatation)

Meddelande 5:
Bit 0-2:	Meddelande ID (100)
Bit 3-5:	3 MSB Gyro (grader rotatation)
Bit 6:		Tejpsensor 1 (vänster, 1 för tejp)
Bit 7:		Tejpsensor 2 (höger, 1 för tejp)

Meddelande 6:
Bit 0-2:	Meddelande ID (101)(ORDER)
Bit 3-7		ORDERID
*/