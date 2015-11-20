/*
 * Styrenhet.c
 *
 * Created: 11/11/2015 5:47:39 PM
 *  Author: joast229
 */ 

#include "../../Robotdefinitions.h"


#define PWM1 PIND5
#define PWM2 PIND4
#define DIR1 PIND3
#define DIR2 PIND2
#define DIR_PWM_PORT PORTD

#define LASER_PIN PIND1
#define LASER_PORT PORTD

#define IR_PWM PINB5
#define IR_PORT PINB

//Orders
#define DO_NOTHING 			0
#define MOVE_FORWARD 		1
#define TURN_LEFT 			2
#define TURN_RIGHT 			3
#define ACTIVATE_LASER 		4
#define DEACTIVATE_LASER 	5
#define TURN_OFF_IR_SIG		6
#define TURN_ON_IR_SIG 		7
#define STOP_MOVING 		8
#define DECREMENT_LED_LIVES	9

// VARIABLES
int period = 255; // Period time
int dutyCycle = period* 0.5; // 50% duty cycle to start

uint8_t currentOrder = 0;

// Function headers
void InitModule();
void SetPWM();

void MoveForward( int speed);
void TurnLeft(int speed);
void TurnRight(int speed);
void ActivateLaser();
void DeactivateLaser();
void TurnInvisible();
void TurnVisible();
void StopMove();



int main(void){

	DDRB = 0b11111111;

	//enable global interupts
	sei();

	//#UART INITS#//

	//initiate UART målsökning to styr
	//set baudrate
	//115200
	uint16_t UBRR_val = UBRR_STYR_MALSOKNING;
	UBRR0H = (unsigned char) (UBRR_val >> 8);
	UBRR0L = (unsigned char) UBRR_val;

	//enable receive + set frame 9 bits
	UCSR0B = (1<<RXEN0) | (1<<UCSZ02);
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
	
	//enable receive interrupt
	UCSR0B |= (1<<RXCIE0);
	//#UART INITS END#//

    while(1)
    {
		if(currentOrder == 0x41){
			PORTB |= (1<<PINB4);
		}
		else{
			PORTB &= 0;
		}
         //Do command
		 uint8_t snapshotOrder = currentOrder & 0xF8;
		 snapshotOrder = snapshotOrder >> 3;
		 
		 
		switch (snapshotOrder) {
		case DO_NOTHING:	
			break;
			
		case MOVE_FORWARD:	// 
			return "Move Forward";
			
		case TURN_LEFT:	// 
			return "Turn Left";
			
		case TURN_RIGHT:	// 
			return "Turn Right";
			
		case ACTIVATE_LASER:	// 
			return "Activate Laser";
			
		case DEACTIVATE_LASER:	// 
			return "Deactivate Laser";
			
		case TURN_OFF_IR_SIG:	// 
			return "Turn Off IR Signature";
			
		case TURN_ON_IR_SIG:	// 
			return "Turn On IR Signature";
			
		case STOP_MOVING:	// 
			return "Stop Moving";
			
		case DECREMENT_LED_LIVES:	// 
			return "Decrement LED Lives";		 
			
		default:
			// Error
			break;	
    }
}


void SetPWM() {
	OCR1A = ICR1 - dutyCycle; // duty cycle on "dutyCycle" of length "period" for PD5
	OCR1B = ICR1 - dutyCycle; // duty cycle on "dutyCycle" of length "period" for PD4
}



// Setup of PWM 
void InitPWM() {
	// PWM setup
	TCCR1A |= 1<<WGM11 | 1<<COM1A1 | 1<<COM1A0 |1<<COM1B0 | 1<<COM1B1;
	TCCR1B |= 1<<WGM12 | 1<<WGM13 | 1<<CS10;
	ICR1 = period;

	// DIR setup
	DIR_PWM_PORT |= (1<<DIR1) | (1<<DIR2) | (1<<PWM1) | (1<<PWM2);
}

// Set PWM1 and PWM2 to HIGH(set dutyCycle)
// Set DIR1 and DIR2 to low
void MoveForward(int speed) {
	dutyCycle = speed;
	SetPWM();
	DIR_PWM_PORT &= ~(1<<DIR1);
	DIR_PWM_PORT &= ~(1<<DIR2);
}

// Set PWM1 and PWM2 to HIGH(set dutyCycle)
// Set DIR1 to LOW and DIR2 to HIGH
void TurnLeft(int speed) {
	dutyCycle = speed;
	SetPWM();
	DIR_PWM_PORT &= ~(1<<DIR1);
	DIR_PWM_PORT |= (1<<DIR2);
}

// Set PWM1 and PWM2 to HIGH(set dutyCycle)
// Set DIR2 to LOW and DIR1 to HIGH
void TurnRight(int speed) {
	dutyCycle = speed;
	SetPWM();
	DIR_PWM_PORT &= ~(1<<DIR2);
	DIR_PWM_PORT |= (1<<DIR1);
}

void ActivateLaser() {

}

void TurnInvisible() {

}

// Set PWM1 and PWM2 to LOW(dutyCycle == 0)
void StopMove() {
	dutyCycle = 0;
	SetPWM();
}



ISR(USART0_RX_vect){
	uint8_t messageID = UDR0 & 0x07;
	//only look at ORDERS
	if(messageID == ORDER_ID){
		currentOrder = UDR0;
	}
}