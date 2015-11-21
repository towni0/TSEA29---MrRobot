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

#define LASER_PIN PINB4
#define LASER_PORT PORTB

#define IR_PWM PINB5
#define IR_PORT PINB

#define LED1_PIN PINB0
#define LED2_PIN PINB1
#define LED3_PIN PINB2
#define INVISIBLE_LED_PIN PINB3
#define LASER_LED_PIN PINB4
#define LED_PORT PORTB

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


#define RESET_SE						20


#define MOVEMENT_SPEED 128
#define ROTATION_SPEED 64

// Function headers
void InitModule();
void SetPWM();

void MoveForward( int speed);
void TurnLeft(int speed);
void TurnRight(int speed);
void ActivateLaser();
void DeactivateLaser();
void StopMove();
void TurnOffIRSignature();
void TurnOnIRSignature();
void DecrementLEDLives();
void ResetSE();


// VARIABLES
int period = 255; // Period time
int dutyCycle = period* 0.5; // 50% duty cycle to start

// Health stuff
int health = 3;

// IR-sender
bool IRisActivive = true;



uint8_t currentOrder = 0;




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
	
	// Call all Init functions in this module
	Init();
	
	
	// Activate LOOP HERE

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
			
		case MOVE_FORWARD:
			MoveForward(MOVEMENT_SPEED);
			break;
			
		case TURN_LEFT:	
			TurnLeft(ROTATION_SPEED);
			break;
			
		case TURN_RIGHT:	
			TurnRight(ROTATION_SPEED);
			break;
			
		case ACTIVATE_LASER:	 
			ActivateLaser();
			break;
			
		case DEACTIVATE_LASER:
			DeactivateLaser();
			break;
			
		case TURN_OFF_IR_SIG:	
			TurnOffIRSignature();
			break;
			
		case TURN_ON_IR_SIG:	
			TurnOnIRSignature();
			break;
			
		case STOP_MOVING:	 
			StopMove();
			break;
			
		case DECREMENT_LED_LIVES:
			DecrementLEDLives();
			break;		 
		
		case TURN_INVISIBLE_AND_DEC_LIFE_LED:
			DecrementLEDLives();
			TurnOffIRSignature();
			break;	
		
		case RESET_SE:
			ResetSE();
			break;	
		
		default:
			// Error
			break;	
			
			
			
			
			
    }
	
	
}

// Reset all neccesary data
void ResetSE() {
	// Reset health
	health = 3;
	
	// Make sure that we dont move when we have restarted this module
	MoveForward(0);
	
	// Set all heath LEDs activte
	LED_PORT |= (1<<LED1_PIN) | (1<<LED2_PIN) | (1<<LED3_PIN);
}

// Set doutyCycle for the PWM pins
void SetPWM() {
	OCR1A = ICR1 - dutyCycle; // duty cycle on "dutyCycle" of length "period" for PD5
	OCR1B = ICR1 - dutyCycle; // duty cycle on "dutyCycle" of length "period" for PD4
}

// Calls all Init functions
void Init() {
	InitPWM();
	InitLEDs();
}

// Set all LED pins as output and light them up!
void InitLEDs() {
	DDRB |= (1<<LED1_PIN) | (1<<LED2_PIN) | (1<<LED3_PIN) | (1<<INVISIBLE_LED_PIN) | (1<<LASER_LED_PIN);
	// Set all heath LEDs activte
	LED_PORT |= (1<<LED1_PIN) | (1<<LED2_PIN) | (1<<LED3_PIN);
}

// Setup of PWM and DIR
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

// Activates the laser pointer and the Laser lED
void ActivateLaser() {
	LASER_PORT |= (1<<LASER_PIN);

}

// Deactivates the laser pointer and the Laser LED
void DeactivateLaser() {
	LASER_PORT &= ~(1<<LASER_PIN);
}

// Turns the IR-sender off (invisible)
void TurnOffIRSignature() {
	IRisActivive = false;
}

// Turns the IR-sender on (not invisible)
void TurnOnIRSignature() {
	IRisActivive = true;
}

// Decrement the amount of lives we have (show on less LED)
void DecrementLEDLives() {
	health--;
	
	if (health == 2) {
		LED_PORT &= ~(1<<LED3_PIN);
	}
	
	if (health == 1) {
		LED_PORT &= ~(1<<LED2_PIN);
	}
	
	if (health == 0) {
		LED_PORT &= ~(1<<LED1_PIN);
	}
	
}

// Set PWM1 and PWM2 to LOW(dutyCycle == 0)
void StopMove() {
	dutyCycle = 0;
	SetPWM();
}


// UART ISR which sets the "currentOrder" variable
ISR(USART0_RX_vect){
	uint8_t messageID = UDR0 & 0x07;
	//only look at ORDERS
	if(messageID == ORDER_ID){
		currentOrder = UDR0;
	}
}