/*
 * Styrenheten
 * 
 * TSEA29 Group 15
 * LIU
 */ 

#include "../../Robotdefinitions.h"

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
void Init();
void InitPWM();
void InitLEDs();
void InitIRSender();
void InitUART();
void IR_sender();
void MoveBackwards(int speed);
void SendUART();

#define LOW 6
#define PAUSE 6
#define HEADER 24
#define HIGH 12
#define ENDPAUSE 24

// VARIABLES
int IRdutyCycle = 240;
int IRperiod = 480;

int pauseTimes[] = {HEADER, PAUSE, HIGH, PAUSE, HIGH, PAUSE, LOW, ENDPAUSE};
int ptIndex = 0;
int ctr = 0;

bool isHigh = true;


int period = 4700; // Period time
int dutyCycle = period* 0.5; // 50% duty cycle to start

// Health stuff
int health = 3;

// IR-sender
bool IRisActivive = true;

uint8_t currentOrder = 0;

//initial values are set to their IDs from design spec. (these bits are never changed)
uint8_t message1 = 0;
uint8_t message2 = 1;
uint8_t message3 = 2;
uint8_t message4 = 3;
uint8_t message5 = 4;
uint8_t message6 = 5;

uint8_t messageNumber = 1;

int main(void){
	DDRB = 0b11111111;

	//Grace time for bluetooth timer prescale /1024
	TCCR2B |= (1<<CS20) | (1<<CS22);
	//enable global interrupts
	sei();
	
	// Call all Init functions in this module
	Init();
	
	waitForActivation();
	//StopMove(); //temp
	//PORTD &= ~(1<<PIND4);
	

    while(1)
    {

		//Do command
		cli();
		uint8_t snapshotOrder = currentOrder;
		sei();
		
		//Send UART to bluetooth without DC
 		if(TCNT2 >= UART_BLUETOOTH_GRACE_PERIOD){
			//LED_PORT ^= (1 << INVISIBLE_LED_PIN);
 			SendUART();
 			TCNT2 = 0;	
 		}

		
		//Reset order so that its not executed more than once
		currentOrder = DO_NOTHING;

		//snapshotOrder = TURN_RIGHT;
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
			case ACTIVATE_LASER_AND_TURN_RIGHT:
				ActivateLaser();
				TurnRight(ROTATION_SPEED);
				break;
				
			case MOVE_BACKWARDS:
				MoveBackwards(MOVEMENT_SPEED);			
				break;
				
			case RESET_SE:
				ResetSE();
				break;
			
			default:
				// Error
				PORTB |= (1<<PINB4);
				break;
		}
		
		// IR-sender
		if (IRisActivive) {
			IR_sender();
		}
		
		
		
    }
}

// Code that send out our signature
void IR_sender() {
	if (isHigh) {
		OCR3A = ICR3 - IRdutyCycle; // duty cycle on 50% of length 26 for PINB6
	}
	else {
		OCR3A = ICR3; // duty cycle on 50% of length 26 for PINB6
	}
	
	if (TCNT0 > 225) {
		ctr++;
		TCNT0 = 0;
		
	}
	
	if (ctr == pauseTimes[ptIndex]) {
		ctr = 0;
		ptIndex++;
		
		if (ptIndex == 8) {
			ptIndex = 0;
		}
		isHigh = !isHigh;			
	}
}

// Reset all necessary data
void ResetSE() {
	// Reset health
	health = 3;
	
	// Make sure that we dont move when we have restarted this module
	MoveForward(0);
	
	// Set all heath LEDs activate
	LED_PORT |= (1<<LED1_PIN) | (1<<LED2_PIN) | (1<<LED3_PIN);
}

// Set doutyCycle for the PWM pins
void SetPWM() {
	OCR1A = ICR1 - dutyCycle; // duty cycle on "dutyCycle" of length "period" for PD5
	OCR1B = ICR1 - dutyCycle; // duty cycle on "dutyCycle" of length "period" for PD4
}

// Calls all Init functions
void Init() {
	InitUART();
	InitPWM();
	InitLEDs();
	InitIRSender();
}

void InitUART() {

	//#UART INITS#//

	//initiate UART målsökning to styr
	//set baud rate
	//115200
	uint16_t UBRR_val = UBRR_STYR_MALSOKNING;
	UBRR0H = (unsigned char) (UBRR_val >> 8);
	UBRR0L = (unsigned char) UBRR_val;

	//enable receive + set frame 8 bits
	UCSR0B = (1<<RXEN0);
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
	
	//enable receive interrupt
	UCSR0B |= (1<<RXCIE0);
	
	
	//FROM STYR TO BLUETOOTH
	//initiate UART målsökning to styr
	//set baud rate
	//115200
	uint16_t UBRR_val1 = UBRR_STYR_MALSOKNING;
	UBRR1H = (unsigned char) (UBRR_val1 >> 8);
	UBRR1L = (unsigned char) UBRR_val1;
		
	//enable transmit + set frame 8 bits
	UCSR1B = (1<<TXEN1);
	UCSR1C = (1<<UCSZ10) | (1<<UCSZ11);
	//#UART INITS END#//
}

// Set all LED pins as output and light them up!
void InitLEDs() {
	DDRB |= (1<<LED1_PIN) | (1<<LED2_PIN) | (1<<LED3_PIN) | (1<<INVISIBLE_LED_PIN) | (1<<LASER_LED_PIN);
	// Set all heath LEDs activate
	LED_PORT |= (1<<LED1_PIN) | (1<<LED2_PIN) | (1<<LED3_PIN);
}

// Setup of PWM and DIR
void InitPWM() {
	// PWM setup
	TCCR1A |= (1<<WGM11) | (1<<COM1A1) | (1<<COM1A0) | (1<<COM1B0) | (1<<COM1B1);
 	TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS10);


	ICR1 = period;
	
	// make sure motor is off.
	OCR1A = ICR1;
	OCR1B = ICR1;
	
	// DIR setup
	
	DDRD |= (1<<PWM1) | (1<<PWM2);
	DDRB |= (1<<DIR1);
	DDRA |= (1<<DIR2);
}


void InitIRSender() {
	DDRB|= (1<<PINB6);
	TCCR3A |= 1<<WGM31 | 1<<COM3A1 | 1<<COM3A0 |1<<COM3B0 | 1<<COM3B1;
	TCCR3B |= 1<<WGM32 | 1<<WGM33 | 1<<CS30;
	
	ICR3 = IRperiod;		// period length in us
	OCR3A = ICR3;
	TCCR0B |= 1<<CS01;	// Starta 8-bit ctr
}

// Set PWM1 and PWM2 to HIGH(set dutyCycle)
// Set DIR1 and DIR2 to low
void MoveForward(int speed) {
	dutyCycle = speed;
	SetPWM();
	PORTB |= (1<<DIR1);
	PORTA |= (1<<DIR2);
}

void MoveBackwards(int speed) {
	dutyCycle = speed;
	SetPWM();
	PORTB &= ~(1<<DIR1);
	PORTA &= ~(1<<DIR2);
}

// Set PWM1 and PWM2 to HIGH(set dutyCycle)
// Set DIR1 to LOW and DIR2 to HIGH
void TurnLeft(int speed) {
	dutyCycle = speed;
	SetPWM();
	PORTB &= ~(1<<DIR1);
	PORTA |= (1<<DIR2);

}

// Set PWM1 and PWM2 to HIGH(set dutyCycle)
// Set DIR2 to LOW and DIR1 to HIGH
void TurnRight(int speed) {
	dutyCycle = speed;
	SetPWM();
	PORTA &= ~(1<<DIR2);
	PORTB |= (1<<DIR1);
}

// Activates the laser pointer and the Laser lED
void ActivateLaser() {
	LASER_PORT |= (1<<LASER_PIN);
}

// Deactivates the laser pointer and the Laser LED
void DeactivateLaser() {
	LASER_PORT &= ~(1<<LASER_PIN);
}

// Turns the IR-sender off (invisible) and turn on Invisible LED
void TurnOffIRSignature() {
	IRisActivive = false;
	LED_PORT |= (1 << INVISIBLE_LED_PIN);
}

// Turns the IR-sender on (not invisible) and turn off Invisible LED
void TurnOnIRSignature() {
	IRisActivive = true;
	LED_PORT &= ~(1 << INVISIBLE_LED_PIN);

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

//UART ISR which sets the "currentOrder" variable
ISR(USART0_RX_vect){
	uint8_t snapbuffer = UDR0;
	uint8_t messageID = snapbuffer & 0x07; //Mask out message ID
	
	//Mux through messages
	switch(messageID){
		case 0:
			message1 = snapbuffer;
			break;
		case 1:
			message2 = snapbuffer;
			break;
		case 2:
			message3 = snapbuffer;
			break;
		case 3:
			message4 = snapbuffer;
			break;
		case 4:
			message5 = snapbuffer;
			break;
		case 5:
			message6 = snapbuffer;
			break;
	}
	//only look at ORDERS
	if(messageID == ORDER_ID){
		currentOrder = (snapbuffer>>3) & 0b00011111; //Mask out the order
	}
}

void SendUART() {
	//check if transmit buffer is empty
	//(UCSR1A & (1<<TXC1)) &&
	//_delay_us(300);
	if((UCSR1A & (1<<UDRE1))){
		//mux through messages
		//may need to disable interrupts
		switch(messageNumber){
			case 1:
			UDR1 = message1;
			break;
			case 2:
			UDR1 = message2;
			break;
			case 3:
			UDR1 = message3;
			break;
			case 4:
			UDR1 = message4;
			break;
			case 5:
			UDR1 = message5;
			break;
			case 6:
				if(currentOrder != DO_NOTHING){ //Only send order if something is to be done
					message6 &= 0b00000111; //reset everything except message ID
					UDR1 = (currentOrder<<3) | message6;
					//nextOrder = 0;
				}
			break;
			default:
			//
			//PORTC |= (1 << PINC0);
			//_delay_us(300);
			//PORTC &= ~(1 << PINC0);
			break;
		}
		//next mux
		messageNumber++;
		if(messageNumber>NUMBER_OF_MESSAGES+1) messageNumber=1;
		//UCSR1A |= (1<<TXC1);
		//_delay_us(300);
	}
	
}