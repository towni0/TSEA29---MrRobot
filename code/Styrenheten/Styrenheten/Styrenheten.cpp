/*
 * Styrenheten
 * 
 * TSEA29 Group 15
 * LIU
 */ 

#include "../../Robotdefinitions.h"
#include "../../queue.h"

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
#define ENDPAUSE 98

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
	//Grace time for bluetooth timer prescaler /1024
	TCCR2B |= (1<<CS20) | (1<<CS22);
	
	// Call all Init functions in this module
	Init();
	
    //init queue where we keep orders
	queue_init(&orderQueue);
	
	//enable global interrupts
	sei();
	
	waitForActivation();
	
    while(1)
    {
        //Send UART to bluetooth without DC, we only send UART after a certain graceperiod because no other fix was able to be found to this problem
 		if(TCNT2 >= UART_BLUETOOTH_GRACE_PERIOD){
 			SendUART();
 			TCNT2 = 0;	
 		}

		//Snapshot order data
		cli();
		uint8_t snapshotOrder = currentOrder;
		sei();
		
		//Reset order so that its not executed more than once
		currentOrder = DO_NOTHING;

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
				
			case MOVE_FORWARD_AND_TURN_INVISIBLE_AND_DEC_LIFE_LED:
				MoveForward(MOVEMENT_SPEED);
				TurnOffIRSignature();
				DecrementLEDLives();
				break;
				
			case RESET_SE:
				ResetSE();
				break;
		}
		
		// IR-sender
		if (IRisActivive) {
			IR_sender();
		}
    }
}

/*
	Sends out our IR-signature.
*/
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

/*
	Reset all necessary data
*/
void ResetSE() {
	// Reset health
	health = 3;

	// Set all health LEDs activate
	LED_PORT |= (1<<LED1_PIN) | (1<<LED2_PIN) | (1<<LED3_PIN);
}

/*
	Sets dutyCycle for the PWM pins
*/
void SetPWM() {
	OCR1A = ICR1 - dutyCycle; // duty cycle on "dutyCycle" of length "period" for PD5
	OCR1B = ICR1 - dutyCycle; // duty cycle on "dutyCycle" of length "period" for PD4
}

/*
	Calls all init functions.
*/
void Init() {
	//Activate pull-up for unused pins to reduce stress on processor
	DDRA = 0;
	PORTA = 0b11111110;
	DDRB = 0;
	PORTB = 0b10001110;
	DDRC = 0;
	PORTC = 0b00111100;
	DDRD = 0;
	PORTD = 0b01110000;
	
	InitUART();
	InitPWM();
	InitLEDs();
	InitIRSender();
}

/*
	Initiates UART.
*/
void InitUART() {
	//initiate UART målsökning to styr
	//set baud rate 115200
	uint16_t UBRR_val = UBRR_STYR_MALSOKNING;
	UBRR0H = (unsigned char) (UBRR_val >> 8);
	UBRR0L = (unsigned char) UBRR_val;

	//enable receive + set frame 8 bits
	UCSR0B = (1<<RXEN0);
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
	
	//enable receive interrupt
	UCSR0B |= (1<<RXCIE0);
	
	//FROM STYR TO BLUETOOTH
	//set baud rate 115200
	uint16_t UBRR_val1 = UBRR_STYR_MALSOKNING;
	UBRR1H = (unsigned char) (UBRR_val1 >> 8);
	UBRR1L = (unsigned char) UBRR_val1;
		
	//enable transmit + set frame 8 bits
	UCSR1B = (1<<TXEN1);
	UCSR1C = (1<<UCSZ10) | (1<<UCSZ11);
	//#UART INITS END#//
}

/*
	Sets all LED pins as output and lights them up!
*/
void InitLEDs() {
	DDRB |= (1<<INVISIBLE_LED_PIN) | (1<<LASER_PIN);
	DDRC |= (1<<LED1_PIN) | (1<<LED2_PIN) | (1<<LED3_PIN) | (1<<LASER_LED_PIN);
	// Turn on all LEDs
	LED_PORT |= (1<<LED1_PIN) | (1<<LED2_PIN) | (1<<LED3_PIN);
}

/*
	Setup of PWM and DIR.
*/
void InitPWM() {
	// PWM setup
	TCCR1A |= (1<<WGM11) | (1<<COM1A1) | (1<<COM1A0) | (1<<COM1B0) | (1<<COM1B1);
 	TCCR1B |= (1<<WGM12) | (1<<WGM13) | (1<<CS10);

	ICR1 = period;
	
	// Make sure motor is off.
	OCR1A = ICR1;
	OCR1B = ICR1;
	
	// DIR setup
	DDRD |= (1<<PWM1) | (1<<PWM2);
	DDRB |= (1<<DIR1);
	DDRA |= (1<<DIR2);
}

/*
	Initiates the IR sender.
*/
void InitIRSender() {
	DDRB|= (1<<PINB6);
	TCCR3A |= 1<<WGM31 | 1<<COM3A1 | 1<<COM3A0 |1<<COM3B0 | 1<<COM3B1;
	TCCR3B |= 1<<WGM32 | 1<<WGM33 | 1<<CS30;
	
	ICR3 = IRperiod; // period length in us
	OCR3A = ICR3;
	TCCR0B |= 1<<CS01; // Start 8-bit ctr
}

/*
	Set PWM1 and PWM2 to HIGH(set dutyCycle)
	Set DIR1 and DIR2 to low
*/
void MoveForward(int speed) {
	dutyCycle = speed;
	SetPWM();
	PORTB |= (1<<DIR1);
	PORTA |= (1<<DIR2);
}

/*
	Move the robot backwards.
*/
void MoveBackwards(int speed) {
	dutyCycle = speed;
	SetPWM();
	PORTB &= ~(1<<DIR1);
	PORTA &= ~(1<<DIR2);
}

/*
	Set PWM1 and PWM2 to HIGH(set dutyCycle)
	Set DIR1 to LOW and DIR2 to HIGH
*/
void TurnLeft(int speed) {
	dutyCycle = speed;
	SetPWM();
	PORTB &= ~(1<<DIR1);
	PORTA |= (1<<DIR2);

}

/*
	Sets PWM1 and PWM2 to HIGH(set dutyCycle)
	Sets DIR2 to LOW and DIR1 to HIGH
*/
void TurnRight(int speed) {
	dutyCycle = speed;
	SetPWM();
	PORTA &= ~(1<<DIR2);
	PORTB |= (1<<DIR1);
}

/*
	Activates the laser pointer and the Laser LED.
*/
void ActivateLaser() {
	LASER_PORT |= (1<<LASER_PIN);
	LED_PORT |= (1<<LASER_LED_PIN);
}

/*
	Deactivates the laser pointer and the Laser LED.
*/
void DeactivateLaser() {
	LASER_PORT &= ~(1<<LASER_PIN);
	LED_PORT &= ~(1<<LASER_LED_PIN);
}

/*
	Turns the IR-sender off (invisible) and turn on Invisible LED.
*/
void TurnOffIRSignature() {
	IRisActivive = false;
	PORTB |= (1 << INVISIBLE_LED_PIN);
}

/*
	Turns the IR-sender on (not invisible) and turn off Invisible LED.
*/
void TurnOnIRSignature() {
	IRisActivive = true;
	PORTB &= ~(1 << INVISIBLE_LED_PIN);

}

/*
	Decrement the amount of lives we have (show on less LED).
*/
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

/*
	Sets PWM1 and PWM2 to LOW(dutyCycle == 0)
*/
void StopMove() {
	dutyCycle = 0;
	SetPWM();
}

/*
	Interrupt function for receiving UART data.
*/
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
		cli();
        //we put order in our orderqueue
		enqueue(snapbuffer, &orderQueue);
		sei();
	}
}

/*
	Sends messages over UART.
*/
void SendUART() {
	//check if transmit buffer is empty
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
				cli();
                //if queue is not empty we send order out and dequeue
				if((orderQueue.front != 0)){
					UDR1 = orderQueue.front->orderdata;
					dequeue(&orderQueue);
				}
				sei();
				break;
		}
		//next mux
		messageNumber++;
		if(messageNumber>NUMBER_OF_MESSAGES+1) messageNumber=1;
	}
}
