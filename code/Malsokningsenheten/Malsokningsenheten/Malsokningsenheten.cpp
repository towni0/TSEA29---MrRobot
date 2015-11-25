/*
 * Målsökningsenheten
 * 
 * TSEA29 Group 15
 * LIU
 */ 


#include "../../Robotdefinitions.h"
#include <avr/io.h>
#include <avr/interrupt.h>
//#include <util/delay.h>

//initial values are set to their IDs from design spec. (these bits are never changed)
uint8_t message1 = 0;
uint8_t message2 = 1;
uint8_t message3 = 2;
uint8_t message4 = 3;
uint8_t message5 = 4;
uint8_t message6 = 5;

//Variables for storing sensor values
volatile uint8_t tapeSensor1 = 0; //1 for tape, 0 for floor
volatile uint8_t tapeSensor2 = 0; //1 for tape, 0 for floor
volatile uint8_t ultraSonicSensor1 = 0; //distance in dm
volatile uint8_t ultraSonicSensor2 = 0; //distance in dm
volatile uint8_t activeIRsignature = 0; //1 if enemy in front
volatile uint8_t IRSignature = 0; //the active IR signature
volatile uint8_t laserSensor = 0; //1 if hit
volatile uint8_t gyro = 0; //Rotation speed

uint8_t nextOrder = 0;

uint8_t testChange = 0;

uint8_t messageNumber = 1;

void testTAPEsensors();

//##### AI #####
// Data
bool canShoot = true;
int health = 3;
bool dead = false;

bool scaning = false;

// Rotation stuff
bool rotating = false;
int currentRotationValue;
int targetRotation;
#define ROTATION_PRIORITY 5

// Laser stuff
#define LASER_TIMER TCCR1B
#define LASER_TIMER_COUNTER TCNT1
int CDCTR = 0;
bool laserActive = false;

#define IR_TIMER TCCR3B
#define IR_TIMER_COUNTER TCNT3

#define ONE_SECOND 18000

// Used to count up to 3
int IRCTR = 0;

void SetPriority(int priority);
void ClearPriority();
void BlinkLEDs();
void WeAreHit();
void Shoot();
void Rotate(int degrees, bool leftTurn);
void StopIRTimer();
void StartIRTimer();
void StartLaserTimer();
void StopLaserTimer();
void Scan();
int Priority(int operation);
int	currentPriority = -1;
bool foundSomethingToDo = false;

int orders[10];

//gyro
uint8_t sampleTimeInSeconds = 0.1; //Since the gyro gives us degrees/sec
uint16_t gyroSum;

int main(void)
{
	DDRB = 0b11111011;
	
	//enable global interrupts
	sei();
	
	//################
	//## UART INITS ##
	//################
	
	//initiate UART målsökning from sensor
	//set baud rate
	uint16_t UBRR_val = UBRR_SENSOR_MALSOKNING;
	UBRR0H = (unsigned char) (UBRR_val >> 8);
	UBRR0L = (unsigned char) UBRR_val;
	//enable receive + set frame 8 bits
	UCSR0B = (1<<RXEN0);
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);

	//enable receive interrupt
	UCSR0B |= (1<<RXCIE0);
	
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
	
	waitForActivation();
	
    while(!dead)
    {
		
		//disable interrupts
		cli();
		
		//############################
		//## Snapshot sensor values ##
		//############################
		//Message 1
		IRSignature = (message1>>IRSIGNATURE_INDEX) & 0b00000111;
		laserSensor = (message1>>LASER_INDEX) & 0b00000001;
		activeIRsignature = (message1>>IRSENSOR_INDEX) & 0b00000001;
		
		//Message 2
		ultraSonicSensor1 = (message2>>ULTRASONICSENSOR1_INDEX) & 0b00011111;
		
		//Message 3
		ultraSonicSensor2 = (message3>>ULTRASONICSENSOR2_INDEX) & 0b00011111;
		
		//Message 4
		gyro = (message4>>LOWERBITSGYRO_INDEX) & 0b00011111; //Low 5 bits
		
		//Message 5
		gyro |= (message5<<2) & 0b11100000; //High 3 bits
		tapeSensor1 = (message5>>TAPESENSOR1_INDEX) & 0b00000001;
		tapeSensor2 = (message5>>TAPESENSOR2_INDEX) & 0b00000001;
		
		//enable interrupts
		sei();
		
		if((PINB>>PINB2) == 0){
			//##################
			//## Tävlingsläge ##
			//##################
			
			// If we are rotating
			if (rotating) {
				// THIS IS JUST PLACEHOLDER, NOT THE ACTUAL CODE, WE NEED TO DO SOMETHING MORE WITH THE GYRO VALUE
				currentRotationValue += gyro;
				if (currentRotationValue >= targetRotation) {
					nextOrder = STOP_MOVING;
				}
			}
		
			// If we are scaning for opponents
			if (scaning) {
				if (ultraSonicSensor1 <= 15 || ultraSonicSensor2 <= 15) {
					nextOrder = STOP_MOVING;
				}
			}
		
			// If something is in front of the robot and the IR-signature is active
			if (ultraSonicSensor1 <= 15 && activeIRsignature) {
				if (canShoot) {
					foundSomethingToDo = true;
					Shoot();
				}
			
			}
		
			if (tapeSensor1 == 1 && tapeSensor2 == 1) {
				Rotate(180, true);
			}
		
			// If the Left line sensor detects tape, turn right
			if(tapeSensor1 == 1){ 
				Rotate(90, false);
			}
		
			// If the Right line sensor detects tape, turn left
			if(tapeSensor2 == 1){ 
				Rotate(90, true);
			}
		
	
			// If the IR-sensor sees an enemy signature ONLY
			if(activeIRsignature == 1 ){
				foundSomethingToDo = true;
				Scan();
			}
		
			// If we are hit
			if(laserSensor == 1){
				foundSomethingToDo = true;
				WeAreHit();
			}
		
			/*
			// If something is in front of the robot, do something
			if(ultraSonicSensor1 == 10){}
	
			// If something is behind the robot, do something else
			if(ultraSonicSensor2 == 10){}		
			*/
		
			// If there is nothing else to do, move forward
			if (!foundSomethingToDo) {
				nextOrder = MOVE_FORWARD;
			}

			// IR timer stuff
			if (IR_TIMER_COUNTER >= ONE_SECOND) {
				IRCTR++;
				if (IRCTR >= 5) {
					StopIRTimer();
					nextOrder = TURN_ON_IR_SIG;
				}
			}
		
			// LASER timer stuff
			if (LASER_TIMER_COUNTER >= ONE_SECOND) {
				CDCTR++;
				if (laserActive) {
					canShoot = false;
					nextOrder = DEACTIVATE_LASER;
					LASER_TIMER_COUNTER = 0;
					laserActive = false;
				
				}
				else {
					if (CDCTR >= 3) {
						canShoot = true;
						StopLaserTimer();
					}
				}
			
			}
		}
		else{
			//##############
			//## Testläge ##
			//##############
		}
	


		//#######################
		//## UART Transmission ##
		//#######################

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
					if(nextOrder != DO_NOTHING){ //Only send order if something is to be done
						message6 &= 0b00000111; //reset everything except message ID
						UDR1 = (nextOrder<<3) | message6;
						nextOrder = 0;
					}
					break;
			}
			//next mux
			messageNumber++;
			if(messageNumber>NUMBER_OF_MESSAGES+1) messageNumber=1;
		}
    }
	
	// Reset the laser timers count variable and start the laser timer (used to get a blinking LEDs)
	StartLaserTimer();
	// This loop blinks the LEDs to show that we are dead
	while (dead) {
		if (LASER_TIMER_COUNTER >= ONE_SECOND) {
			LASER_TIMER_COUNTER = 0;
			CDCTR++;
			nextOrder = DECREMENT_LED_LIVES;
			if (CDCTR == 3) {
				nextOrder = RESET_SE;
				CDCTR = 0;
			}
		}
	}
}

ISR(USART0_RX_vect){
	
	//may need to disable interrupts
	uint8_t buffer = UDR0;
	uint8_t messageID = buffer & 0x07; //Mask out message ID
	
	//Mux through messages
	switch(messageID){
		case 0:
			message1 = buffer;
			break;
		case 1:
			message2 = buffer;
			break;
		case 2:
			message3 = buffer;
			break;
		case 3:
			message4 = buffer;
			break;
		case 4:
			message5 = buffer;
			//message5 = 0b11000100; //Temp test:
			break;
	}
}

//Called once every sampleTimeInSeconds
uint16_t CalcGyro(uint16_t gyroData){
   gyroSum += gyroData;
   return (gyroSum * sampleTimeInSeconds);
}

int Abs(int value) {
	if (value < 0) {
		value *= -1;
	}
	
	return value;
}
	
void SetPriority(int priority) {
	currentPriority = priority;
}

void ClearPriority() {
	currentPriority = -1;
}

void BlinkLEDs() {
	
}

void WeAreHit() {
	health--;
	
	// We are dead
	if (health <= 0) {
		dead = true;
		return;
	}
	
	//SetPriority() // Måste skicka argument!
	nextOrder = TURN_INVISIBLE_AND_DEC_LIFE_LED;
	StartIRTimer();
}

void Shoot() {
	laserActive = true;
	nextOrder = ACTIVATE_LASER;
}


void StartLaserTimer() {
	TCNT1 = 0;
	LASER_TIMER = (1<< CS12) | (1<< CS10);
}

void StopLaserTimer() {
	LASER_TIMER &= ~(1 << CS12);
	LASER_TIMER &= ~(1 << CS10);
	TCNT1 = 0;
}

void StartIRTimer() {
	TCNT3 = 0;
	IR_TIMER = (1<< CS32) | (1<< CS30);
}

void StopIRTimer() {
	IR_TIMER &= ~(1 << CS32);
	IR_TIMER &= ~(1 << CS30);
	TCNT3 = 0;
}

void Rotate(int degrees, bool leftTurn) {
	rotating = true;
	targetRotation = degrees;
	
	if (leftTurn) {
		nextOrder = TURN_LEFT;
	}
	else {
		nextOrder = TURN_RIGHT;
	}
	
}

int Priority(int operation) {
	switch(operation){
		case DO_NOTHING:
			return 0;
		case MOVE_FORWARD:
			return 1;
		case TURN_LEFT:
			return 1;
		case TURN_RIGHT:
			return 1;
		case ACTIVATE_LASER:
			return 2;
		case DEACTIVATE_LASER:
			return 2;
		case TURN_OFF_IR_SIG:
			return 2;
		case TURN_ON_IR_SIG:
			return 2;
		case STOP_MOVING:
			return 10;
		case DECREMENT_LED_LIVES:
			return 20;
		case TURN_INVISIBLE_AND_DEC_LIFE_LED:
			return 21;
		case ACTIVATE_LASER_AND_TURN_RIGHT:
			return 21;
		default :
			return 0;
	}
}



void Scan() {
	scaning = true;
	Rotate(360, true);
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
