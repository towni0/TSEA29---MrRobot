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


// Function headers
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
float CalcGyro(int gyroData);
void testTAPEsensors();


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



//##### AI #####

#define LASER_TIMER TCCR1B
#define LASER_TIMER_COUNTER TCNT1
#define IR_TIMER TCCR3B
#define IR_TIMER_COUNTER TCNT3

#define ONE_SECOND 18000

// Data
bool canShoot = true;
int health = 3;
bool dead = false;

bool scaning = false;

// Rotation stuff
bool rotating = false;
int currentRotationValue;
int targetRotation;

// Laser stuff
int coolDownCTR = 0;
bool laserActive = false;

bool laserSensorHit = false;

// Used to count up to 3
int IRCTR = 0;
int	currentPriority = -1;
bool foundSomethingToDo = false;
int orders[10];

//gyro
const float sampleTimeInSeconds = 0.01; //Since the gyro gives us degrees/sec
const int sampleticks = 180 ;
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
	
	//Rotate(90, false);
	nextOrder = MOVE_FORWARD;
	
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
		
		if((PINB>>PINB2) == 0){
			//##################
			//## Tävlingsläge ##
			//##################
			
			// IR timer stuff
			if (IR_TIMER_COUNTER >= ONE_SECOND) {
				IRCTR++;
				if (IRCTR >= 5) {
					StopIRTimer();
					nextOrder = TURN_ON_IR_SIG;
					continue;
				}
			}
		
			// LASER timer stuff
			if (LASER_TIMER_COUNTER >= ONE_SECOND) {
				coolDownCTR++;
				LASER_TIMER_COUNTER = 0;
				
				// Lasers been active for 1 sec, turn it off
				if (coolDownCTR == 1) {
					nextOrder = DEACTIVATE_LASER;					
					laserActive = false;
					continue;
				}
				// cooldown is over
				else if (coolDownCTR == 4) {
					canShoot = true;
					StopLaserTimer();				
				}
			
			}
			
			// If we are hit, we turn invisible and decrement our live pool
			if(laserSensor == 1 && !laserSensorHit ){
				foundSomethingToDo = true;
				laserSensorHit = true;
				WeAreHit();
				continue;
				
			}
			else if (laserSensor == 0 && laserSensorHit ) {
				laserSensorHit = false;
			}
			
			// If both line sensor detects tape and we havn't startet rotating, turn around (180 degrees)
			if ((tapeSensor1 == 1 && tapeSensor2 == 1) && !rotating) {
				Rotate(180, true);
				continue;
			}
		
			// If the Left line sensor detects tape and we havn't startet rotating, turn right
			if((tapeSensor1 == 1) && !rotating){ 
				Rotate(90, false);
				continue;

			}
		
			// If the Right line sensor detects tape and we havn't startet rotating, turn left
			if((tapeSensor2 == 1) && !rotating){ 
				Rotate(90, true);
				continue;
			}
		
			
			
			// If we are rotating
			if (rotating) {
				if(TCNT2 >= sampleticks){
					int angularVelocity = gyro - ANGULAR_RATE_IDLE; 
					if (CalcGyro(abs(angularVelocity)) >= targetRotation) {
						gyroSum = 0;
						rotating = false;
						nextOrder = STOP_MOVING;
						TCCR2B &= ~((1 << CS20) | (1 << CS21) | (1 << CS22));
						
						// If we reached the end of the first turn, turn to the oppisite way
						if (laserActive ) {
							Rotate(SHOOT_SWEEP_DEGREES, true);
						}
						
					}
					//Send how many degrees we have rotated over uart
					message4 &= 0b00000111; //Reset bits
					message4 |= (gyroSum<<LOWERBITSGYRO_INDEX);
					message5 &= 0b11000111; //Reset bits
					message5 |= ((gyroSum>>2) & 0b00111000);
					TCNT2 = 0;
				}
			}
		
		 	// If we are scaning for opponents and we find something within 1,5 meters, stop move, 
			if (scaning && rotating) {
				if (ultraSonicSensor1 <= 15 || ultraSonicSensor2 <= 15) {
					rotating = false;
					scaning = false;
					nextOrder = STOP_MOVING;
					continue;
				}
			}
		
			// If something is in front of the robot and the IR-signature is active
			if (ultraSonicSensor1 <= 15 && activeIRsignature) {
				if (canShoot) {
					foundSomethingToDo = true;
					Shoot();
					continue;
				}
			
			}
			
			if (laserActive) {
				
			}
		
			// If the IR-sensor sees an enemy signature ONLY
			if(activeIRsignature == 1 ){
				foundSomethingToDo = true;
				Scan();
				continue
			}
		 
			 // If there is nothing else to do, move forward, KEEP THIS??
			if (!foundSomethingToDo) {
				nextOrder = MOVE_FORWARD;
			}


		}
		else{
			//##############
			//## Testläge ##
			//##############
			
		
		
			// If the Right line sensor detects tape and we havn't startet rotating, turn left
			if((tapeSensor2 == 1) && !rotating){ 
				nextOrder = STOP_MOVING;
				continue;
			}
			
			
			
			
		}
	



    }
	
	// Reset the laser timers count variable and start the laser timer (used to get a blinking LEDs)
	StartLaserTimer();
	// This loop blinks the LEDs to show that we are dead
	while (dead) {
		if (LASER_TIMER_COUNTER >= ONE_SECOND) {
			LASER_TIMER_COUNTER = 0;
			coolDownCTR++;
			nextOrder = DECREMENT_LED_LIVES;
			if (coolDownCTR == 3) {
				nextOrder = RESET_SE;
				coolDownCTR = 0;
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
			//message4 = buffer;
			break;
		case 4:
			message5 &= 0b00111000; //We set the gyro bits elsewhere
			message5 |= (buffer & 0b11000111);
			//message5 = 0b11000100; //Temp test:
			break;
	}
}

void Scan() {
	scaning = true;
	Rotate(360, true);
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
	//start timer
	TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22);
}

//Called once every sampleTimeInSeconds
float CalcGyro(int gyroData){
   gyroSum += gyroData;
   return (gyroSum * sampleTimeInSeconds);
}

int Abs(int value) {
	if (value < 0) {
		value *= -1;
	}
	
	return value;
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
	canShoot = false;
	rotating = true;
	targetRotation = SHOOT_SWEEP_DEGREES / 2;
	
	//start timer
	TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22);
	
	nextOrder = ACTIVATE_LASER_AND_TURN_RIGHT;
	StartLaserTimer();
	
}


void StartLaserTimer() {
	LASER_TIMER_COUNTER = 0;
	LASER_TIMER = (1<< CS12) | (1<< CS10);
}

void StopLaserTimer() {
	LASER_TIMER &= ~(1 << CS12);
	LASER_TIMER &= ~(1 << CS10);
	LASER_TIMER_COUNTER = 0;
}

void StartIRTimer() {
	IR_TIMER_COUNTER = 0;
	IR_TIMER = (1<< CS32) | (1<< CS30);
}

void StopIRTimer() {
	IR_TIMER &= ~(1 << CS32);
	IR_TIMER &= ~(1 << CS30);
	IR_TIMER_COUNTER = 0;
	IRCTR = 0;
}


void BlinkLEDs() {
	
}


