/*
 * Målsökningsenheten
 * 
 * TSEA29 Group 15
 * LIU
 */ 


#include "../../Robotdefinitions.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


// Function headerss
void SetPriority(int priority);
void ClearPriority();
void WeAreHit();
void Shoot();
void Rotate(long milliDegrees, bool leftTurn);
int calculateGyroOffset(long milliDegrees);
void StopIRTimer();
void StartIRTimer();
void StartLaserTimer();
void StopLaserTimer();
void Scan();
int Priority(int operation);
unsigned long CalcGyro(int gyroData);
void testTAPEsensors();
int Abs(int value);
void SendUART();
void StartBackwardsTimer();
void StopBackwardsTimer();
void StopRotate(int orderToPerformOnStop);
bool UpdateRotation();




//initial values are set to their IDs from design spec. (these bits are never changed)
uint8_t message1 = 0;
uint8_t message2 = 1;
uint8_t message3 = 2;
uint8_t message4 = 3;
uint8_t message5 = 4;
uint8_t message6 = 5;

uint8_t messageout4 = 3;
uint8_t messageout5 = 4;

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

#define BACKWARDS_TIMER TCCR0B
#define BACKWARDS_TIMER_CTR TCNT0

#define LASER_TIMER TCCR1B
#define LASER_TIMER_COUNTER TCNT1
#define IR_TIMER TCCR3B
#define IR_TIMER_COUNTER TCNT3

#define ONE_SECOND 18000
#define HALF_SECOND 9000
#define QUARTER_SECOND 4500

// Data
bool canShoot = true;
int health = 3;
bool dead = false;

bool scaning = false;

bool movingForward = false;
bool bothTapeSensors = false;

// Rotation stuff
bool rotating = false;
int currentRotationValue;
long targetRotation;

// Backing stuff
bool backing = false;
bool leftTapeHit = false;
bool rightTapeHit = false;
int backing_ctr = 0;
const int timeToReachOneHundredth = 180;

// Laser stuff
int coolDownCTR = 0;
bool laserActive = false;

bool laserSensorHit = false;

// IR-signature counting
long enemySignatureCTR = 0;
long enemySignatureLimit = 100000;

// Used to count up to 3
int IRCTR = 0;
int	currentPriority = -1;
bool foundSomethingToDo = false;
int orders[10];

//gyro
const float sampleTimeInMS = 5; // the gyro gives us degrees/sec
const int sampleticks = 90 ;
double millidegreesTurned = 0;


//Test kod för testläge
bool targetDistanceIsSet = false;
bool needToRotate = false;
int maxDistance = 5;
int distanceToTarget = 0;
bool isPositioning = false;
bool checkLeft = false;
bool scanDone = false;
bool prepareToFire = false;

void positioning();


int main(void)
{
	DDRB = 0b11111011;
	DDRC = 0xFF;
	
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
	
	//enable transmit interrupt was for testing blueetooth
	//UCSR1B |= (1<<TXCIE0);
	//#UART INITS END#//
	
	waitForActivation();
	
	//start first UART transmission just for testing bluetooth
	//UDR1 = 0x00;
	
	//###first order!###
	nextOrder = MOVE_FORWARD;
	//nextOrder = ACTIVATE_LASER;
	//nextOrder = TURN_OFF_IR_SIG;
	//Shoot();
	
	//WeAreHit();
	//health = 1;
	//WeAreHit();
	
	
	
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
		
		//debugging
		if(ultraSonicSensor1 < 1){
			PORTC |= (1 << PINC0);
		}
		else{
			PORTC &= ~(1 << PINC0);
		}
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
		SendUART();
		
		if((PINB>>PINB2) == 0){
			//Tävling
			
// 			if (!rotating) {
// 				Shoot();
// 			}
// 			
// 			// LASER timer stuff
// 			if (LASER_TIMER_COUNTER >= ONE_SECOND) {
// 				coolDownCTR++;
// 				LASER_TIMER_COUNTER = 0;
// 							
// 				// Lasers been active for 1 sec, turn it off
// 				if (coolDownCTR == 1) {
// 					nextOrder = DEACTIVATE_LASER;
// 					laserActive = false;
// 					foundSomethingToDo = true;
// 					continue;
// 				}
// 				// cooldown is over
// 				else if (coolDownCTR == 4) {
// 					canShoot = true;
// 					StopLaserTimer();
// 				}
// 							
// 			}

			// IR timer stuff
// 			if (IR_TIMER_COUNTER >= ONE_SECOND) {
// 				IR_TIMER_COUNTER = 0;
// 				IRCTR++;
// 				if (IRCTR >= 5) {
// 					StopIRTimer();
// 					nextOrder = TURN_ON_IR_SIG;
// 					continue;
// 				}
// 			}
		
		
		

		}
		else{
			//##############
			//## Testläge ##
			//##############
			
			// LASER timer stuff
			if (LASER_TIMER_COUNTER >= ONE_SECOND) {
				coolDownCTR++;
				LASER_TIMER_COUNTER = 0;
							
				// Lasers been active for 1 sec, turn it off
				if (coolDownCTR == 1) {
					nextOrder = DEACTIVATE_LASER;
					laserActive = false;
					foundSomethingToDo = true;
					continue;
				}
				// cooldown is over
				else if (coolDownCTR == 4) {
					canShoot = true;
					StopLaserTimer();
					coolDownCTR = 0;
				}
							
							
			}
			
			
			if (activeIRsignature) {
				enemySignatureCTR++;	
			}
			
			else{
				enemySignatureCTR = 0;
			}
			
			
			//test för att centrera framför fyr.
			if (enemySignatureCTR >= enemySignatureLimit && !isPositioning) {
// 				nextOrder = ACTIVATE_LASER;
// 				continue;
				isPositioning = true;
			}
			

			
// 			if(laserSensor == 1 && !laserSensorHit){
// 				foundSomethingToDo = true;
// 				laserSensorHit = true;
// 				WeAreHit();
// 				continue;
// 
// 			}
// 			else if (laserSensor == 0 && laserSensorHit) {
// 				laserSensorHit = false;
// 			}
			/*
			// IR timer stuff
			if (IR_TIMER_COUNTER >= ONE_SECOND) {
				IRCTR++;
				if (IRCTR >= 5) {
					StopIRTimer();
					nextOrder = TURN_ON_IR_SIG;
					foundSomethingToDo = true;
					continue;
				}
			}
		*/

			
			
		
			// If the Left line sensor detects tape and we havn't startet rotating, turn right
			if((tapeSensor1 == 1) && !rotating && !backing){ 
				leftTapeHit = true;
				StartBackwardsTimer();
				nextOrder = MOVE_BACKWARDS;
				continue;

			}
		
			// If the Right line sensor detects tape and we havn't startet rotating, turn left
			if((tapeSensor2 == 1) && !rotating && !backing){
				rightTapeHit = true; 
				StartBackwardsTimer();
				nextOrder = MOVE_BACKWARDS;
				continue;
			}
		
			if (backing) {
				if(BACKWARDS_TIMER_CTR >= timeToReachOneHundredth){
					BACKWARDS_TIMER_CTR = 0;
					backing_ctr++;
					
					if (backing_ctr >= 50) {
						backing = false;
						StopBackwardsTimer();
						
						if (leftTapeHit) {
							leftTapeHit = false;
							Rotate(45000, false);
						}
						else if (rightTapeHit) {
							rightTapeHit = false;
							Rotate(45000, true);
						}
					}
					
				}
			}
			
			// If we are rotating
			if (rotating) {
				if (UpdateRotation())
					continue;
			}
			
			if (isPositioning) {
				positioning();
				continue;
			}
			
 		/*
		 	// If we are scaning for opponents and we find something within 1,5 meters, stop move, 
			if (scaning && rotating) {
				if (ultraSonicSensor1 <= 10) {
					rotating = false;
					scaning = false;
					foundSomethingToDo = true;
					nextOrder = STOP_MOVING;
					continue;
				}
			}
		
			// If something is in front of the robot and the IR-signature is active
			if (ultraSonicSensor1 <= 10 && activeIRsignature) {
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
				continue;
			}
		 
			 // If there is nothing else to do, move forward, KEEP THIS??
			if (!foundSomethingToDo) {
				nextOrder = MOVE_FORWARD;
			}
		*/
			
			
		}

    }
	
	//#############################
	//#### DEATH CODE #############
	//#############################
	
	// Reset the laser timers count variable and start the laser timer (used to get a blinking LEDs)
	StartLaserTimer();
	// This loop blinks the LEDs to show that we are dead
	while (dead) {
		SendUART();
		if (LASER_TIMER_COUNTER >= 2250) {
			LASER_TIMER_COUNTER = 0;
			coolDownCTR++;
			nextOrder = DECREMENT_LED_LIVES;
			if (coolDownCTR == 4) {
				nextOrder = RESET_SE;
				coolDownCTR = 0;
			}
		}
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
			if(nextOrder != DO_NOTHING){ //Only send order if something is to be done
				message6 &= 0b00000111; //reset everything except message ID
				UDR1 = (nextOrder<<3) | message6;
				nextOrder = 0;
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


//UART TESTING WITH INTERUPTS
/*
ISR(USART1_TX_vect){

	//mux through messages
	//may need to disable interrupts
	cli();
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
		else{
			UDR1 = 0x00;
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
	sei();
	
}
*/

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
// 			message5 &= 0b00111000; //We set the gyro bits elsewhere
// 			message5 |= (buffer & 0b11000111);
	
			break;
	}
}

//####################### NEW ###############################


// This functions stops the rotation and performs the provided order
// Should call continue after this
void StopRotate(int orderToPerformOnStop) {
	rotating = false;
	millidegreesTurned = 0;
	targetRotation = 0;
	TCNT2 = 0;
	TCCR2B &= ~((1 << CS20) | (1 << CS21) | (1 << CS22));
	
	nextOrder = orderToPerformOnStop;
}

<<<<<<< HEAD
bool UpdateRotation() {
=======
void UpdateRotation() {
>>>>>>> origin/master
		if(TCNT2 >= sampleticks){
		//reset counter
		TCNT2 = 0;
		
		//300 is max angular rate from gyro
		//calculate how much we rotate per sample in millidegrees/second and add it total total millidegreesturned
		float degreesPerPart = 300/128;
		int parts = Abs(gyro - ANGULAR_RATE_IDLE);
		float angularVelocity = parts*degreesPerPart;
		millidegreesTurned += angularVelocity*sampleTimeInMS;
		
		if (millidegreesTurned >= targetRotation) {
<<<<<<< HEAD
			if(laserActive){
				Rotate(SHOOT_SWEEP_DEGREES, true);
				laserActive = false;
			}
			else{
				StopRotate(MOVE_FORWARD);
			}
			return true;
=======
			StopRotate(MOVE_FORWARD);
>>>>>>> origin/master
		}
		//Send how many degrees we have rotated over uart
		
// 		uint8_t degreesTurned = millidegreesTurned/1000;
// 		messageout4 &= 0b00000000; //Reset bits
// 		messageout4 |= (degreesTurned<<LOWERBITSGYRO_INDEX);
// 		messageout4 |= (message4 & 0b00000111);
// 		messageout5 &= 0b00000000; //Reset bits
// 		messageout5 |= (degreesTurned>>2);
// 		messageout5 |= (message5 & 0b11000111);
		
//		messageout5 = gyro;
<<<<<<< HEAD
	}
	return false;
=======
		

	}
>>>>>>> origin/master
}

void Scan() {
	scaning = true;
	Rotate(360, true);
}

void Rotate(long milliDegrees, bool leftTurn) {
	rotating = true;
	movingForward = false;
	//different offsets for different rotations
	
	targetRotation = milliDegrees - calculateGyroOffset(milliDegrees);
	
	
	if (leftTurn) {
		nextOrder = TURN_LEFT;
	}
	else {
		nextOrder = TURN_RIGHT;
	}
	TCNT2 = 0;
	//start timer
	TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22);
	
}

/*
sampleTimeSecs = 1;
unsigned long CalcGyro(int gyroData){
	gyroSum += gyroData;
	return (gyroSum * sampleTimeSecs);
}*/

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
 	targetRotation = (SHOOT_SWEEP_DEGREES / 2) - calculateGyroOffset(SHOOT_SWEEP_DEGREES / 2);
	
	//start timer
	TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22);
	
	nextOrder = ACTIVATE_LASER_AND_TURN_RIGHT;
	StartLaserTimer();
	
}

int calculateGyroOffset(long milliDegrees){
	long degreeOffset;
	switch(milliDegrees){
		case 22500:
		degreeOffset = OFFSET_ROTATE_22POINT5;
		break;
		case 45000:
		degreeOffset = OFFSET_ROTATE_45;
		break;
		case 90000:
		degreeOffset = OFFSET_ROTATE_90;
		break;
		case 135000:
		degreeOffset = OFFSET_ROTATE_135;
		break;
		case 180000:
		degreeOffset = OFFSET_ROTATE_180;
		break;
		default:
		degreeOffset = (milliDegrees*333.333)/1000 + 2500;
		break;
	}
	return degreeOffset;
}

void StartBackwardsTimer() {
	backing = true;
	BACKWARDS_TIMER_CTR = 0;
	BACKWARDS_TIMER = (1<< CS02) | (1<< CS00);
	backing_ctr = 0;
}

void StopBackwardsTimer() {
	
	BACKWARDS_TIMER &= ~(1 << CS12);
	BACKWARDS_TIMER &= ~(1 << CS10);
	BACKWARDS_TIMER_CTR = 0;
	backing_ctr = 0;
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
	IRCTR = 0;
	IR_TIMER = (1<< CS32) | (1<< CS30);
}

void StopIRTimer() {
	IR_TIMER &= ~(1 << CS32);
	IR_TIMER &= ~(1 << CS30);
	IR_TIMER_COUNTER = 0;
	IRCTR = 0;
}


void positioning() {
	if (!targetDistanceIsSet) {
		//check if something is inside the cone.
		if (!needToRotate && (ultraSonicSensor1 <= maxDistance)) {
			//save the distance to your enemy
			distanceToTarget = ultraSonicSensor1;
			
			targetDistanceIsSet = true;
			Rotate(90000, true);
		}
		else {
			needToRotate = true;
			
			//Check to the left.
			if (!scanDone) {
				if (!checkLeft) {
					checkLeft = true;
					if (!rotating){
						Rotate(90000, true);
					}
				}
				else {	//check to the right.
					if (!rotating) {
						checkLeft = false;
						Rotate(180000, false);
						scanDone = true;
					}
				}
				
			}
			
			//if you find something, stop rotating.
			if (ultraSonicSensor1 <= maxDistance) {				
				distanceToTarget = ultraSonicSensor1 - 1;
				targetDistanceIsSet = true;
			}	
			if(scanDone && !rotating){
				isPositioning = false;
				needToRotate = false;
			}
		
		}
	}
	else {
		//when distance is greater then target distance, rotate half a cone to the opposite direction.

		if (ultraSonicSensor1 > distanceToTarget && !prepareToFire) {
			if (checkLeft) {
				Rotate(22500, true);
			}
			else {	//if you were rotating right, keep rotating right.
				Rotate(22500, false);
			}
			prepareToFire = true;
		}
		
		//Rotating is complete. SHOOT.
		if (!rotating && canShoot) {
			//nextOrder = STOP_MOVING;
			Shoot();
			isPositioning = false;
			targetDistanceIsSet = false;
			needToRotate = false;
			checkLeft = false;
			prepareToFire = false;
			enemySignatureCTR = 0;
		}
	
	}
}