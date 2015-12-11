/*
 * M�ls�kningsenheten
 * 
 * TSEA29 Group 15
 * LIU
 */ 


#include "../../Robotdefinitions.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>


// Function headers
void SetPriority(int priority);
void ClearPriority();
void WeAreHit();
void Shoot();
void Rotate(long milliDegrees, bool leftTurn);
long calculateGyroOffset(long milliDegrees);
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


//Variables for storing sensor values
volatile uint8_t tapeSensor1 = 0; //1 for tape, 0 for floor
volatile uint8_t tapeSensor2 = 0; //1 for tape, 0 for floor
volatile uint8_t ultraSonicSensor1 = 0; //distance in dm
volatile uint8_t ultraSonicSensor2 = 0; //distance in dm
volatile uint8_t isEnemy = 0; //1 if enemy in front
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


//Main loop functions
void IRDebouncer();
bool checkForTape();
bool checkBacking();
bool checkLaserSensor();
bool checkLaserCooldown();
bool invisibilityHandler();
bool collisionCheck();
void snapshotUART();

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
const long enemySignatureLimit = 50000;

// Used to count up to 3
int IRCTR = 0;
int	currentPriority = -1;
int orders[10];

//gyro
const float sampleTimeInMS = 5; // the gyro gives us degrees/sec
const int sampleticks = 90 ;
double millidegreesTurned = 0;


//Test kod f�r testl�ge
bool targetDistanceIsSet = false;
bool needToRotate = false;
int maxDistance = 20;
int distanceToTarget = 0;
bool isPositioning = false;
bool checkLeft = false;
bool scanDone = false;
bool prepareToFire = false;

void positioning();

// Test kod f�r t�vling
long forwardTickCTR = 0;	// This is used to see how long we have Moved Forward(kinda)
long forwardMaxTimeInTicks = 100000;
int maxRange = 15;
bool isMovingForward = true;
bool signatureConfirmed = false;



int main(void)
{
	DDRB = 0b11111011;
	DDRC = 0xFF;
	
	//enable global interrupts
	sei();
	
	//################
	//## UART INITS ##
	//################
	
	//initiate UART m�ls�kning from sensor
	//set baud rate
	uint16_t UBRR_val = UBRR_SENSOR_MALSOKNING;
	UBRR0H = (unsigned char) (UBRR_val >> 8);
	UBRR0L = (unsigned char) UBRR_val;
	//enable receive + set frame 8 bits
	UCSR0B = (1<<RXEN0);
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);

	//enable receive interrupt
	UCSR0B |= (1<<RXCIE0);
	
	//initiate UART m�ls�kning to styr
	//set baud rate
	//115200
	uint16_t UBRR_val1 = UBRR_STYR_MALSOKNING;
	UBRR1H = (unsigned char) (UBRR_val1 >> 8);
	UBRR1L = (unsigned char) UBRR_val1;
	
	//enable transmit + set frame 8 bits
	UCSR1B = (1<<TXEN1);
	UCSR1C = (1<<UCSZ10) | (1<<UCSZ11);
	
	//# UART INITS END #//
	
	waitForActivation();
	
	//###first order!###
	Rotate(50000,true);
	
	//Main loop when we are alive
    while(!dead)
    {
		snapshotUART(); 
		SendUART();
	
		//Check a switch on the robot to determine what mode the robot is set to
		if((PINB>>PINB2) == 0){
			//#############
			//## T�vling ##
			//#############

			IRDebouncer();			
			if(enemySignatureCTR >= enemySignatureLimit && !isPositioning) isPositioning = true;
			if(checkLaserSensor() || invisibilityHandler() || checkLaserCooldown() || checkForTape() || checkBacking() || collisionCheck()) continue;
			
			if (rotating) {
				if (UpdateRotation()){
					continue;
				}
			}
			
			if (isPositioning) {
				positioning();
				continue;
			}
		}
		else{
			//##############
			//## Testl�ge ##
			//##############
		
			IRDebouncer();
			if(enemySignatureCTR >= enemySignatureLimit && !isPositioning) isPositioning = true; //Start positioning if we find an enemy signature
			if(checkLaserSensor() || invisibilityHandler() || checkLaserCooldown() || checkForTape() || checkBacking()) continue;
			
			if (rotating) {
				if (UpdateRotation()){
					continue;
				}
			}
			
			if (isPositioning) {
				positioning();
				continue;
			}	
		}
    }
	
	//#############################
	//#### DEATH CODE #############
	//#############################

	// Reset the laser timers count variable and start the laser timer (used to get a blinking LEDs)
	StartLaserTimer();
	// This loop blinks the LEDs to show that we are dead
	nextOrder = MOVE_FORWARD_AND_TURN_INVISIBLE_AND_DEC_LIFE_LED;
	
	bool startedMoveForward = false;
	
	//Loops when we are dead
	while (dead) {
		SendUART();
		snapshotUART();
		
		// If the left tape sensor detects tape and we haven't started rotating, turn right
		if(tapeSensor1 == 1 && !leftTapeHit){
			leftTapeHit = true;
			nextOrder = TURN_LEFT;
			continue;
		}
	
		// If the right line sensor detects tape and we haven't started rotating, turn left
		if(tapeSensor2 == 1 && !rightTapeHit){
			rightTapeHit = true;
			nextOrder = TURN_RIGHT;
			continue;
		}
		
		// When both left and right tape sensors have detected tape, move forward
		if (leftTapeHit && rightTapeHit && !startedMoveForward) {
			nextOrder = MOVE_FORWARD;
			startedMoveForward = true;
			StartIRTimer();
			continue;
		}
	
		// Stop after we've moved forward for 1 second (should now be outside the map)
		if (IR_TIMER_COUNTER >= 18000) {
			nextOrder = STOP_MOVING;
			StopIRTimer();
		}
		
		// Pulse LEDs
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

/*
	Sends messages to the "styrenhet"
*/
void SendUART() {
	//check if transmit buffer is empty
	if((UCSR1A & (1<<UDRE1))){
		//mux through messages
		switch(messageNumber){
			case 1:
				message1 &= ~(1<<IRSENSOR_INDEX); //Reset bit
				(isEnemy ? message1 |= (1<<IRSENSOR_INDEX) : message1 &= ~(1<<IRSENSOR_INDEX));
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

/*
	Interrupt vector for receiving UART
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
			break;
	}
}


/*
	This functions stops the rotation and performs the provided order
	Should call continue after this.
*/
void StopRotate(int orderToPerformOnStop) {
	rotating = false;
	millidegreesTurned = 0;
	targetRotation = 0;
	TCNT2 = 0;
	TCCR2B &= ~((1 << CS20) | (1 << CS21) | (1 << CS22));
	
	nextOrder = orderToPerformOnStop;
}

/*
	Checks if we have reached the target rotation.
	Returns true when reached.
*/
bool UpdateRotation() {
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
			if(laserActive){
				Rotate(SHOOT_SWEEP_DEGREES, true);
				laserActive = false;
			}
			else{
				StopRotate(MOVE_FORWARD);
				isMovingForward = true;		// t�vlingsl�ge
			}
			
			// used to only rotate 360 when searching.
			isPositioning = false;
			enemySignatureCTR = 0;
			
			return true;
		}
	}
	return false;
}

/*
	Rotates milliDegrees with direction left if leftTurn == true, otherwise right
*/
void Rotate(long milliDegrees, bool leftTurn) {
	rotating = true;
	movingForward = false;
	//different offsets for different rotations
	
	millidegreesTurned = 0;
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
	Returns the absolute value.
*/
int Abs(int value) {
	if (value < 0) {
		value *= -1;
	}
	return value;
}
	
/*
	Decrements life by 1 and turns us invisible (turns off the IR signature)
*/
void WeAreHit() {
	health--;
	nextOrder = TURN_INVISIBLE_AND_DEC_LIFE_LED;
		
	// We are dead
	if (health <= 0) {
		dead = true;
		return;
	}

	StartIRTimer();
}

/*
	Activates the laser and does a sweeping movement first to the right, then back to the left.
*/
void Shoot() {
	laserActive = true;
	canShoot = false;
 	rotating = true;
	millidegreesTurned = 0;
 	targetRotation = (SHOOT_SWEEP_DEGREES / 2) - calculateGyroOffset(SHOOT_SWEEP_DEGREES / 2);
	
	//start timer
	TCCR2B |= (1 << CS20) | (1 << CS21) | (1 << CS22);
	
	nextOrder = ACTIVATE_LASER_AND_TURN_RIGHT;
	StartLaserTimer();
}

/*
	For some reason the gyro doesn't give us good enough data so we have to use offsets
	for turns, and this function calculates the offsets.
	Returns the offset.
*/
long calculateGyroOffset(long milliDegrees){
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
		case 360000:
			degreeOffset = OFFSET_ROTATE_360;
			break;
		default:
			degreeOffset = (milliDegrees*333.333)/1000 + 2500;
			break;
	}
	return degreeOffset;
}

/*
	Resets the backing timer and starts it.
*/
void StartBackwardsTimer() {
	backing = true;
	BACKWARDS_TIMER_CTR = 0;
	BACKWARDS_TIMER = (1<< CS02) | (1<< CS00);
	backing_ctr = 0;
}

/*
	Stops the backing timer and resets it.
*/
void StopBackwardsTimer() {
	BACKWARDS_TIMER &= ~(1 << CS12);
	BACKWARDS_TIMER &= ~(1 << CS10);
	BACKWARDS_TIMER_CTR = 0;
	backing_ctr = 0;
}

/*
	Resets the laser timer and starts it.
*/
void StartLaserTimer() {
	LASER_TIMER_COUNTER = 0;
	LASER_TIMER = (1<< CS12) | (1<< CS10);
}

/*
	Stops the laser timer and resets it.
*/
void StopLaserTimer() {
	LASER_TIMER &= ~(1 << CS12);
	LASER_TIMER &= ~(1 << CS10);
	LASER_TIMER_COUNTER = 0;
}

/*
	Resets the IR timer and starts it.
*/
void StartIRTimer() {
	IR_TIMER_COUNTER = 0;
	IRCTR = 0;
	IR_TIMER = (1<< CS32) | (1<< CS30);
}

/*
	Stops the IR timer and resets it.
*/
void StopIRTimer() {
	IR_TIMER &= ~(1 << CS32);
	IR_TIMER &= ~(1 << CS30);
	IR_TIMER_COUNTER = 0;
	IRCTR = 0;
}

/*
	Rotates 360 degrees or until an enemy is detected in front of it and if so shoots at it if the laser is off cooldown.
*/
void positioning() {
	if (ultraSonicSensor1 <= maxDistance && enemySignatureCTR >= enemySignatureLimit && canShoot) {
		Shoot();
		isPositioning = false;
		enemySignatureCTR = 0;
	}
	else {
		if (!rotating) {
			Rotate(360000, false);
		}
	}
}

/*
	Checks if we have have detected tape and if so, sets backing to true to back away from it and then turn.
	Returns true if tape is detected.
*/
bool checkForTape(){
	// If the Left line sensor detects tape and we haven't started rotating, turn right.
	// If backing is true, the tape has already been registered.
	if((tapeSensor1 == 1) && !backing){
		leftTapeHit = true;
		StartBackwardsTimer();
		nextOrder = MOVE_BACKWARDS;
		return true;
	}
	
	// If the Right line sensor detects tape and we haven't started rotating, turn left.
	// If backing is true, the tape has already been registered.
	if((tapeSensor2 == 1) && !backing){
		rightTapeHit = true;
		StartBackwardsTimer();
		nextOrder = MOVE_BACKWARDS;
		return true;
	}
	return false;
}

/*
	Handles backing away from the edge of the map and after 0.5 sec makes a random turn
	Returns true after a turn order has been given.
*/
bool checkBacking(){
	if (backing) {
		if(BACKWARDS_TIMER_CTR >= timeToReachOneHundredth){
			BACKWARDS_TIMER_CTR = 0;
			backing_ctr++;
			if (backing_ctr >= 50) {
				backing = false;
				StopBackwardsTimer();
				if (leftTapeHit) {
					leftTapeHit = false;
					Rotate(60000 + (rand()%170)*1000, false);
					return true;
				}
				else if (rightTapeHit) {
					rightTapeHit = false;
					Rotate(60000 + (rand()%170)*1000, true);
					return true;
				}
			}
		}
	}
	return false;
}

/*
	Stabilizes when we shoot at an enemy. This is necessary because the IR-sensor is a bit unreliable
	when there's more than one IR-signature nearby and we must not shoot at allies.
*/
void IRDebouncer(){
	// IR Signature debouncing
	if (isEnemy) {
		//Cap at enemySignatureLimit * 2
		if (!(enemySignatureCTR >= enemySignatureLimit * 2)) {
			enemySignatureCTR += (rotating ? 6 : 1); //Increment faster if we are rotating to faster pick up new enemies
		}
	}
	else{
		enemySignatureCTR -= (rotating ? 12 : 3); //Decrement faster if we are rotating to minimize risk of shooting allies
		//Cap at 0
		if (enemySignatureCTR < 0){
			enemySignatureCTR = 0;
		}
	}
	
	//test f�r att centrera framf�r fyr.
	if (enemySignatureCTR >= enemySignatureLimit && !isPositioning) {
		isPositioning = true;
	}
}

/*
	Checks if we've been hit by a laser and makes sure that we only register every hit 1 time
	Returns true if we're hit.
*/
bool checkLaserSensor(){
	if(laserSensor == 1 && !laserSensorHit){
		laserSensorHit = true;
		WeAreHit();
		return true;
		
	}
	else if (laserSensor == 0 && laserSensorHit) {
		laserSensorHit = false;
	}
	return false;
}

/*
	Makes sure that we don't shoot too often (must be 3 sec between each shot), and that every shot is 1 second
	Returns true after laser has been active for 1 second.
*/
bool checkLaserCooldown(){
	// LASER timer stuff
	if (LASER_TIMER_COUNTER >= ONE_SECOND) {
		coolDownCTR++;
		LASER_TIMER_COUNTER = 0;
		
		// Lasers been active for 1 sec, turn it off
		if (coolDownCTR == 1) {
			nextOrder = DEACTIVATE_LASER;
			laserActive = false;
			return true;
		}
		// cooldown is over
		else if (coolDownCTR == 4) {
			canShoot = true;
			StopLaserTimer();
			coolDownCTR = 0;
		}
	}
	return false;
}

/*
	Makes sure that we turn on the IR-signature 5 seconds after we turn it off (after we've been shot).
	Returns true when the IR-signature is turned back on.
*/
bool invisibilityHandler(){
	//IR timer stuff
	if (IR_TIMER_COUNTER >= ONE_SECOND) {
		IR_TIMER_COUNTER = 0;
		IRCTR++;
		if (IRCTR >= 5) {
			StopIRTimer();
			nextOrder = TURN_ON_IR_SIG;
			return true;
		}
	}
	return false;
}

/*
	Checks if the robot is about to crash into something.
	Returns true if a turn order is given.
*/
bool collisionCheck(){
	if(ultraSonicSensor1 <= COLLISION_DISTANCE && !rotating){
		Rotate(80000, (rand()%2 ? true : false)); //Turn 80 degrees left/right randomly
		return true;
	}
	return false;
}

/*
	Snapshots values from the sensor unit from UART.
*/
void snapshotUART(){
	//disable interrupts
	cli();
	
	//############################
	//## Snapshot sensor values ##
	//############################
	//Message 1
	IRSignature = (message1>>IRSIGNATURE_INDEX) & 0b00000111;
	laserSensor = (message1>>LASER_INDEX) & 0b00000001;
	isEnemy = (message1>>IRSENSOR_INDEX) & 0b00000001;
	
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
}