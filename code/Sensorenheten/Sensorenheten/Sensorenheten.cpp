/*
 * Sensorenheten
 * 
 * TSEA29 Group 15
 * LIU
 */ 

#include "../../Robotdefinitions.h"
#include "util/delay.h"

#define GYRO		0
#define TAPESENSOR1 1
#define TAPESENSOR2 2

//initial values are set to their IDs from design spec. (these bits are never changed)
uint8_t message1 = 0;
uint8_t message2 = 1;
uint8_t message3 = 2;
uint8_t message4 = 3;
uint8_t message5 = 4;

uint8_t ADCcount = 1;

uint8_t messageNumber = 1;

uint8_t laser;

void laserSensorFunction();
void waitForActivationSensor();
uint8_t tapeCheck(uint16_t message, uint8_t tapeSensorNumber);
void clearADCMUX();
void testTAPEsensors();

//###############
//## IR SENSOR ##
//###############

#define IRheader 20
#define IRzerovalue 7

bool compareSignature(uint8_t* oursignature, uint8_t* signature);
void IRSensorFunction();
bool edge = false;
bool header = false;
bool enemy = false;
uint8_t signature[] = {0,0,0};
uint8_t signatureS = 0;
uint8_t oursignature[] = {1,1,0};
int index = 0;
int IRtimer = 0;

//########################
//## Ultrasonic Sensors ##
//########################

void ultrasonicFunction();
void StartPulse1();
void StartPulse2();

void CalculateTime1();
void CalculateTime2();

void resetTimerValues();

int calculateDistance();

int timer = 0;

// Variables
uint8_t distance1 = 0; //Ultrasonic sensor 1 distance [dm]
uint8_t distance2 = 0; //Ultrasonic sensor 2 distance [dm]

bool triggerStarted = false;
bool triggerSend = false;

bool timerStarted = false;
bool timeTaken = false;

bool useSensor1 = true;


int main(void)
{
	//init �controller
	
	/*
	PIN0: Ultrasonic 1		(in)
	PIN1: Ultrasonic 1		(out)
	PIN2: Ultrasonic 2		(in)
	PIN3: Ultrasonic 2		(out)
	PIN4: Laser Aktivera	(out)
	PIN5: Laser				(in)
	*/
	
	DDRD &= ~(1<<PIND7); //Aktiveringsknapp (in)
	DDRB = 0b10011010;
		
	//enable global interrupt
	sei();
	
	//Ultrasonic
	TCCR2B |= 1 << CS20; // Start timer
	TCNT2 = 0;
	
	//###############
	//## ADC INITS ##
	//###############
		
	//turn on ADC
	ADCSRA |= (1 << ADEN);
	//Set ADC clock to 1000 000 / 16 Hz
	ADCSRA |= (1 << ADPS2);
	//ADC interrupt enabled
	ADCSRA |= (1 << ADIE);
	//mux PINA0, start with TAPE
	ADMUX |= (1<<PINA0);
	
	//################
	//## UART INITS ##
	//################
	
	//initiate UART m�ls�kning
	//set baud rate
	uint16_t UBRR_val = UBRR_SENSOR_MALSOKNING;
	UBRR0H = (unsigned char) (UBRR_val >> 8);
	UBRR0L = (unsigned char) UBRR_val;
	//enable transmit + set frame 8 bits
	UCSR0B = (1<<TXEN0);
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
	//enable transmit interrupt
	UCSR0B |= (1<<TXCIE0);
	
	//inactivate
	PORTB |= (1 << LASER_AKTIVERA_PORT);
	_delay_ms(10);
	//activate, might have to wait if this is too fast
	//PORTB &= 0b11101111;
	PORTB &= ~(1 << LASER_AKTIVERA_PORT);
	
	//## IR Sensor ##
	TCCR1B |= 1 << CS10;
	
	waitForActivationSensor();
	
	//start first UART transmission
	UDR0 = 0x00;
	//Start 1st ADC conversion
	ADCSRA |= (1 << ADSC);
	
    while(1)
    {
		//###############
		//## IR SENSOR ##
		//###############
		
		IRSensorFunction();
		
		//##################
		//## Laser sensor ##
		//##################
		
		laserSensorFunction();
		
		//########################
		//## Ultrasonic Sensors ##
		//########################
		
		ultrasonicFunction();
		
	}
	
}

ISR(USART0_TX_vect){
	//disable interrupts (might not be necessary?) interrupts get queued?
	cli();
	
	//mux through messages
	switch(messageNumber){
		case 1:
			UDR0 = message1;
			break;
		case 2:
			UDR0 = message2;
			break;
		case 3:
			UDR0 = message3;
			break;
		case 4:
			UDR0 = message4;
			break;
		case 5: 
			UDR0 = message5;
			break;
	}
	
	//next mux
	messageNumber++;
	if(messageNumber>NUMBER_OF_MESSAGES) messageNumber=1;
	
	//enable interrupts
	sei();
}


ISR(ADC_vect){
	uint8_t lowbits = ADCL;
	uint16_t message = ADCH <<8 | lowbits;
	
	switch(ADCcount){
		case GYRO:
			message >>= 2; //Divide by 4
			message4 &= 0b00000111; //Reset bits
			message4 |= (message<<LOWERBITSGYRO_INDEX);
			message5 &= 0b11000111; //Reset bits
			message5 |= ((message>>2) & 0b00111000);
			clearADCMUX();
			ADMUX |= 1;
			break;
		case TAPESENSOR1:
			//clear TAPESENSOR1_INDEX bit
			message5 &= 0b10111111;
			tape2CurrentValue = message; //Save current value, used for calibration
			//mask in actual value
			message5 |= (tapeCheck(message, 1)<<TAPESENSOR1_INDEX);
			//next muxed ADC
			clearADCMUX();
			ADMUX |= 2;
			break;
		case TAPESENSOR2:
			//clear TAPESENSOR2_INDEX bit
			message5 &= 0b01111111;
			tape2CurrentValue = message; //Save current value, used for calibration
			//mask in actual value
			message5 |= (tapeCheck(message, 2)<<TAPESENSOR2_INDEX);
			//next muxed ADC
			clearADCMUX();
			//MUX to IRSENSOR-ADC?
			break;
	}
	
	ADCcount++;
	//go around
	if(ADCcount >= NUMBER_OF_ADC_SENSORS){
		 ADCcount = 0;
		 clearADCMUX();
	}
	
	//Start next conversion
	ADCSRA |= (1 << ADSC);
}

//Check if the converted value from the tape sensors indicate tape or not
uint8_t tapeCheck(uint16_t message, uint8_t tapeSensorNumber){
	if((tapeSensorNumber == 1 && message >= tape1Threshold) || (tapeSensorNumber == 2 && message >= tape2Threshold)){
		return 1;
	}
	else{
		return 0;
	}
}

void clearADCMUX(){
	ADMUX &= 0b11100000;
}

//just test function
//PIN B4 and B5 as outputs


void resetTimerValues() {
	TCNT2 = 0;
	timer = 0;
}


void StartPulse1() {
	//set trigger to high
	if (!triggerStarted) {
		triggerStarted = true;
		PORTB |= (1 << PULSE_TRIGGER_PIN1);
		resetTimerValues();
	}
	
	//set trigger to low
	if (triggerStarted && timer == 2) { // and atleast 15 us has passed.
		triggerSend = true;
		PORTB &= ~(1 << PULSE_TRIGGER_PIN1);
	}
}

void StartPulse2() {
	//set trigger to high
	if (!triggerStarted) {
		triggerStarted = true;
		PORTB |= (1 << PULSE_TRIGGER_PIN2);
		resetTimerValues();
	}
	
	//set trigger to low
	if (triggerStarted && timer == 2) { // and atleast 15 us has passed.
		triggerSend = true;
		PORTB &= ~(1 << PULSE_TRIGGER_PIN2);
	}
}


void CalculateTime1() {
	//when echo output is high, start timer.
	if (!timeTaken && triggerSend && !timerStarted && (PINB & (1<<ECHO_PIN1))) {
		timerStarted = true;
		resetTimerValues();
	}
	
	//when echo output is low, stop timer. (save time)
	if (!timeTaken && triggerSend && timerStarted && !(PINB & (1<<ECHO_PIN1))) {
		//set timer variables ot zero.
		distance1 = calculateDistance();
		message2 &= ~(0b11111<<3); //Reset distance bits
		message2 |= (distance1<<3); //Set UART message with new distance
		
		resetTimerValues();
		timeTaken = true;
		useSensor1 = false;
		//reset trigger variables.
	}
}

void CalculateTime2() {
	//when echo output is high, start timer.
	if (!timeTaken && triggerSend && !timerStarted && (PINB & (1<<ECHO_PIN2))) {
		timerStarted = true;
		
		resetTimerValues();
	}
	
	//when echo output is low, stop timer. (save time)
	if (!timeTaken && triggerSend && timerStarted && !(PINB & (1<<ECHO_PIN2))) {
		//set timer variables ot zero.
		distance2 = calculateDistance();
		message3 &= ~(0b11111<<3); //Reset distance bits
		message3 |= (distance2<<3); //Set UART msg with new distance
		
		resetTimerValues();
		timeTaken = true;
		useSensor1 = true;
		//reset trigger variables.
	}
}

//returns value in cm.
int calculateDistance() {
	// 12 is time per tick.
	//divide 1000000 to make it to meters.
	
	int uTime = timer * 12;
	float seconds = uTime / 100;
	int cenitMeters = seconds * 2;
	
	if (cenitMeters > 300)
	return 0;
	return cenitMeters;
}

bool compareSignature(uint8_t* oursignature, uint8_t* signature){
	for (int i = 0; i<3;i++){
		if (oursignature[i] != signature[i]){
			return false;
		}
	}
	return true;
}

void IRSensorFunction(){
	if (TCNT1 > 1840){ //Timer counter
		IRtimer++;
		TCNT1 = 0;
	}
	
	if (IRtimer == IRheader * 10 && bit_is_set(PINB, 6)){
		enemy = false;
	}
	if (!edge && bit_is_clear(PINB, 6)){ // Found edge
		TCNT1 = 0;
		IRtimer = 0;
		edge = true;
	}
	if (!header){ // Go into header mode if not in header already
		if(edge && bit_is_set(PINB, 6) && IRtimer >= IRheader){ // header if equal greater than ~2000uS
			header = true;
			index = 0;
		}
	}
	else{
		if (edge && bit_is_set(PINB, 6)) { //PIND & (1<<PIND7)) {
			if (IRtimer > IRzerovalue){ // 1 if longer than 700us, else 0
				signature[index] = 1;
			}
			else{
				signature[index] = 0;
			}
			index++;
		}
	}
	if (edge && bit_is_set(PINB, 6)){ // End of signal
		edge = false;
	}
	if (index == 3){
		if (compareSignature(oursignature, signature)){
			enemy = false;
		}
		else {
			enemy = true;
		}
		header = false;
		index = 0;
		IRtimer = 0;
		
		//Set UART message
		signatureS = (signature[0]<<0) | (signature[1]<<1) | signature[2];
		message1 &= ~(0b111<<IRSIGNATURE_INDEX); //Reset signature in message
		message1 |= (signatureS<<IRSIGNATURE_INDEX); //Set new signature
		(enemy ? message1 |= (1<<IRSENSOR_INDEX) : message1 &= ~(1>>IRSENSOR_INDEX));
	}
}


void laserSensorFunction(){
	//Reactivate laser 2 sec after hit
	if(TCNT3 >= 36000){
		PORTB |= (1<<PINB6);
		//inactivate
		PORTB |= (1 << LASER_AKTIVERA_PORT);
		//activate, might have to wait if this is too fast
		if(TCNT3 >= 36010){
			PORTB &= ~(1 << LASER_AKTIVERA_PORT);
			TCCR3B &= 0b11111000; //Reset pre-scaler to stop counting
			TCNT3 = 0; //Reset counter
		}
	}
	
	//mask LASER bit
	laser = (PINB>>5);
	laser &= 0b00000001;
	if(laser == 0b00000001){
		PORTB |= (1<<PINB7);
		message1 &= 0b10111111; //Reset laser bit
		message1 |= (laser<<LASER_INDEX); //Mask in new laser bit
		//Start timer with pre-scaler 1024
		TCCR3B |= (1<<CS32) | (1<<CS30);
	}
	else{
		//no hit
		message1 &= 0b10111111;
		PORTB &= ~(1<<PINB7);
	}
}

void ultrasonicFunction(){
	if (TCNT2 > 184) {
		TCNT2 = 0;
		timer++;
	}
	
	if (useSensor1) {
		
		if (!triggerSend) {
			StartPulse1();
		}
		CalculateTime1();
	}
	else {
		if (!triggerSend) {
			StartPulse2();
		}
		CalculateTime2();
	}
	
	if (timeTaken && timer == 1) {
		triggerSend = false;
		triggerStarted = false;
		timerStarted = false;
		timeTaken = false;
	}
}

//Wait for press on activation button
void waitForActivationSensor(){
	while((PIND>>PIND7) == 0){
		//Do nothing, wait for activation
		if((PINB>>PINB7) == 1){
			tape1Threshold = tape1CurrentValue - TAPE_ERROR_MARGIN; //Should possibly be +, not -
			tape2Threshold = tape2CurrentValue - TAPE_ERROR_MARGIN; //Should possibly be +, not -
		}
	}
	return;
}


/* NEW SCHEME
Meddelande 1:
Bit 0-2:	Meddelande ID (000)
Bit 3-5:	IR-signaturen
Bit 6:      Laser (1 f�r tr�ff)
Bit 7:		Aktiv IR-signatur (robot framf�r oss)

Meddelande 2:
Bit 0-2:	Meddelande ID (001)
Bit 3-7:	Fr�mre avst�ndssensorn (ca 1 dm precision)

Meddelande 3:
Bit 0-2:	Meddelande ID (010)
Bit 3-7:	Bakre avst�ndssensorn (ca 1 dm precision)

Meddelande 4:
Bit 0-2:	Meddelande ID (011)
Bit 3-7:	5 LSB Gyro (grader rotatation)

Meddelande 5:
Bit 0-2:	Meddelande ID (100)
Bit 3-5:	3 MSB Gyro (grader rotatation)
Bit 6:		Tejpsensor 1 (v�nster, 1 f�r tejp)
Bit 7:		Tejpsensor 2 (h�ger, 1 f�r tejp)

Meddelande 6:
Bit 0-2:	Meddelande ID (101)(ORDER)
Bit 3-7		ORDERID
*/