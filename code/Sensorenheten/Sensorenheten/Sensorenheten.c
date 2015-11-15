/*
 * Sensorenheten.c
 *
 * Created: 11/9/2015 1:39:31 PM
 *  Author: joast229
 */ 

#include "../../Robotdefinitions.h"

#define GYRO		0
#define TAPESENSOR1 1
#define TAPESENSOR2 2

//initial values are set to their IDs from design spec. (these bits are never changed)
unsigned message1 = 0;
unsigned message2 = 1;
unsigned message3 = 2;
unsigned message4 = 3;
unsigned message5 = 4;

uint16_t tapeThreshold = 500;

uint8_t ADCcount = 1;

uint8_t messageNumber = 1;

uint8_t tapeCheck(uint16_t message);
void clearADCMUX();
void testTAPEsensors();

int main(void)
{
	
	//init µcontroller
	
	DDRB = 0b11111111;
		
	
		
	//enable global interupt
	sei();
	
	//#ADC INITS#//
		
	//turn on ADC
	ADCSRA |= (1 << ADEN);
	//Set ADC clock to 1000 000 / 16 Hz
	ADCSRA |= (1 << ADPS2);
	//ADC interupt enabled
	ADCSRA |= (1 << ADIE);
	//mux PINA0, start with TAPE
	ADMUX |= (1<<PINA0);
	//Start 1st conversion
	ADCSRA |= (1 << ADSC);
	
	//#UART INITS#//
	
	//initiate UART målsökning
	//set baudrate
	//4800
	uint16_t UBRR_val = UBRR_SENSOR_MALSOKNING;
	UBRR0H = (unsigned char) (UBRR_val >> 8);
	UBRR0L = (unsigned char) UBRR_val;
	//enable transmit + set frame 9 bits
	UCSR0B = (1<<TXEN0) | (1<<UCSZ02);
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);
	//enable transmit interupt
	UCSR0B |= (1<<TXCIE0);
	//start first transmission	
	UDR0 = 0x00;
    while(1)
    {
		testTAPEsensors();
	}
	
}

ISR(USART0_TX_vect){
	//muxa through messages
	//disable interupts
	cli();
	//need to reset bit8 for "reasons", this bit is not shifted out and must manually be reset each time
	UCSR0B &= 0xFE;
	switch(messageNumber){
		case 1:
			//set highest bit
			UCSR0B |= (message1 >> 8) & 1;
			//set others bits
			UDR0 = message1;
			break;
		case 2:
			UCSR0B |= (message2 >> 8) & 1;
			UDR0 = message2;
			break;
		case 3:
			UCSR0B |= (message3 >> 8) & 1;
			UDR0 = message3;
			break;
		case 4:
			UCSR0B |= (message4 >> 8) & 1;
			UDR0 = message4;
			break;
		case 5: 
			UCSR0B |= (message5 >> 8) & 1;
			UDR0 = message5;
			break;
		default:
			break;
	}
	//next mux
	messageNumber++;
	if(messageNumber>NUMBER_OF_MESSAGES) messageNumber=1;
	//enable interupts
	
	sei();
}

ISR(ADC_vect){
	uint8_t lowbits = ADCL;
	uint16_t message = ADCH <<8 | lowbits;
	

	switch(ADCcount){
		case GYRO:
			clearADCMUX();
			ADMUX |= 1;
			break;
		case TAPESENSOR1:
			//clear TAPESENSOR1_INDEX bit
			message1 &= 0b1111111101111111;
			//mask in actual value
			message1 |= (tapeCheck(message)<<TAPESENSOR1_INDEX);
			//next muxed ADC
			clearADCMUX();
			ADMUX |= 2;
			break;
		case TAPESENSOR2:
			//clear TAPESENSOR2_INDEX bit
			message1 &= 0b1111111011111111;
			//mask in actual value
			message1 |= (tapeCheck(message)<<TAPESENSOR2_INDEX);
			//next muxed ADC
			clearADCMUX();
			//MUX to IRSENSOR-ADC?
			break;
		default:
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

uint8_t tapeCheck(uint16_t message){
	if(message >= tapeThreshold){
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
//PIN B0 and B1 as outputs
void testTAPEsensors(){
	//AD omvandlinar 2 TAPEsensorer + gyro med interupts
	unsigned snapshot = 0b0000000010000000&message1;
			
	if(snapshot == 0b0000000010000000){
		PORTB |= (1<<PINB0);
	}
	else{
		PORTB &= 0b11111110;
	}
			
	snapshot = 0b0000000100000000&message1;
			
	if(snapshot == 0b0000000100000000){
		PORTB |= (1<<PINB1);
	}
	else{
		PORTB &= 0b11111101;
	}
}

/*
Meddelande 1:
Bit 0-2:	Meddelande ID (000)
Bit 3-5:	IR-signaturen
Bit 6:		Aktiv IR-signatur (robot framför oss)
Bit 7:		Tejpsensor 1 (vänster, 1 för tejp)
Bit 8:		Tejpsensor 2 (höger, 1 för tejp)

Meddelande 2:
Bit 0-2:	Meddelande ID (001)
Bit 3-7:	Främre avståndssensorn (ca 1 dm precision)
Bit 8  :    Laser (1 för träff)

Meddelande 3:
Bit 0-2:	Meddelande ID (010)
Bit 3-7:	Bakre avståndssensorn (ca 1 dm precision)

Meddelande 4:
Bit 0-2:	Meddelande ID (011)
Bit 3-8:	6 LSB Gyro (grader rotatation)

Meddelande 5: 
Bit 0-2:	Meddelande ID (100)
Bit 3-4:	2 MSB Gyro (grader rotatation)

Meddelande 6:
Bit 0-2:	Meddelande ID (101)(ORDER)
Bit 3-8		ORDERID
*/