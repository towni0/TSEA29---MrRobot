/*
 * Målsökningsenheten.c
 *
 * Created: 11/10/2015 5:30:26 PM
 *  Author: joast229
 */ 


#include "../../Robotdefinitions.h"
#include <avr/io.h>
#include <avr/interrupt.h>
//#include <util/delay.h>

//global vars for messages
unsigned message1 = 0;
unsigned message2 = 0;
unsigned message3 = 0;
unsigned message4 = 0;
unsigned message5 = 0;

void testTAPEsensors();


int main(void)
{
	
	DDRB = 0b11111111;
	
	//enable global interupts
	sei();
	
	//#UART INITS#//
	
	//initiate UART målsökning from sensor
	//set baudrate
	//4800
	uint16_t UBRR_val = UBRR_SENSOR_MALSOKNING;
	UBRR0H = (unsigned char) (UBRR_val >> 8);
	UBRR0L = (unsigned char) UBRR_val;
	//enable receive + set frame 9 bits
	UCSR0B = (1<<RXEN0) | (1<<UCSZ02);
	UCSR0C = (1<<UCSZ00) | (1<<UCSZ01);

	//enable receive interupt
	UCSR0B |= (1<<RXCIE0);
	
	
	//initiate UART målsökning to styr
	//set baudrate
	//115200
	uint16_t UBRR_val1 = UBRR_STYR_MALSOKNING;
	UBRR1H = (unsigned char) (UBRR_val1 >> 8);
	UBRR1L = (unsigned char) UBRR_val1;
	
	//enable transmit + set frame 9 bits
	UCSR1B = (1<<TXEN1) | (1<<UCSZ12);
	UCSR1C = (1<<UCSZ10) | (1<<UCSZ11);
	//#UART INITS END#//
	
	//Variables for storing sensorvalues
	uint8_t tapeSensor1 = 0;
	uint8_t tapeSensor2 = 0;
	uint8_t ultraSonicSensor1 = 0;
	uint8_t ultraSonicSensor2 = 0;
	
	unsigned nextOrder = 0;
	
	uint8_t testChange = 0;
	
	uint8_t messageNumber = 1;
	
    while(1)
    {
		//PORTB ^= (1<<PINB4);
		//PORTB = message2;
		testTAPEsensors();
		/*
		//disable interupts
		cli();
		//snaptshot values
		
		//enable interupts
		sei();
		*/
		//AI
		
		//sväng vänster:
		//nextOrder = TURN_LEFT;
		if (testChange == 0){
			nextOrder = 0x41;
			testChange = 1;
		}
		else{
			nextOrder = 0x2B;
			testChange = 0;
		}
        
		

		//check if transmit buffer is empty
		//if so send next order and reset nextOrder as in have no order
		if((UCSR1A & (1<<UDRE1))){
			//muxa through messages
			//may need to disable interupts
			//need to reset bit8 for "reasons", this bit is not shifted out and must manually be reset each time
			UCSR1B &= 0xFE;
			switch(messageNumber){
				case 1:
					//set highest bit
					UCSR1B |= (message1 >> 8) & 1;
					//set others bits
					UDR1 = message1;
					break;
				case 2:
					UCSR1B |= (message2 >> 8) & 1;
					UDR1 = message2;
					break;
				case 3:
					UCSR1B |= (message3 >> 8) & 1;
					UDR1 = message3;
					break;
				case 4:
					UCSR1B |= (message4 >> 8) & 1;
					UDR1 = message4;
					break;
				case 5: 
					UCSR1B |= (message5 >> 8) & 1;
					UDR1 = message5;
					break;
				case 6:
					if(nextOrder != 0){
						UCSR1B |= (nextOrder >> 8) & 1;
						UDR1 = nextOrder;
						nextOrder = 0;
					}
				default:
					break;
			}
			//next mux
			messageNumber++;
			if(messageNumber>NUMBER_OF_MESSAGES+1) messageNumber=1;
		}
    }
}

ISR(USART0_RX_vect){
	
	//may need to disable interupts
	//mask "bit8"
	uint16_t buffer = UCSR0B & 0x02;
	//shift to position 8
	buffer = buffer << 7;
	//or in rest of bits
	buffer |= UDR0;
	uint8_t messageID = buffer & 0x07;
	//PORTB |= (1<<PINB4);
	switch(messageID){
		case 0:
			//_delay_ms(500);
			//PORTB ^= (1<<PINB4);
			message1 = buffer;
			break;
		case 1:
			//PORTB ^= (1<<PINB5);
			message2 = buffer;
			break;
		case 2:
			//_delay_ms(500);
			//PORTB ^= (1<<PINB6);
			message3 = buffer;
			break;
		case 3:
			//PORTB ^= (1<<PINB7);
			message4 = buffer;
			break;
		case 4:
			//PORTB ^= (1<<PINB7);
			message5 = buffer;
			break;
	}
}
	
//just test function
//PIN B4 and B5 as outputs
void testTAPEsensors(){
	//AD omvandlinar 2 TAPEsensorer + gyro med interupts
	unsigned snapshot = 0b0000000010000000&message1;
	
	if(snapshot == 0b0000000010000000){
		PORTB |= (1<<PINB4);
	}
	else{
		PORTB &= 0b11101111;
	}
	
	snapshot = 0b0000000100000000&message1;
	
	if(snapshot == 0b0000000100000000){
		PORTB |= (1<<PINB5);
	}
	else{
		PORTB &= 0b11011111;
	}
}
