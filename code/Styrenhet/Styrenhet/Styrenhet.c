/*
 * Styrenhet.c
 *
 * Created: 11/11/2015 5:47:39 PM
 *  Author: joast229
 */ 

#include "../../Robotdefinitions.h"

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

    while(1)
    {
		if(currentOrder == 0x41){
			PORTB |= (1<<PINB4);
		}
		else{
			PORTB &= 0;
		}
         //Do command
    }
}

ISR(USART0_RX_vect){
	uint8_t messageID = UDR0 & 0x07;
	//only look at ORDERS
	if(messageID == ORDER_ID){
		currentOrder = UDR0;
	}
}