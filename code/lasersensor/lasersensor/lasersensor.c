/*
 * lasersensor1.c
 *
 * Created: 11/3/2015 3:46:19 PM
 *  Author: joast229
 */ 

#define F_CPU 1000000UL

#include <avr/io.h>
#include <util/delay.h>


#define AKTIVERA 4

int main(void){
	//bit 4 = !AKTIVERA
	//DDRB = 0b00010001;
	DDRB |= (1<< PINB4);
	DDRB |= (1<< PINB0);
	
	//PORTB |= (1 << AKTIVERA);
	//_delay_us(100);
	//PORTB &= (0 << AKTIVERA);
	
	
    while(1){
		//mask LASER bit
		uint8_t laser = PINB;
		laser &= 0b00100000; 
		//(PINB & (1<<PINB5))
		if(laser == 0x20){
			//light led
			PORTB |= (1 << PINB0);
			//wait 2 secs
			//inactivate
			PORTB |= (1 << AKTIVERA);
			//unlight led
			PORTB &= (0 <<PINB0);
			//activate
			PORTB &= (0 << AKTIVERA);
			
		}
    }
}
