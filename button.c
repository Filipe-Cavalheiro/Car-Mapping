/*
 * @author      : fil2k 
 * @created     : Wednesday Jan 04, 2023 15:29:08 GMT
 * @file        : button
 */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

int main(){
	EICRA = 0x00;
	EIMSK = 0x00;
	cli();

	DDRD &= ~(1 << PORTD2);
	PORTD |= (1 << PORTD2);
	
	DDRB |= (1 << PORTB5);
	PORTB |= (1 << PORTB5);

	EICRA |= (1 << ISC01);  
	EICRA &= ~(1 << ISC00);		// 0b00000010 set wanted flags (falling level interrupt)
	EIMSK |= (1 << INT0);  		// 0b00000001 enable interrupt bits on mask
	sei();				// set interrupt
	
	while(1){
	}
}

ISR(INT0_vect){
	PORTB ^= (1 << PORTB5);
}
