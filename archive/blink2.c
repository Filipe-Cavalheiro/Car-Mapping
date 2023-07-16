/*
 * FileName: blink2.c
 * Name: Fil2k Date: 02/01/2023 
 *
 * Description: blinks light with digital pin 5;	 
 */

#include <avr/io.h>
#include <util/delay.h>	

int main(void){
	//set port DDB5 as  output
	DDRD |= (1 << DDD5);

	while(1){
		PORTD |= (1 << PORTD5);
		
		_delay_ms(1000);

		PORTD &= ~(1 << PORTD5);
	
		_delay_ms(1000);
	}
}
