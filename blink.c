 /*
 * FileName: blink.c
 * Name: Fil2k Date: 02/01/2023 
 *
 * Description: blinks arduino internal ligh (13);	 
 */

#include <avr/io.h>
#include <util/delay.h>	

int main(void){
	//set port DDB5 as  output
	DDRB = DDRB | (1 << DDB5);

	while(1){
		PORTB = PORTB | (1 << PORTB5);
		
		_delay_ms(1000);

		PORTB = PORTB & ~(1 << PORTB5);
	
		_delay_ms(1000);
	}
}
