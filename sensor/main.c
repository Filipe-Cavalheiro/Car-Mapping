#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define TRIGPIN PORTC4
#define ECHOPIN PORTC5

volatile float timer;

/*******************************************INTURRUPT PCINT1 FOR PIN C5*******************************************/
ISR(PCINT1_vect){
	if (PINC & (1 << PORTC5)){					// Checks if echo is high
		TCNT1 = 0;								// Reset Timer
	} 
  else {
	  timer = TCNT1;					        // Save Timer value
	}
}

int main(){
    DDRC |=  (1 << TRIGPIN); //A4 as trigger

    DDRC &= ~(1 << ECHOPIN); 
    PORTC |= (1 << ECHOPIN); //A5 as echo

	PRR &= ~(1<<PRTIM1);					    // To activate timer1 module
	TCNT1 = 0;								    // Initial timer value
	TCCR1B |= (0b001 << CS10) | (1 << ICES1);	// No prescaler, trigger in rising edge

	PCICR = (1<<PCIE1);						    // Enable PCINT[14:8] we use pin C5 which is PCINT13
	PCMSK1 = (1<<PCINT13);				        // Enable C5 interrupt

	sei();									    // Enable Global Interrupts

    int distance;
    uint8_t str[] = "Distance: ";
    while(1){
        PORTC &= ~(1 << TRIGPIN);
        _delay_us(2);
		PORTC |=  (1 << TRIGPIN);				// Set trigger high
		_delay_us(10);							// for 10uS
		PORTC &= ~(1 << TRIGPIN);				// Set trigger low

        distance = timer * 0.024 / 2;
        _delay_ms(1000);
    }
    return 0;
}
