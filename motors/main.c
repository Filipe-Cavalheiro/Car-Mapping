#include <avr/io.h>
#include <util/delay.h>

#define EN_A PB0    // pin 8
#define SPEED_A PD5 // pin 5
#define DIR_A PD7   // pin 7

#define EN_B PB1    // pin 9
#define SPEED_B PD6 // pin 6
#define DIR_B PB3   // pin 11


int main(void) {
    //set all motor pins as output
    DDRB |= (1 << EN_A) | (1 << EN_B) | (1 << DIR_A);
    DDRD |= (1 << SPEED_A) | (1 << SPEED_B) | (1 << DIR_B);

    //set direction (1 = forward)
    PORTB |= (1 << DIR_B);
    PORTD |= (1 << DIR_A);

    // Turn on motor A & B (0 = on)
    PORTB &= ~(1 << EN_A) & ~(1 << EN_B);
    
    
    TCCR0A |= (0b10 << COM2A0) |(0b10 << COM2B0) | (0b01 << WGM20); // Phase correct, TOP = 0xFF
    TCCR0B |= (0 << WGM22) | (0b001 << CS20); //no presecaler
    OCR0A = 255; 
    OCR0B = 255; 
    _delay_ms(50); 
    
    //values of speed range form 175 to 255 (but why????)
    while(1){
        OCR0A = 175; 
        OCR0B = 175; 
    }

  return 0;
}
