#include <avr/io.h>
#include <util/delay.h>

#define SERVO PD3
#define SERVO_MIN 4
#define SERVO_MED 12
#define SERVO_MAX 20

int main(){
    DDRD |= (1 << SERVO); // Set the Servo pin as an output
    
    TCCR2A |= (0b01 << COM2A0) |(0b10 << COM2B0) | (0b01 << WGM20); // Phase Correct PWM
    TCCR2B |= (1 << WGM22) | (0b111 << CS20); // Prescale of 1024
    OCR2A = 156; 

    while(1){   
        OCR2B = SERVO_MIN;
        _delay_ms(1000);
        OCR2B = SERVO_MED;
        _delay_ms(1000);
        OCR2B = SERVO_MAX;
        _delay_ms(1000);
        OCR2B = SERVO_MED;
        _delay_ms(1000);
    }

    return 0;
}
