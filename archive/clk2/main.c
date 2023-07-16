#include <avr/io.h>
#include <util/delay.h>

#define SERVO_PIN PB1
#define PWM_FREQUENCY 50 //50Hz
#define PWM_TOP ((F_CPU / (PWM_FREQUENCY * 8)) - 1)
#define SERVO_MIN 999 // 500 Microseconds 0 deg
#define SERVO_MED 2999 // 90 deg
#define SERVO_MAX 4999 // 2400 Microseconds 180 deg

int main(){
    DDRB |= (1 << SERVO_PIN); // Set the Servo pin as an output

    TCCR1A = (0b10 << COM1A0) | (0b00 << COM1B0) | (0b10 << WGM10); // Fast PWM mode ICR1A = TOP
    TCCR1B = (0b11 << WGM12) | (0b010 << CS10); // Prescaler of 8
    ICR1 = PWM_TOP;

    while(1){
        OCR1A = SERVO_MIN;
        _delay_ms(1000);
        OCR1A = SERVO_MED;
        _delay_ms(1000);
        OCR1A = SERVO_MAX;
        _delay_ms(1000);
        OCR1A = SERVO_MED;
        _delay_ms(1000);
    }

  return 0;
}
