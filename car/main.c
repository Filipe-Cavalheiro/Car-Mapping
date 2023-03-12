#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include <stdint.h>

#define TRIGPIN PORTC5
#define ECHOPIN PORTC4

void uart_init(uint32_t baud);
void uart_send_byte(uint8_t c);
void uart_send_string(uint8_t *c);

volatile static uint8_t uart_tx_busy = 1;
volatile float timer;

ISR(USART_TX_vect){
        uart_tx_busy = 1;
}

/*******************************************INTURRUPT (SENSOR CONTROLS) PCINT1 FOR PIN C4*******************************************/
ISR(PCINT1_vect){
        if (PINC & (1 << ECHOPIN)){                                      // Checks if echo is high
                TCNT1 = 0;                                               // Reset Timer
        } 
  else {
          timer = TCNT1;                                                // Save Timer value
        }
}

void uart_init(uint32_t baud){
        baud = (F_CPU/(16*baud)) - 1;

        UBRR0H = (baud & 0x0F00) >> 8;
        UBRR0L = (baud & 0x00FF);

        UCSR0B |= (1 << TXEN0) | (1 << TXCIE0);

  //8 bist  
  UCSR0C = (0b11 << UCSZ00);
}


void uart_send_byte(uint8_t c){
        while(uart_tx_busy == 0);
        uart_tx_busy = 0;
        UDR0 = c;
}

void uart_send_string(uint8_t *c){
        uint16_t i = 0;
        do{
                uart_send_byte(c[i]);
                i++;
        }while(c[i] != '\0');
}

int main(void){
  uint8_t start[] = "Program Start\n\0";
  uart_init(9600);

  /*******************************************SENSOR CONTROLS*******************************************/
  DDRC |=  (1 << TRIGPIN); //A4 as trigger

  DDRC &= ~(1 << ECHOPIN); 
  PORTC |= (1 << ECHOPIN); //A5 as echo

  PRR &= ~(1<<PRTIM1);                                        // To activate timer1 module
  TCNT1 = 0;                                                                  // Initial timer value
  TCCR1B |= (0b001 << CS10) | (1 << ICES1);       // No prescaler, trigger in rising edge

  PCICR = (1<<PCIE1);                                                 // Enable PCINT[14:8] we use pin C5 which is PCINT13
  PCMSK1 = (1<<PCINT12);                                  // Enable C4 interrupt
  uint16_t distance;
  
  int numb;
  
  sei();                                                                      // Enable Global Interrupts
      
  uart_send_string(start);
  while (1){
    PORTC &= ~(1 << TRIGPIN);
    _delay_us(2);
    PORTC |=  (1 << TRIGPIN);                               // Set trigger high
    _delay_us(10);                                                  // for 10uS
    PORTC &= ~(1 << TRIGPIN);                               // Set trigger low

    distance = timer * 0.034 / 2;

    for(numb = 0; distance >= 1000; ++numb, distance -= 1000);
    uart_send_byte(numb + '0');
    for(numb = 0; distance >= 100; ++numb, distance -= 100);
    uart_send_byte(numb + '0');
    for(numb = 0; distance >= 10; ++numb, distance -= 10);
    uart_send_byte(numb + '0');
    for(numb = 0; distance >= 1; ++numb, --distance);
    uart_send_byte(numb + '0');
    uart_send_byte('\n');
    /*
    uart_send_byte((distance & 0xFF00) >> 8);
    uart_send_byte(distance & 0x00FF);
    uart_send_byte('\n');
    */
    _delay_ms(200);
  }
}
