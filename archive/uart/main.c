#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

void uart_init(uint32_t baud,uint8_t high_speed);
void uart_send_byte(uint8_t c);
void uart_send_string(uint8_t *c);

volatile static uint8_t uart_tx_busy = 1;

ISR(USART_TX_vect){
	uart_tx_busy = 1;
}

void uart_init(uint32_t baud,uint8_t high_speed){
	
	uint8_t speed = 16;
	
	if(high_speed != 0){
		speed = 8;
		UCSR0A |= 1 << U2X0;
	}
	
	baud = (F_CPU/(speed*baud)) - 1;
	
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
	const uint8_t start[] = "Program Start\n\0";
	uint8_t data = 'A';
	
	uart_init(9600,1);

	sei();
	uart_send_string(start);
    while (1){
			uart_send_byte(data);
      ++data;
      if(data == 'Z' + 1)
        data = 'A';
      _delay_ms(1000);
    }
}
