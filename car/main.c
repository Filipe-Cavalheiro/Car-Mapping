#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

#define TRIGPIN PORTC5
#define ECHOPIN PORTC4
#define RX_BUFFER_SIZE 128

void uart_init(uint32_t baud);
void uart_send_byte(uint8_t c);
void uart_send_string(uint8_t *c);

volatile static uint8_t rx_buffer[RX_BUFFER_SIZE] = { 0 };
volatile static uint16_t rx_count = 0;
volatile static uint8_t uart_tx_busy = 1;
volatile float timer;

ISR(USART_RX_vect) {
  volatile static uint16_t rx_write_pos = 0;

  rx_buffer[rx_write_pos] = UDR0;
  rx_count++;
  rx_write_pos++;
  if (rx_write_pos >= RX_BUFFER_SIZE) {
    rx_write_pos = 0;
  }
}

ISR(USART_TX_vect) {
  uart_tx_busy = 1;
}

/*******************************************INTURRUPT (SENSOR CONTROLS) PCINT1 FOR PIN C4*******************************************/
ISR(PCINT1_vect) {
  if (PINC & (1 << ECHOPIN)) {  // Checks if echo is high
    TCNT1 = 0;                  // Reset Timer
  } else {
    timer = TCNT1;  // Save Timer value
  }
}

void uart_init(uint32_t baud) {
  baud = (F_CPU / (16 * baud)) - 1;

  UBRR0H = (baud & 0x0F00) >> 8;
  UBRR0L = (baud & 0x00FF);

  UCSR0B |= (1 << TXEN0) | (1 << RXEN0) | (1 << TXCIE0) | (1 << RXCIE0);

  //8 bist
  UCSR0C = (0b11 << UCSZ00);
}


void uart_send_byte(uint8_t c) {
  while (uart_tx_busy == 0)
    ;
  uart_tx_busy = 0;
  UDR0 = c;
}

uint8_t uart_read() {
  static uint16_t rx_read_pos = 0;
  uint8_t data = 0;

  data = rx_buffer[rx_read_pos];
  rx_read_pos++;
  rx_count--;
  if (rx_read_pos >= RX_BUFFER_SIZE) {
    rx_read_pos = 0;
  }
  return data;
}

void start_distance_sensor(){
  /*******************************************SENSOR CONTROLS*******************************************/
  DDRC |= (1 << TRIGPIN);  //A4 as trigger

  DDRC &= ~(1 << ECHOPIN);
  PORTC |= (1 << ECHOPIN);  //A5 as echo

  PRR &= ~(1 << PRTIM1);                     // To activate timer1 module
  TCNT1 = 0;                                 // Initial timer value
  TCCR1B |= (0b001 << CS10) | (1 << ICES1);  // No prescaler, trigger in rising edge

  PCICR = (1 << PCIE1);     // Enable PCINT[14:8] we use pin C5 which is PCINT13
  PCMSK1 = (1 << PCINT12);  // Enable C4 interrupt

  PORTC &= ~(1 << TRIGPIN);
  _delay_us(2);
  PORTC |= (1 << TRIGPIN);   // Set trigger high
  _delay_us(10);             // for 10uS
  PORTC &= ~(1 << TRIGPIN);  // Set trigger low
}

uint16_t measure_distance(){
  PORTC &= ~(1 << TRIGPIN);
  _delay_us(2);
  PORTC |= (1 << TRIGPIN);   // Set trigger high
  _delay_us(10);             // for 10uS
  PORTC &= ~(1 << TRIGPIN);  // Set trigger low
  return timer * 0.034 / 2;
}

int main(void) {
  uart_init(9600);
  start_distance_sensor();
  sei();

  uint16_t distance;
  int numb;
  while (1) {
    if (rx_count == 0) {continue;}
    uart_read();
    distance = measure_distance();
    for (numb = 0; distance >= 1000; ++numb, distance -= 1000)
      ;
    uart_send_byte(numb + '0');
    for (numb = 0; distance >= 100; ++numb, distance -= 100)
      ;
    uart_send_byte(numb + '0');
    for (numb = 0; distance >= 10; ++numb, distance -= 10)
      ;
    uart_send_byte(numb + '0');
    for (numb = 0; distance >= 1; ++numb, --distance)
      ;
    uart_send_byte(numb + '0');
    uart_send_byte('\n');
    /*
		uart_send_byte((distance & 0xFF00) >> 8);
		uart_send_byte(distance & 0x00FF);
		uart_send_byte('\n');
    */
  }
}
