#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

//distance sensor
#define TRIGPIN PORTC5
#define ECHOPIN PORTC4
#define RX_BUFFER_SIZE 128

//motors
#define DIR_L_B PB0    // pin 8
#define SPEED_A PD5    // pin 5
#define DIR_L_F PD7    // pin 7

#define DIR_R_B PB1    // pin 9
#define SPEED_B PD6    // pin 6
#define DIR_R_F PB3    // pin 11
#define MIN_SPEED 240

//servo
#define SERVO PD3
#define SERVO_MIN 4
#define SERVO_MED 12
#define SERVO_MAX 20

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
    if (PINC & (1 << ECHOPIN)) {    // Checks if echo is high
        TCNT1 = 0;                  // Reset Timer
    } else {
        timer = TCNT1;    // Save Timer value
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

void initialize_distance_sensor() {
    /*******************************************SENSOR CONTROLS*******************************************/
    DDRC |= (1 << TRIGPIN);    //A4 as trigger

    DDRC &= ~(1 << ECHOPIN);
    PORTC |= (1 << ECHOPIN);    //A5 as echo

    PRR &= ~(1 << PRTIM1);                       // To activate timer1 module
    TCNT1 = 0;                                   // Initial timer value
    TCCR1B |= (0b001 << CS10) | (1 << ICES1);    // No prescaler, trigger in rising edge

    PCICR = (1 << PCIE1);       // Enable PCINT[14:8] we use pin C5 which is PCINT13
    PCMSK1 = (1 << PCINT12);    // Enable C4 interrupt

    PORTC &= ~(1 << TRIGPIN);
    _delay_us(2);
    PORTC |= (1 << TRIGPIN);     // Set trigger high
    _delay_us(10);               // for 10uS
    PORTC &= ~(1 << TRIGPIN);    // Set trigger low
}

uint16_t measure_distance(uint8_t times) {
    uint64_t sum = 0;
    for(int i = 0; i < times; ++i){
        PORTC &= ~(1 << TRIGPIN);
        _delay_us(2);
        PORTC |= (1 << TRIGPIN);     // Set trigger high
        _delay_us(10);               // for 10uS
        PORTC &= ~(1 << TRIGPIN);    // Set trigger low
        sum += timer * 0.034 / 2;
        delay(10);
    }
    return (sum >> (times/2));
}

void send_distance(uint16_t distance) {
    uart_send_byte(distance & 0x00FF);
    uart_send_byte((distance & 0xFF00) >> 8);
}

void initialize_motors() {
    //set all motor pins as output
    DDRB |= (1 << DIR_L_B) | (1 << DIR_R_B) | (1 << DIR_L_F);
    DDRD |= (1 << SPEED_A) | (1 << SPEED_B) | (1 << DIR_R_F);

    // Turn on motor A & B (0 = on)
    PORTB |= (1 << DIR_L_B) | (1 << DIR_R_B);

    //set direction (1 = forward)
    PORTB |= (1 << DIR_L_F);
    PORTD |= (1 << DIR_R_F);

    TCCR0A |= (0b10 << COM2A0) | (0b10 << COM2B0) | (0b01 << WGM20);    // Phase correct, TOP = 0xFF
    TCCR0B |= (0 << WGM22) | (0b001 << CS20);                           //no presecaler
}

//motor speed is between 175 and 255
void set_motor_speed(uint8_t right_speed, uint8_t left_speed) {
    if (!right_speed && !left_speed) {
        OCR0A = 0;
        OCR0B = 0;
        return;
    }
    if ((OCR0B < MIN_SPEED && left_speed) && (OCR0A < MIN_SPEED && right_speed)) {
        OCR0B = 255;
        OCR0A = 255;

    } else if (OCR0A < MIN_SPEED && right_speed) {
        OCR0A = 255;
    } else if (OCR0B < MIN_SPEED && left_speed) {
        OCR0B = 255;
    }
    _delay_ms(10);
    OCR0A = right_speed;
    OCR0B = left_speed;
}

//motor direction 1 is forward 0 is backwards
void set_motor_direction(uint8_t right, uint8_t left) {
    PORTB &= ~(1 << DIR_L_B) & ~(1 << DIR_R_B) & ~(1 << DIR_R_F);
    PORTD &= ~(1 << DIR_L_F);
    if (right && left) {
        PORTB |= (1 << DIR_R_F);
        PORTD |= (1 << DIR_L_F);
    } else if (!right && !left) {
        PORTB |= (1 << DIR_R_B);
        PORTB |= (1 << DIR_L_B);
    } else if (!right) {
        PORTD |= (1 << DIR_L_F);
        PORTB |= (1 << DIR_R_B);
    } else if (!left) {
        PORTB |= (1 << DIR_R_F);
        PORTB |= (1 << DIR_L_B);
    }
}

void stop_car() {
    set_motor_speed(0, 0);
    PORTB |= (1 << DIR_R_F);
    PORTD |= (1 << DIR_L_F);
    PORTB |= (1 << DIR_R_B);
    PORTB |= (1 << DIR_L_B);
}

void increase_car_speed(uint8_t *carSpeed) {
    *carSpeed += 10;
    uart_send_byte(*carSpeed);
    if (*carSpeed < MIN_SPEED) {    //it overflows so check for min
        *carSpeed = 255;
    }
    return;
}

void decrease_car_speed(uint8_t *carSpeed) {
    *carSpeed -= 10;
    if (*carSpeed < MIN_SPEED) {
        *carSpeed = MIN_SPEED;
    }
    return;
}

void initialize_servo() {
    DDRD |= (1 << SERVO);    // Set the Servo pin as an output

    TCCR2A |= (0b01 << COM2A0) | (0b10 << COM2B0) | (0b01 << WGM20);    // Phase Correct PWM
    TCCR2B |= (1 << WGM22) | (0b111 << CS20);                           // Prescale of 1024
    OCR2A = 156;

    set_servo_angle(SERVO_MED);
}

//servo andlge is between 4 and 20
void set_servo_angle(uint8_t servo_angle) {
    OCR2B = servo_angle;
}

void find_wall(uint8_t *carSpeed) {
    uint16_t distance = measure_distance(1);
    if (distance < 300) return;
    while (distance >= 300) {
        set_motor_speed(*carSpeed, *carSpeed);
        set_motor_direction(1, 1);
        _delay_ms(100);
        stop_car();
        distance = measure_distance(1);
    }
}

void get_perpendicular(uint8_t *carSpeed) {
    uint16_t left_45 = 1000, right_45 = 0;
    uint16_t last_left = 0, last_right = 1000;

    while (1) {
        set_servo_angle(15);
        uint16_t left_45 = measure_distance(10);
        send_distance(left_45);
        _delay_ms(500);
        set_servo_angle(8);
        uint16_t right_45 = measure_distance(10);
        send_distance(right_45);
        _delay_ms(500);
        if((left_45 - right_45) < 50) break;
        if((right_45 - left_45) < 50) break;

        if ((((right_45 - last_right) < 100) && ((left_45 - last_left) < 100)) && (((last_right - right_45) < 100) && ((last_left - left_45) < 100)))
            increase_car_speed(carSpeed);
        last_right = right_45;
        last_left = left_45;

        if (left_45 > right_45){
            set_motor_speed(*carSpeed, *carSpeed);
            set_motor_direction(0, 1);
        } else {
            set_motor_speed(*carSpeed, *carSpeed);
            set_motor_direction(1, 0);
        }
        _delay_ms(500);
        stop_car();
    }
    set_servo_angle(SERVO_MED);
    return;
}

int main(void) {
    uart_init(9600);
    initialize_distance_sensor();
    initialize_motors();
    initialize_servo();
    sei();
    _delay_ms(1000);
    uint8_t data = 0;
    uint8_t carSpeed = (uint8_t)MIN_SPEED;

    stop_car();
    //find_wall(&carSpeed);
    get_perpendicular(&carSpeed);
    while (1) {
        set_motor_speed(carSpeed, carSpeed);
        if (rx_count == 0) { continue; }
        data = uart_read();
        switch (data) {
            case 10: stop_car(); break;
            case 11:
                {
                    set_motor_direction(1, 1);
                    break;
                }
            case 12:
                {
                    set_motor_direction(0, 0);
                    break;
                }
            case 13:
                {
                    set_motor_direction(1, 0);
                    break;
                }
            case 14:
                {
                    set_motor_direction(0, 1);
                    break;
                }
            case 15:
                {
                    increase_car_speed(&carSpeed);
                    break;
                }
            case 16:
                {
                    decrease_car_speed(&carSpeed);
                    break;
                }
            case 20:
                {
                    send_distance(measure_distance(10));
                    break;
                }
            case 30:
                {
                    set_servo_angle(SERVO_MIN);
                    break;
                }
            case 31:
                {
                    set_servo_angle(SERVO_MED);
                    break;
                }
            case 32:
                {
                    set_servo_angle(SERVO_MAX);
                    break;
                }
        }
    }
}

