
// To enable intelisense for macros from avr/io.h.
#ifndef __AVR_ATmega328P__
#define __AVR_ATmega328P__
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define FOSC 16000000 // Clock Speed
#define BAUD 9600
#define MYUBRR FOSC/16/BAUD-1

volatile int direction  = 0;
volatile uint16_t speed = 200;

ISR (INT0_vect) {
    direction = !direction;
}

void delay_1ms(uint16_t ms) {
    uint16_t i;
    for(i=0;i<ms;i++) _delay_ms(1);
}

void running_leds_right(void) {
    // Run through the pins of the D port.
    PORTD = (1 << DDD3);
    delay_1ms(speed);
    PORTD = (1 << DDD4);
    delay_1ms(speed);
    PORTD = (1 << DDD5);
    delay_1ms(speed);
    PORTD = (1 << DDD6);
    delay_1ms(speed);
    PORTD = (1 << DDD7);
    delay_1ms(speed);

    // Turn off all the D pins.
    PORTD = 0;

    // Run through the pins of the B port.
    PORTB = (1 << DDB0);
    delay_1ms(speed);
    PORTB = (1 << DDB1);
    delay_1ms(speed);
    PORTB = (1 << DDB2);
    delay_1ms(speed);
    PORTB = (1 << DDB3);
    delay_1ms(speed);

    // Turn off all the B pins.
    PORTB = 0;
}

void running_leds_left(void) {
    // Run through the pins of the B port.
    PORTB = (1 << DDB3);
    delay_1ms(speed);
    PORTB = (1 << DDB2);
    delay_1ms(speed);
    PORTB = (1 << DDB1);
    delay_1ms(speed);
    PORTB = (1 << DDB0);
    delay_1ms(speed);

    // Turn off all the B pins.
    PORTB = 0;

    // Run through the pins of the D port.
    PORTD = (1 << DDD7);
    delay_1ms(speed);
    PORTD = (1 << DDD6);
    delay_1ms(speed);
    PORTD = (1 << DDD5);
    delay_1ms(speed);
    PORTD = (1 << DDD4);
    delay_1ms(speed);
    PORTD = (1 << DDD3);
    delay_1ms(speed);

    // Turn off all the D pins.
    PORTD = 0;
}

void interrupt_init(void) {
    // Enable INT0.
    EIMSK = 0x01;

    // Rising edge of INT0.
    EICRA = 0x03;

    // Enable global interrupts.
    sei();
}

void usart_init(unsigned int ubrr) {
    // Disable global interrupts.
    cli();

    // Set baud rate.
    UBRR0H = (unsigned char) (ubrr>>8);
    UBRR0L = (unsigned char) (ubrr);

    // Enable receiver and transmitter.
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);

    // Set frame format: 8 data, 2 stop bits.
    UCSR0C = (1 << USBS0) | (3 << UCSZ00);
}

void usart_transmit(unsigned char data) {
    // Wait for empty transmit buffer.
    while ( !(UCSR0A & (1 << UDRE0)) )
        ;

    // Put data into buffer, sends the data.
    UDR0 = data;
}

void usart_transmit_string(unsigned char string[8]) {
    for (int i = 0; i < 8 && string[i] != '\0'; i++) {
        usart_transmit(string[i]);
    }

    usart_transmit('\n');
}

unsigned char usart_receive(void) {
    // Wait for data to be received.
    while ( !(UCSR0A & (1 << RXC0)) )
        ;

    // Get and return received data from buffer.
    return UDR0;
}

void main(void) {
    /** 
     * Setup.
     */

    // Set these pins to output.
    DDRB = DDRB | (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3);
    // Initialize to 0, so that INT0 (pin 2) is input.
    DDRD = DDRD | (1 << DDD3) | (1 << DDD4) | (1 << DDD5) | (1 << DDD6) | (1 << DDD7);

    usart_init(MYUBRR);

    interrupt_init();

    /** 
     * Loop.
     */

    while (1) {
        if (direction) {
            running_leds_right();
        } else {
            running_leds_left();
        }

        usart_transmit_string("Running");
    }
}