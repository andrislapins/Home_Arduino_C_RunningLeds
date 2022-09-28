#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

volatile int direction = 0;

void running_leds_right(void) {
    // Run through the pins of the D port.
    PORTD = (1 << DDD3);
    _delay_ms(200);
    PORTD = (1 << DDD4);
    _delay_ms(200);
    PORTD = (1 << DDD5);
    _delay_ms(200);
    PORTD = (1 << DDD6);
    _delay_ms(200);
    PORTD = (1 << DDD7);
    _delay_ms(200);

    // Turn off all the D pins.
    PORTD = 0;

    // Run through the pins of the B port.
    PORTB = (1 << DDB0);
    _delay_ms(200);
    PORTB = (1 << DDB1);
    _delay_ms(200);
    PORTB = (1 << DDB2);
    _delay_ms(200);
    PORTB = (1 << DDB3);
    _delay_ms(200);

    // Turn off all the B pins.
    PORTB = 0;
}

void running_leds_left(void) {
    // Run through the pins of the B port.
    PORTB = (1 << DDB3);
    _delay_ms(200);
    PORTB = (1 << DDB2);
    _delay_ms(200);
    PORTB = (1 << DDB1);
    _delay_ms(200);
    PORTB = (1 << DDB0);
    _delay_ms(200);

    // Turn off all the B pins.
    PORTB = 0;

    // Run through the pins of the D port.
    PORTD = (1 << DDD7);
    _delay_ms(200);
    PORTD = (1 << DDD6);
    _delay_ms(200);
    PORTD = (1 << DDD5);
    _delay_ms(200);
    PORTD = (1 << DDD4);
    _delay_ms(200);
    PORTD = (1 << DDD3);
    _delay_ms(200);

    // Turn off all the D pins.
    PORTD = 0;
}

void main(void) {
    /** 
     * Setup.
     */

    // Set these pins to output.
    DDRB = DDRB | (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB3);
    // Initialize to 0, so that INT0 (pin 2) is input.
    // DDRD = 0;
    DDRD = DDRD | (1 << DDD3) | (1 << DDD4) | (1 << DDD5) | (1 << DDD6) | (1 << DDD7);

    // Enable INT0.
    EIMSK = 0x01;
    // Rising edge of INT0.
    EICRA = 0x03;
    
    // Enable global interrupts.
    sei();

    /** 
     * Loop.
     */

    while (1) {
        if (direction) {
            running_leds_right();
        } else {
            running_leds_left();
        }
    }
}

ISR (INT0_vect)
{
    direction = !direction;
}