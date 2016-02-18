#include <avr/io.h>
#include <avr/interrupt.h>


uint8_t cycles;


int main(void)
{

    cli(); 
    TCCR0B |= 1<<CS00 | 1<<CS02;  //Divide by 1024
    TIMSK0 |= 1<<TOIE0;     //enable timer overflow interrupt
    sei();


    DDRB |= (1<<3);  
    PORTB |= (1<<3);

    while(1) {}    
}



ISR(TIM0_OVF_vect)     //timer0 overflow
{
    if ( cycles < 8 ) {
        ++cycles;
    } else {
        cycles = 0;
        PORTB ^= (1<<3);
    }
}
