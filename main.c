#include <avr/io.h>
#include <avr/interrupt.h>


uint8_t cycles;


int main(void)
{
    cli();
    if(MCUSR & (1<<WDRF)){            // If a reset was caused by the Watchdog Timer...
        MCUSR &= ~(1<<WDRF);                // Clear the WDT reset flag
        WDTCR |= (1<<WDCE) | (1<<WDE);     // Enable the WD Change Bit
        WDTCR = 0x00;                      // Disable the WDT
    }
    
    //setup the WDT properly now
    WDTCR |= (1<<WDCE) | (1<<WDE);     // Enable the WDT Change Bit
    WDTCR = (1<<WDTIE) |                //Enable WDT Interrupt
             (1<<WDP2) | (1<<WDP1);     // Set Timeout to ~1s
    /*
    TCCR0B |= 1<<CS00 | 1<<CS02;  //Divide by 1024
    TIMSK0 |= 1<<TOIE0;     //enable timer overflow interrupt
    */
    sei();

    DDRB |= (1<<3);  
    PORTB |= (1<<3);

    while(1) {}    
}


ISR(WDT_vect)
{
    /*
    if ( cycles < 8 ) {
        ++cycles;
    } else {
      */  cycles = 0;
        PORTB ^= (1<<3);
    //}
}
