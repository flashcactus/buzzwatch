#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

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
    sei();
   
    //we want the deepest sleep 
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    DDRB |= (1<<3);  
    PORTB |= (1<<3);

    while(1) {
        if (MCUCR & (1<<SE)) {//sleep enable bit is set
            cli(); 
            sleep_bod_disable();   // Disable BOD
            sei(); 
            sleep_cpu();           // Go to Sleep

        }
    } 
}


ISR(WDT_vect){
    sleep_disable(); 
    PORTB ^= (1<<3);
    sleep_enable();
}
