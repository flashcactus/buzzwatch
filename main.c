#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define POT_ENABLE_PIN 3
#define POT_PIN 4
#define BUZZ_PIN 0

uint8_t cycles;
uint8_t buzz_on;
#define BUZZ_LEN 1

void shift_out(uint8_t b, uint8_t data_pins, uint8_t clk_pins) { //LSB first
    uint8_t bb = b;
    for(uint8_t i = 8; i>0; --i) {
        DDRB |= data_pins | clk_pins;
        PORTB &= ~clk_pins;     //pull clock down first
        if(bb%2){               //write bit
            PORTB |= data_pins;
        } else {
            PORTB &= ~data_pins;
        }
        PORTB |= clk_pins;      //clock
        bb = bb>>1;
    }
}

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

    DDRB |= (1<<POT_ENABLE_PIN) | (1<<BUZZ_PIN);  
    DDRB = 0xff;
    DDRB &= ~(1<<POT_PIN);

    //init globals
    cycles = 0;
    buzz_on = 0;

    while(1) {
        if (MCUCR & (1<<SE)) {//sleep enable bit is set
            cli(); 
            sleep_bod_disable();   // Disable BOD
            sei(); 
            sleep_cpu();           // Go to Sleep

        }
    } 
}

#define DATA_P 1
#define CLK_P 2
#define MV_P 0
ISR(WDT_vect){
    sleep_disable(); 
    PORTB ^= (1<<POT_ENABLE_PIN);
    
    PORTB &= ~(1<<MV_P); //pull down preemptively
    ++cycles; //count
    /*
    if(cycles > 3) {
        cycles = 0;
        PORTB |= (1<<BUZZ_PIN);
        buzz_on = BUZZ_LEN;
    }
   
    if(!buzz_on) {
        PORTB &= ~(1<<BUZZ_PIN);  //turn off
    } else {
        --buzz_on; //for the next cycle
    }*/

    shift_out(cycles, 1<<DATA_P, 1<<CLK_P); //shift out
    PORTB |= (1<<MV_P); //move to output register

    sleep_enable();
}
