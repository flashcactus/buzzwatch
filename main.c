#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "shift.h"

#define POT_ENABLE_PIN 2
#define POT_PIN 3 //ADCx, not PBx! (though here they match)
#define BUZZ_PIN 4
#define BTN_PIN 1
#define SHIFT_PIN 0

#define POT_FREQ 2  //check the pot every 2^n ticks of delay_ctr
#define POT_FMASK ~((~0) << POT_FREQ) //00..01..1, POT_FREQ ones
#define BUZZ_PATT 0b1101

//globals
uint8_t prescale_mask=0;
uint8_t prescale_ctr;
uint8_t delay_ctr;
uint8_t ovf_ticks; //counts timer overflows [see timsetup()]

uint8_t pot_value;
//uint8_t count_mode;

uint16_t buzz_pattern;

inline void timsetup() {
    TCCR0B |= (1<<CS02);    //set prescaler to 1/256. 
                            //@1MHz:
                            //  tick=.25 ms; 
                            //  overflow=65.5ms
                            //  1s=15.25ovf;  
    
    TIMSK0 |= (1<<TOIE0);        //enable the interrupt

    ovf_ticks=0;            //zero the timer&overflow.
    TCNT0=0;
}

int main(void)
{
    //disable interrupts before setup
    cli();
    
    //--- begin setup ---

    if(MCUSR & (1<<WDRF)){              // If a reset was caused by the Watchdog Timer...
        MCUSR &= ~(1<<WDRF);            // Clear the WDT reset flag
        WDTCR |= (1<<WDCE) | (1<<WDE);  // Enable the WD Change Bit
        WDTCR = 0x00;                   // Disable the WDT
    }
    
    //setup the WDT properly now
    WDTCR |= (1<<WDCE) | (1<<WDE);      // Enable the WDT Change Bit
    WDTCR = (1<<WDTIE) |                //Enable WDT Interrupt
            //(1<<WDP1) | (1<<WDP2);    // Set Timeout to ~1s
            (1<<WDP1)|(1<<WDP0);        //.125s
            //0; //speed up 64x for debug
    //setup pins
    DDRB |= (1<<POT_ENABLE_PIN) | (1<<BUZZ_PIN) | (1<<SHIFT_PIN);  
    DDRB &= ~(1<<POT_PIN) & ~(1<<BTN_PIN);

    //setup the ADC
    ADMUX &= ~(1 << REFS0);                 //use VCC as ref voltage
    ADMUX |= (1<<ADLAR);                    //left-adjust result (for 8 MSBs in ADCH); 
    ADMUX |= POT_PIN;                       //select pin ADC2 (PB4)
    ADCSRA |= (1 << ADPS1) | (1 << ADPS0);  //set prescaler to 1/8 (125 KHz)
    ADCSRA |= (1 << ADIE);                  //enable interrupt
    ADCSRA |= (1 << ADEN);                  //enable the ADC

    //setup sleep
    //we want the deepest sleep 
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    //init globals
    delay_ctr = 0;
    buzz_pattern = 0;
    
    //setup the timer
    timsetup();
    
    //--- setup complete; re-enable interrupts ---
    sei();
    
    //start main loop
    while(1) {
        if (                        //sleep-allowed conditions (all at once):
            !(ADCSRA & (1<<ADSC))   //  ADC not currently converting
            && MCUCR & (1<<SE)      //  sleep-enable bit is set (WDT routine finished)
            && !buzz_pattern        //  buzzed everything already
        ){
            //turn everything off, to be sure
            PORTB &= ~( (1<<BUZZ_PIN) | (1<<POT_ENABLE_PIN) ) ;
            
            // Disable BOD
            cli(); 
            sleep_bod_disable();    
            sei(); 
            
            // Go to Sleep
            sleep_cpu();            
        }
    } 
}


ISR(WDT_vect){
    sleep_disable(); 

    if (! (prescale_ctr&POT_FMASK) ){   //check if time to look at the pot
        //get a new potvalue
        PORTB |= (1<<POT_ENABLE_PIN);       //put some current into the pot 
        ADCSRA |= (1<<ADEN) | (1<<ADSC);    //start ADC conversion
    }
    
    //increment after that check to escape deadlock @ pot_value = 0 (which will never update)
    ++prescale_ctr;
    //do the sleepscaling
    if(! (prescale_ctr&prescale_mask)) {
        ++delay_ctr;
        prescale_ctr = 0;
    }
    //check whether it's time to buzz
    if(delay_ctr >= pot_value) {
        delay_ctr=0;
        buzz_pattern = BUZZ_PATT;   //push the pattern
        timsetup();                 //start the timer
    }
    
    sleep_enable();
}

ISR(TIM0_OVF_vect){
    ++ovf_ticks;
    shift_out_1w(ovf_ticks, &PORTB, 1<<SHIFT_PIN, 10);
    shift_out_1w(delay_ctr, &PORTB, 1<<SHIFT_PIN, 10);
    if(buzz_pattern&1) {
        PORTB |= (1<<BUZZ_PIN);   //turn on
    } else {
        PORTB &= ~(1<<BUZZ_PIN);  //turn off
    }
    buzz_pattern = buzz_pattern>>1;
}


ISR(ADC_vect){                      //conversion complete
    PORTB &= ~(1<<POT_ENABLE_PIN);  //turn the pot off
    pot_value = ADCH;               //get the value
    ADCSRA &= ~(1<<ADEN);           //kill the ADC
    //wait_len = pot_value;
}
