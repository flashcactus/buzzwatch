#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include "shift.h"

#define POT_ENABLE_PIN 2
#define POT_PIN 3 //ADCx, not PBx!
#define BUZZ_PIN 4
#define BTN_PIN 1
#define SHIFT_PIN 0

#define POT_FREQ 2 //check the pot every 2^n delay_ctr
#define BUZZ_PATT 0b1010001101100011

//globals
uint8_t prescale_mask=0;
uint8_t prescale_ctr;
uint8_t delay_ctr;
//uint8_t wait_len;
uint16_t buzz_pattern;
uint8_t pot_value;
uint8_t count_mode;

int main(void)
{
    //disable interrupts before setup
    cli();
    
    //--- begin setup ---

    if(MCUSR & (1<<WDRF)){            // If a reset was caused by the Watchdog Timer...
        MCUSR &= ~(1<<WDRF);                // Clear the WDT reset flag
        WDTCR |= (1<<WDCE) | (1<<WDE);     // Enable the WD Change Bit
        WDTCR = 0x00;                      // Disable the WDT
    }
    
    //setup the WDT properly now
    WDTCR |= (1<<WDCE) | (1<<WDE);     // Enable the WDT Change Bit
    WDTCR = (1<<WDTIE) |                //Enable WDT Interrupt
            //(1<<WDP1) | (1<<WDP2);     // Set Timeout to ~1s
            (1<<WDP1);
            //0; //speed up 64x for debug
    //setup pins
    DDRB |= (1<<POT_ENABLE_PIN) | (1<<BUZZ_PIN) | (1<<SHIFT_PIN);  
    DDRB &= ~(1<<POT_PIN) & ~(1<<BTN_PIN);

    //setup the ADC
    ADMUX &= ~(1 << REFS0);                 //use VCC as ref voltage
    ADMUX |= (1<<ADLAR);                    //left-adjust result (for 8 MSBs in ADCH); 
    ADMUX |= POT_PIN;                     //select pin ADC2 (PB4)
    ADCSRA |= (1 << ADPS1) | (1 << ADPS0);  //set prescaler to 1/8 (125 KHz)
    ADCSRA |= (1 << ADIE);                  //enable interrupt
    ADCSRA |= (1 << ADEN);                  //enable the ADC

    //setup sleep
    //we want the deepest sleep 
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    //init globals
    delay_ctr = 0;
    buzz_pattern = 0;

    //--- setup complete; re-enable interrupts ---
    sei();

    //start main loop
    while(1) {
        if ( !(ADCSRA & (1<<ADSC))  //ADC not currently converting
            && MCUCR & (1<<SE))     //and sleep-enable bit is set
        {
            cli(); 
            sleep_bod_disable();   // Disable BOD
            sei(); 
            sleep_cpu();           // Go to Sleep
        }
    } 
}

#define BPS_0 3
#define BP_0 0b1
#define BPS_1 4
#define BP_1 0b11
inline uint16_t gen_patt(uint8_t number){
    uint16_t bpat=0;
    for(uint8_t pmsk=1<<4; pmsk;pmsk = pmsk<<1){
        if(number&pmsk){
            bpat = bpat<<BPS_1;
            bpat |= BP_1;
        } else {
            bpat = bpat<<BPS_0;
            bpat |= BP_0;
        }
    }   
    return bpat;
}


ISR(WDT_vect){
    sleep_disable(); 

    if (! (prescale_ctr%(1<<POT_FREQ)) ){
        //get a new potvalue
        PORTB |= (1<<POT_ENABLE_PIN);   //put some current into the pot 
                                        //(it's a PNP transistor, so pull it down)
        ADCSRA |= (1<<ADEN) | (1<<ADSC);            //start ADC conversion
    }
    
    //increment after that check to escape deadlock @ pot_value = 0 (which will never update)
    ++prescale_ctr;
    if(! (prescale_ctr&prescale_mask)) {
        ++delay_ctr;
        prescale_ctr = 0;
    }
    if(delay_ctr >= pot_value) {
        delay_ctr=0;
        //buzz it!
        //PORTB |= (1<<BUZZ_PIN);
        buzz_pattern = gen_patt(pot_value);
    }

    if(buzz_pattern&1) {
        PORTB |= (1<<BUZZ_PIN);  //turn on
    } else {
        PORTB &= ~(1<<BUZZ_PIN);  //turn on
    }
    buzz_pattern = buzz_pattern>>1;

    sleep_enable();
}


ISR(ADC_vect){                      //conversion complete
    PORTB &= ~(1<<POT_ENABLE_PIN);   //turn the pot off
    pot_value = ADCH;               //get the value
    ADCSRA &= ~(1<<ADEN);           //kill the ADC
    //wait_len = pot_value;
    shift_out_1w(gen_patt(pot_value), &PORTB, 1<<SHIFT_PIN, 10);
}
