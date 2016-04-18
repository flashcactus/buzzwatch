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

//times for clicks (in ovf_ticks)
#define CLICK_S 1      //65ms must be enough
#define CLICK_M 15     //just under 1s
#define CLICK_L 46     //just over 3s        
#define CLICK_XXL 152   //10s
#define MINRELEASE 2   //~130 ms
#define MAXRELEASE 16  //~1s

//globals
uint8_t prescale_mask=0;//1<<exp
uint8_t prescale_ctr;   //when this reaches 2^exp, delay_ctr is incremented
uint8_t delay_ctr;      //this gets compared with the ADC value
uint8_t ovf_ticks;      //counts timer overflows [see timsetup()]

uint8_t pot_value;      //whatever we got from the ADC
uint8_t btn_states;     //previous states of the button (will probably be 10101010... but whatever)

uint8_t shortclick_cnt; //interestingly enough, counts small clicks.

uint16_t buzz_pattern;  //buzz a rhythm!


inline void timzero(){
    //zero the timer&overflow.
    ovf_ticks=0;            
    TCNT0=0;
}

inline void timsetup() {
    TCCR0B |= (1<<CS02);    //set prescaler to 1/256. 
                            //@1MHz:
                            //  tick=.25 ms; 
                            //  overflow=65.5ms
                            //  1s=15.25ovf;  
                            //  ovf_ticks overflow=16.77s
    
    TIMSK0 |= (1<<TOIE0);        //enable the interrupt
    timzero();
}

inline void adcsetup() {
    //setup the ADC
    ADMUX &= ~(1 << REFS0);                 //use VCC as ref voltage
    ADMUX |= (1<<ADLAR);                    //left-adjust result (for 8 MSBs in ADCH); 
    ADMUX |= POT_PIN;                       //select pin ADC2 (PB4)
    ADCSRA |= (1 << ADPS1) | (1 << ADPS0);  //set prescaler to 1/8 (125 KHz)
    ADCSRA |= (1 << ADIE);                  //enable interrupt
    ADCSRA |= (1 << ADEN);                  //enable the ADC
}

int main(void)
{
    //disable interrupts before setup
    cli();
    
    //############## begin setup ##############

    if(MCUSR & (1<<WDRF)){              // If a reset was caused by the Watchdog Timer..
                                        // (highly unlikely, but just in case)
        MCUSR &= ~(1<<WDRF);            // Clear the WDT reset flag
        WDTCR |= (1<<WDCE) | (1<<WDE);  // Enable the WD Change Bit
        WDTCR = 0x00;                   // Disable the WDT
    }
    
    //setup the WDT properly now
    WDTCR |= (1<<WDCE) | (1<<WDE);      // Enable the WDT Change Bit
    WDTCR = (1<<WDTIE) |                //Enable WDT Interrupt
            //(1<<WDP1) | (1<<WDP2);    // Set Timeout to ~1s
            (1<<WDP1)|(1<<WDP0);        //.125s (8x, for debug)
            //0; //speed up 64x for debug
    
    //set i/o pin modes
    DDRB |= (1<<POT_ENABLE_PIN) | (1<<BUZZ_PIN) | (1<<SHIFT_PIN);   //out
    DDRB &= ~(1<<POT_PIN) & ~(1<<BTN_PIN);                          //in
    PORTB |= (1<<BTN_PIN);  //enable the internal pullup for the button

    //setup the timer
    timsetup();

    //setup the ADC
    adcsetup();

    //enable the INT0 interrupt for button capture
    EIMSK |= (1<<INT0);                 //enable
    MCUCR &= ~((1<<ISC01)|(1<<ISC00));  //set INT0 mode to level-triggered

    //setup sleep
    //we want the deepest (powerdown) sleep mode 
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);

    //init globals
    delay_ctr = 0;
    buzz_pattern = 0;
    
    //########## setup complete; re-enable interrupts ##########
    sei();
    
    //do the sleepyloop
    while(1) {
        if (                        //sleep-allowed conditions (all at once):
            !(ADCSRA & (1<<ADSC))   //  ADC not currently converting
            && MCUCR & (1<<SE)      //  sleep-enable bit is set (=WDT routine finished)
            && !buzz_pattern        //  buzzed everything already
            && PORTB & (1<<BTN_PIN) //  button is not pressed
        ){
            cli(); 
            //turn everything off, to be sure
            PORTB &= ~( (1<<BUZZ_PIN) | (1<<POT_ENABLE_PIN) ) ;
            
            // Disable BOD
            sleep_bod_disable();    
            sei(); 
            
            //set INT0 mode to level-triggered 
            //(otherwise it couldn't wake the MCU)
            MCUCR &= ~((1<<ISC01)|(1<<ISC00));  
            
            // Go to Sleep
            sleep_cpu();            
        }
    } 
}

ISR(INT0_vect){
    MCUCR |= (1<<ISC00);        //set edge-triggered mode
    btn_states <<= 1;
    if(! PORTB & (1<<BTN_PIN)) {//button is pressed
        btn_states |= 1;
    }
    /*switch(btn_states&0b11){
        case 0b01:  //release
        case 0b10:  //press
    }*/
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

    shift_out_1w(ovf_ticks, &PORTB, 1<<SHIFT_PIN, 10); //debug
    shift_out_1w(delay_ctr, &PORTB, 1<<SHIFT_PIN, 10);

    if(buzz_pattern&1) {            //bzzzzz!
        PORTB |= (1<<BUZZ_PIN);
    } else {
        PORTB &= ~(1<<BUZZ_PIN);
    }
    buzz_pattern = buzz_pattern>>1;

    //do some button-processing
    if (PORTB & (1<<BTN_PIN)){          //button is released
        if(ovf_ticks >= MAXRELEASE) {   //and has been for some time now
            //TODO: finalize buttonstats
        }
    } else {                            //button is pressed
        //do a small buzz at each of the thresholds
        if (ovf_ticks == CLICK_M || ovf_ticks == CLICK_L) { 
            PORTB |= (1<<BUZZ_PIN);
        }
        //long long press = reset.
        else if (ovf_ticks >= CLICK_XXL) {  
            //TODO: soft-reset the MCU
        }
    }
}

ISR(ADC_vect){                      //conversion complete
    PORTB &= ~(1<<POT_ENABLE_PIN);  //turn the pot off
    pot_value = ADCH;               //get the value
    ADCSRA &= ~(1<<ADEN);           //kill the ADC
}
