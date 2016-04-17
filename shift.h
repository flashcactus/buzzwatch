
inline void shift_out(uint8_t b, volatile uint8_t *port, uint8_t data_mask, uint8_t clk_mask) { //LSB first
    uint8_t bb = b;
    for(uint8_t i = 8; i; --i) {
        (*port) &= ~clk_mask;     //pull clock down first
        if(bb&1){               //write bit
            (*port) |= data_mask;
        } else {
            (*port) &= ~data_mask;
        }
        (*port) |= clk_mask;      //clock
        bb = bb>>1;
    }
}

inline void shift_out_16(uint16_t b, volatile uint8_t *port, uint8_t data_mask, uint8_t clk_mask) {
    shift_out(b, port, data_mask, clk_mask);
    shift_out(b>>8, port, data_mask, clk_mask);
}

inline void shift_out_1w(uint8_t b, volatile uint8_t *port, uint8_t out_mask, uint8_t delay) {
    uint8_t bb = b;
    for(uint8_t i = 8; i; --i) {
        if(bb&1){                   //write bit
            (*port) |= out_mask;
        } else {
            (*port) &= ~out_mask;
        }
        bb = bb>>1;                 //shift first (to conserve time after clock)
        for(uint8_t j = 0; j<delay; ++j) {//wait out the stab delay
            (*port) |= 0;           //do nothing (anti-optimization)
        }
        (*port) &= ~out_mask;       //clock (down&up)
        (*port) |= out_mask;
    }
}

inline void shift_out_1w_16(uint16_t b, volatile uint8_t *port, uint8_t out_mask, uint8_t delay) {
    shift_out_1w(b, port, out_mask, delay);
    shift_out_1w(b>>8, port, out_mask, delay);
}
