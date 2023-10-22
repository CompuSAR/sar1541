#include "mos6522.h"

void Mos6522::main() {
    while(true) {
        timer1--;

        if( timer1==0 ) {
            if( aux_ctrl_reg & (1<<7) ) {
                if( timer1_int_armed )
                    orb ^= 0x80;        // XXX check whether affects ORB on real chip
            }

            if( aux_ctrl_reg & (1<<6) ) {
                set_intr( Interrupts::T1 );
                timer1 = timer1_latch_high << 8 | timer1_latch_low;
            } else {
                if( timer1_int_armed ) {
                    set_intr( Interrupts::T1 );
                    timer1_int_armed = false;
                }
            }
        }
        //systemClock.tick();
    }
}

uint8_t Mos6522::read(Addr registerSelect) {
    switch( registerSelect & 0x0f ) {
    case 0x00:
        return irb;
    case 0x01:
        return ira;
    case 0x02:
        return ddrb;
    case 0x03:
        return ddra;
    case 0x04:
        reset_intr( Interrupts::T1 );
        return timer1 & 0xff;
    case 0x05:
        return timer1 >> 8;
    case 0x06:
        return timer1_latch_low;
    case 0x07:
        return timer1_latch_high;
    case 0x0a:
        return aux_ctrl_reg;
    }
}

void Mos6522::write(Addr registerSelect, uint8_t data) {
    switch( registerSelect & 0x0f ) {
    case 0x00:
        orb = data;
        update_output_b();
        break;
    case 0x01:
        ora = data;
        update_output_a();
        break;
    case 0x02:
        ddrb = data;
        update_output_b();
        break;
    case 0x03:
        ddra = data;
        update_output_a();
        break;
    case 0x04:
    case 0x06:
        timer1_latch_low = data;
        break;
    case 0x05:
        timer1_latch_high = data;
        timer1 = timer1_latch_high << 8 | timer1_latch_low;
        reset_intr( Interrupts::T1 );
        break;
    case 0x07:
        timer1_latch_high = data;
        reset_intr( Interrupts::T1 );
        break;
    case 0x0a:
        aux_ctrl_reg = data;
        break;
    }
}

void Mos6522::set_input_bit( Port port, int bit, bool value ) {
    uint8_t &in = port==Port::A ? ira : irb;

    if( value ) {
        in |= 1<<bit;
    } else {
        in &= ~( 1<<bit );
    }
}

// Private methods
void Mos6522::update_output_a() {
    interface_.out_pins_a( ora | ddra );
}

void Mos6522::update_output_b() {
    interface_.out_pins_b( orb | ddrb );
}
