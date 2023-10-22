#include "mos6522.h"

void Mos6522::main() {
    while(true) {
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
