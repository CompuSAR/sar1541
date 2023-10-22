#pragma once

#include "mos6522_interface.h"

class SystemClock;

class Mos6522 {
    Mos6522Interface &interface_;
    SystemClock &systemClock_;

    // Registers
    uint8_t orb = 0, irb = 0, ddrb = 0;
    uint8_t ora = 0, ira = 0, ddra = 0;

    uint16_t timer1 = 0;
    uint8_t timer1_latch_low = 0, timer1_latch_high = 0;
    bool timer1_int_armed = false;

    uint8_t aux_ctrl_reg = 0;           // ACR

    enum class Interrupts { T1 };
    uint8_t intr = 0;
public:
    enum class Port { A=0, B=1 };

    Mos6522(SystemClock &systemClock, Mos6522Interface &pinOut) :
        interface_(pinOut),
        systemClock_(systemClock)
    {}

    void main();

    // CPU interface
    uint8_t read(Addr registerSelect);
    void write(Addr registerSelect, uint8_t data);

    // Peripheral interface
    void set_input_bit( Port port, int bit, bool value );

private:
    void update_output_a();
    void update_output_b();

    void reset_intr( Interrupts intr );
    void set_intr( Interrupts intr );
};
