#pragma once

#include "mos6522_interface.h"

class SystemClock;

class Mos6522 {
    Mos6522Interface &interface_;
    SystemClock &systemClock_;

    uint8_t orb = 0, irb = 0, ddrb = 0;
    uint8_t ora = 0, ira = 0, ddra = 0;
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
};
