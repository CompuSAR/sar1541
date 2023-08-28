#pragma once

#include <stdint.h>

using Addr = uint16_t;

class c6502;

class Bus {
public:
    virtual uint8_t read( c6502 *cpu, Addr address, bool sync = false ) = 0;
    virtual void write( c6502 *cpu, Addr address, uint8_t ) = 0;
};
