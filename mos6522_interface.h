#pragma once

#include "Bus.h"

class Mos6522Interface {
public:
    virtual ~Mos6522Interface() = default;

    virtual void out_pins_a( uint8_t data ) = 0;
    virtual void out_pins_b( uint8_t data ) = 0;
};
