#pragma once

#include "Bus.h"

#include <stdint.h>

class c6502 {
    Bus &bus_;
    uint8_t regA, regX, regY;
    uint8_t regSp;
    uint8_t regStatus;
    uint8_t regPcL, regPcH;

    uint8_t current_opcode;

    enum class CC {
        Carry,
        Zero,
        IntMask,
        Decimal,
        NA,
        NA2,
        oVerflow,
        Negative
    };

public:
    explicit c6502(Bus &bus) : bus_(bus) {}

    void runCpu();

private:
    void handleInstruction();
    void advance_pc();

    Addr pc() const;

    uint8_t ccGet( CC cc ) const;
    void ccSet( CC cc, bool value );


    // Address modes
    Addr addrmode_abs();
    Addr addrmode_immediate();
    Addr addrmode_stack();
    Addr addrmode_zp();


    // Operations
    void op_lda(Addr addr);
    void op_pha(Addr addr);
    void op_pla(Addr addr);
    void op_rol(Addr addr);
    void op_sta(Addr addr);
};
