#pragma once

#include "Bus.h"

#include <stdint.h>

class c6502 {
    class CpuReset {};

    Bus &bus_;
    uint8_t regA, regX, regY;
    uint8_t regSp;
    uint8_t regStatus = 0xff;
    uint8_t regPcL, regPcH;

    uint8_t current_opcode;

    bool reset = false, irq = false, nmi = false, ready = false, so = false;

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
    void setReset(bool state);
    void setIrq(bool state);
    void setNmi(bool state);
    void setReady(bool state);
    void setSo(bool state);

private:
    void handleInstruction();
    void resetSequence();
    void advance_pc();

    Addr pc() const;

    uint8_t read( Addr address, bool sync = false );
    void write( Addr address, uint8_t data );

    uint8_t ccGet( CC cc ) const;
    void ccSet( CC cc, bool value );


    void branch_helper(Addr addr, bool jump);

    // Address modes
    Addr addrmode_abs();
    Addr addrmode_immediate();
    Addr addrmode_implicit();
    Addr addrmode_stack();
    Addr addrmode_zp();
    Addr addrmode_special();


    // Operations
    void op_bcc(Addr addr);
    void op_bcs(Addr addr);
    void op_beq(Addr addr);
    void op_bmi(Addr addr);
    void op_bne(Addr addr);
    void op_bpl(Addr addr);
    void op_bvc(Addr addr);
    void op_bvs(Addr addr);
    void op_jmp(Addr addr);
    void op_jsr(Addr addr);
    void op_lda(Addr addr);
    void op_ldx(Addr addr);
    void op_nop(Addr addr);
    void op_pha(Addr addr);
    void op_php(Addr addr);
    void op_pla(Addr addr);
    void op_plp(Addr addr);
    void op_rol(Addr addr);
    void op_rti(Addr addr);
    void op_rts(Addr addr);
    void op_sta(Addr addr);
    void op_txs(Addr addr);
};
