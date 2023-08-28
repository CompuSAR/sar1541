#include "c6502.h"

#include <iostream>

static Addr compose(uint8_t high, uint8_t low) {
    return high<<8 | low;
}

void c6502::runCpu() {
    while(true) {
        try {
            if( reset )
                resetSequence();
            else
                handleInstruction();
        } catch( CpuReset ex ) {
        }
    }
}

void c6502::setReset(bool state) {
    reset = state;
    std::cout<<"CPU reset "<<state<<"\n";
}
void c6502::setIrq(bool state) {
    irq = state;
    std::cout<<"CPU IRQ "<<state<<"\n";
}
void c6502::setNmi(bool state) {
    nmi = state;
    std::cout<<"CPU NMI "<<state<<"\n";
}
void c6502::setReady(bool state) {
    ready = state;
    std::cout<<"CPU ready "<<state<<"\n";
}
void c6502::setSo(bool state) {
    so = state;
    std::cout<<"CPU SO "<<state<<"\n";
}


void c6502::handleInstruction() {
    current_opcode = read( pc(), true );
    advance_pc();

    switch(current_opcode) {
    case 0x26: op_rol( addrmode_zp() );                         break;
    case 0x2e: op_rol( addrmode_abs() );                        break;
    case 0x48: op_pha( addrmode_stack() );                      break;
    case 0x68: op_pla( addrmode_stack() );                      break;
    case 0x85: op_sta( addrmode_zp() );                         break;
    case 0x9a: op_txs( addrmode_implicit() );                   break;
    case 0xa2: op_ldx( addrmode_immediate() );                  break;
    case 0xa5: op_lda( addrmode_zp() );                         break;
    case 0xa9: op_lda( addrmode_immediate() );                  break;
    case 0xad: op_lda( addrmode_abs() );                        break;
    default: std::cerr<<"Unknown command "<<std::hex<<int(current_opcode)<<" at "<<(pc()-1)<<"\n"; abort();
    }
}

void c6502::resetSequence() {
    regPcL = read(0xfffc);
    regPcH = read(0xfffd);
}

void c6502::advance_pc() {
    Addr pc_ = pc();
    pc_++;
    regPcH = pc_>>8;
    regPcL = pc_ & 0xff;
}

Addr c6502::pc() const {
    return compose(regPcH, regPcL);
}

uint8_t c6502::read( Addr address, bool sync ) {
    uint8_t result = bus_.read( this, address, sync );

    if( reset ) {
        throw CpuReset();
    }

    return result;
}

void c6502::write( Addr address, uint8_t data ) {
    if( reset ) {
        bus_.read( this, address );

        throw CpuReset();
    }

    bus_.write( this, address, data );
}


uint8_t c6502::ccGet( CC cc ) const {
    return bool( regStatus & (1<<int(cc)) );
}

void c6502::ccSet( CC cc, bool value ) {
    if( value ) {
        regStatus |= 1<<int(cc);
    } else {
        regStatus &= ~( 1<<int(cc) );
    }
}

// Address modes
Addr c6502::addrmode_abs() {
    Addr res = read( pc() );
    advance_pc();
    res |= read( pc() ) << 8;
    advance_pc();

    return res;
}

Addr c6502::addrmode_immediate() {
    Addr stored_pc = pc();
    advance_pc();

    return stored_pc;
}

Addr c6502::addrmode_implicit() {
    return pc();
}

Addr c6502::addrmode_stack() {
    read( pc() );

    return compose( 0x01, regSp );
}

Addr c6502::addrmode_zp() {
    uint8_t addr = read( pc() );
    advance_pc();

    return addr;
}

void c6502::op_lda(Addr addr) {
    regA = read( addr );
}

void c6502::op_ldx(Addr addr) {
    regX = read( addr );
}

void c6502::op_pha(Addr addr) {
    write( addr, regA );

    regSp--;
}

void c6502::op_pla(Addr addr) {
    read( addr );

    regSp++;
    regA = read( compose( 0x01, regSp ) );

    ccSet( CC::Negative, regA & 0x80 );
    ccSet( CC::Zero, regA==0 );
}

void c6502::op_rol(Addr addr) {
    uint16_t value = read( addr );
    write( addr, value );

    value <<= 1;
    value |= ccGet( CC::Carry );
    ccSet( CC::Carry, value & 0x100 );
    ccSet( CC::Negative, value & 0x80 );
    ccSet( CC::Zero, (value&0xff)==0 );

    write( addr, value );
}

void c6502::op_sta(Addr addr) {
    write( addr, regA );
}

void c6502::op_txs(Addr addr) {
    regSp = regX;
}
