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
    case 0x08: op_php( addrmode_stack() );                      break;
    case 0x10: op_bpl( addrmode_immediate() );                  break;
    case 0x20: op_jsr( addrmode_special() );                    break;
    case 0x26: op_rol( addrmode_zp() );                         break;
    case 0x28: op_plp( addrmode_stack() );                      break;
    case 0x2e: op_rol( addrmode_abs() );                        break;
    case 0x30: op_bmi( addrmode_immediate() );                  break;
    case 0x40: op_rti( addrmode_stack() );                      break;
    case 0x48: op_pha( addrmode_stack() );                      break;
    case 0x4c: op_jmp( addrmode_abs() );                        break;
    case 0x50: op_bvc( addrmode_immediate() );                  break;
    case 0x60: op_rts( addrmode_stack() );                      break;
    case 0x68: op_pla( addrmode_stack() );                      break;
    case 0x70: op_bvs( addrmode_immediate() );                  break;
    case 0x85: op_sta( addrmode_zp() );                         break;
    case 0x90: op_bcc( addrmode_immediate() );                  break;
    case 0x9a: op_txs( addrmode_implicit() );                   break;
    case 0xa2: op_ldx( addrmode_immediate() );                  break;
    case 0xa5: op_lda( addrmode_zp() );                         break;
    case 0xa9: op_lda( addrmode_immediate() );                  break;
    case 0xad: op_lda( addrmode_abs() );                        break;
    case 0xb0: op_bcs( addrmode_immediate() );                  break;
    case 0xd0: op_bne( addrmode_immediate() );                  break;
    case 0xea: op_nop( addrmode_implicit() );                   break;
    case 0xf0: op_beq( addrmode_immediate() );                  break;
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
    read( pc() );

    return pc();
}

Addr c6502::addrmode_special() {
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

void c6502::branch_helper(Addr addr, bool jump) {
    int8_t offset = read(addr);

    if( jump ) {
        read( pc() );

        uint16_t program_counter = pc();
        program_counter += offset;

        regPcL = program_counter & 0xff;

        if( regPcH != (program_counter>>8) ) {
            read( pc() );
            regPcH = program_counter >> 8;
        }
    }
}

void c6502::op_bcc(Addr addr) {
    branch_helper(addr, ! ccGet(CC::Carry));
}

void c6502::op_bcs(Addr addr) {
    branch_helper(addr, ccGet(CC::Carry));
}

void c6502::op_beq(Addr addr) {
    branch_helper(addr, ccGet(CC::Zero));
}

void c6502::op_bne(Addr addr) {
    branch_helper(addr, ! ccGet(CC::Zero));
}

void c6502::op_bmi(Addr addr) {
    branch_helper(addr, ccGet(CC::Negative));
}

void c6502::op_bpl(Addr addr) {
    branch_helper(addr, ! ccGet(CC::Negative));
}


void c6502::op_bvc(Addr addr) {
    branch_helper(addr, ! ccGet(CC::oVerflow));
}

void c6502::op_bvs(Addr addr) {
    branch_helper(addr, ccGet(CC::oVerflow));
}

void c6502::op_jmp(Addr addr) {
    regPcL = addr & 0xff;
    regPcH = addr >> 8;
}

void c6502::op_jsr(Addr addr) {
    uint8_t dest_low = read( pc() );
    advance_pc();

    read( compose( 0x01, regSp ) );
    write( compose( 0x01, regSp-- ), regPcH );
    write( compose( 0x01, regSp-- ), regPcL );

    regPcH = read( pc() );
    regPcL = dest_low;
}

void c6502::op_lda(Addr addr) {
    regA = read( addr );
}

void c6502::op_ldx(Addr addr) {
    regX = read( addr );
}

void c6502::op_nop(Addr addr) {
}

void c6502::op_pha(Addr addr) {
    write( addr, regA );

    regSp--;
}

void c6502::op_php(Addr addr) {
    write( addr, regStatus );

    regSp--;
}

void c6502::op_pla(Addr addr) {
    read( addr );

    regSp++;
    regA = read( compose( 0x01, regSp ) );

    ccSet( CC::Negative, regA & 0x80 );
    ccSet( CC::Zero, regA==0 );
}

void c6502::op_plp(Addr addr) {
    read( addr );

    regSp++;
    regStatus = read( compose( 0x01, regSp ) ) | 0x30;
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

void c6502::op_rti(Addr addr) {
    read( addr );
    regSp++;

    regStatus = read( compose(0x01, regSp++) ) | 0x30;

    regPcL = read( compose(0x01, regSp++) );
    regPcH = read( compose(0x01, regSp) );
}

void c6502::op_rts(Addr addr) {
    read( compose(0x01, regSp++) );

    regPcL = read( compose(0x01, regSp++) );
    regPcH = read( compose(0x01, regSp) );

    read( pc() );
    advance_pc();
}

void c6502::op_sta(Addr addr) {
    write( addr, regA );
}

void c6502::op_txs(Addr addr) {
    regSp = regX;
}
