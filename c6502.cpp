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
    case 0x00: op_brk( addrmode_immediate() );                  break;
    case 0x01: op_ora( addrmode_zp_x_ind() );                   break;
    case 0x05: op_ora( addrmode_zp() );                         break;
    case 0x06: op_asl( addrmode_zp() );                         break;
    case 0x08: op_php( addrmode_stack() );                      break;
    case 0x09: op_ora( addrmode_immediate() );                  break;
    case 0x0a: op_aslA();                                       break;
    case 0x0d: op_ora( addrmode_abs() );                        break;
    case 0x0e: op_asl( addrmode_abs() );                        break;
    case 0x10: op_bpl( addrmode_immediate() );                  break;
    case 0x11: op_ora( addrmode_zp_ind_y() );                   break;
    case 0x15: op_ora( addrmode_zp_x() );                       break;
    case 0x16: op_asl( addrmode_zp_x() );                       break;
    case 0x18: op_clc( addrmode_implicit() );                   break;
    case 0x19: op_ora( addrmode_abs_y() );                      break;
    case 0x1a: op_incA( addrmode_implicit() );                  break;
    case 0x1d: op_ora( addrmode_abs_x() );                      break;
    case 0x1e: op_asl( addrmode_abs_x(true) );                  break;
    case 0x20: op_jsr( addrmode_special() );                    break;
    case 0x21: op_and( addrmode_zp_x_ind() );                   break;
    case 0x24: op_bit( addrmode_zp() );                         break;
    case 0x25: op_and( addrmode_zp() );                         break;
    case 0x26: op_rol( addrmode_zp() );                         break;
    case 0x28: op_plp( addrmode_stack() );                      break;
    case 0x29: op_and( addrmode_immediate() );                  break;
    case 0x2a: op_rolA();                                       break;
    case 0x2c: op_bit( addrmode_abs() );                        break;
    case 0x2d: op_and( addrmode_abs() );                        break;
    case 0x2e: op_rol( addrmode_abs() );                        break;
    case 0x30: op_bmi( addrmode_immediate() );                  break;
    case 0x31: op_and( addrmode_zp_ind_y() );                   break;
    case 0x34: op_bit( addrmode_zp_x() );                       break;
    case 0x35: op_and( addrmode_zp_x() );                       break;
    case 0x36: op_rol( addrmode_zp_x() );                       break;
    case 0x38: op_sec( addrmode_implicit() );                   break;
    case 0x39: op_and( addrmode_abs_y() );                      break;
    case 0x3a: op_decA( addrmode_implicit() );                  break;
    case 0x3c: op_bit( addrmode_abs_x() );                      break;
    case 0x3d: op_and( addrmode_abs_x() );                      break;
    case 0x3e: op_rol( addrmode_abs_x(true) );                  break;
    case 0x40: op_rti( addrmode_stack() );                      break;
    case 0x41: op_eor( addrmode_zp_x_ind() );                   break;
    case 0x45: op_eor( addrmode_zp() );                         break;
    case 0x46: op_lsr( addrmode_zp() );                         break;
    case 0x48: op_pha( addrmode_stack() );                      break;
    case 0x49: op_eor( addrmode_immediate() );                  break;
    case 0x4a: op_lsrA();                                       break;
    case 0x4c: op_jmp( addrmode_abs() );                        break;
    case 0x4d: op_eor( addrmode_abs() );                        break;
    case 0x4e: op_lsr( addrmode_abs() );                        break;
    case 0x50: op_bvc( addrmode_immediate() );                  break;
    case 0x51: op_eor( addrmode_zp_ind_y() );                   break;
    case 0x52: op_eor( addrmode_zp_ind() );                     break;
    case 0x55: op_eor( addrmode_zp_x() );                       break;
    case 0x56: op_lsr( addrmode_zp_x() );                       break;
    case 0x58: op_cli( addrmode_implicit() );                   break;
    case 0x59: op_eor( addrmode_abs_y() );                      break;
    case 0x5d: op_eor( addrmode_abs_x() );                      break;
    case 0x5e: op_lsr( addrmode_abs_x() );                      break;
    case 0x60: op_rts( addrmode_stack() );                      break;
    case 0x61: op_adc( addrmode_zp_x_ind() );                   break;
    case 0x65: op_adc( addrmode_zp() );                         break;
    case 0x66: op_ror( addrmode_zp() );                         break;
    case 0x68: op_pla( addrmode_stack() );                      break;
    case 0x69: op_adc( addrmode_immediate() );                  break;
    case 0x6a: op_rorA();                                       break;
    case 0x6c: op_jmp( addrmode_abs_ind() );                    break;
    case 0x6d: op_adc( addrmode_abs() );                        break;
    case 0x6e: op_ror( addrmode_abs() );                        break;
    case 0x70: op_bvs( addrmode_immediate() );                  break;
    case 0x71: op_adc( addrmode_zp_ind_y() );                   break;
    case 0x75: op_adc( addrmode_zp_x() );                       break;
    case 0x76: op_ror( addrmode_zp_x() );                       break;
    case 0x78: op_sei( addrmode_implicit() );                   break;
    case 0x79: op_adc( addrmode_abs_y() );                      break;
    case 0x7d: op_adc( addrmode_abs_x() );                      break;
    case 0x7e: op_ror( addrmode_abs_x(true) );                  break;
    case 0x81: op_sta( addrmode_zp_x_ind() );                   break;
    case 0x85: op_sta( addrmode_zp() );                         break;
    case 0x86: op_stx( addrmode_zp() );                         break;
    case 0x88: op_dey( addrmode_implicit() );                   break;
    case 0x89: op_bit( addrmode_immediate() );                  break;
    case 0x8a: op_txa( addrmode_implicit() );                   break;
    case 0x8c: op_sty( addrmode_abs() );                        break;
    case 0x8d: op_sta( addrmode_abs() );                        break;
    case 0x8e: op_stx( addrmode_abs() );                        break;
    case 0x90: op_bcc( addrmode_immediate() );                  break;
    case 0x91: op_sta( addrmode_zp_ind_y(true) );               break;
    case 0x92: op_sta( addrmode_zp_ind() );                     break;
    case 0x95: op_sta( addrmode_zp_x() );                       break;
    case 0x96: op_stx( addrmode_zp_y() );                       break;
    case 0x98: op_tya( addrmode_implicit() );                   break;
    case 0x99: op_sta( addrmode_abs_y(true) );                  break;
    case 0x9a: op_txs( addrmode_implicit() );                   break;
    case 0x9d: op_sta( addrmode_abs_x(true) );                  break;
    case 0xa0: op_ldy( addrmode_immediate() );                  break;
    case 0xa2: op_ldx( addrmode_immediate() );                  break;
    case 0xa4: op_ldy( addrmode_zp() );                         break;
    case 0xa5: op_lda( addrmode_zp() );                         break;
    case 0xa6: op_ldx( addrmode_zp() );                         break;
    case 0xa8: op_tay();                                        break;
    case 0xa9: op_lda( addrmode_immediate() );                  break;
    case 0xaa: op_tax();                                        break;
    case 0xac: op_ldy( addrmode_abs() );                        break;
    case 0xad: op_lda( addrmode_abs() );                        break;
    case 0xae: op_ldx( addrmode_abs() );                        break;
    case 0xb0: op_bcs( addrmode_immediate() );                  break;
    case 0xb1: op_lda( addrmode_zp_ind_y() );                   break;
    case 0xb4: op_ldy( addrmode_zp_x() );                       break;
    case 0xb5: op_lda( addrmode_zp_x() );                       break;
    case 0xb6: op_ldx( addrmode_zp_y() );                       break;
    case 0xb8: op_clv( addrmode_implicit() );                   break;
    case 0xb9: op_lda( addrmode_abs_y() );                      break;
    case 0xbc: op_ldy( addrmode_abs_x() );                      break;
    case 0xbd: op_lda( addrmode_abs_x() );                      break;
    case 0xbe: op_ldx( addrmode_abs_y() );                      break;
    case 0xc0: op_cpy( addrmode_immediate() );                  break;
    case 0xc1: op_cmp( addrmode_zp_x_ind() );                   break;
    case 0xc4: op_cpy( addrmode_zp() );                         break;
    case 0xc5: op_cmp( addrmode_zp() );                         break;
    case 0xc6: op_dec( addrmode_zp() );                         break;
    case 0xc8: op_iny( addrmode_implicit() );                   break;
    case 0xc9: op_cmp( addrmode_immediate() );                  break;
    case 0xca: op_dex( addrmode_implicit() );                   break;
    case 0xcc: op_cpy( addrmode_abs() );                        break;
    case 0xcd: op_cmp( addrmode_abs() );                        break;
    case 0xce: op_dec( addrmode_abs() );                        break;
    case 0xd0: op_bne( addrmode_immediate() );                  break;
    case 0xd1: op_cmp( addrmode_zp_ind_y() );                   break;
    case 0xd2: op_cmp( addrmode_zp_ind() );                     break;
    case 0xd5: op_cmp( addrmode_zp_x() );                       break;
    case 0xd6: op_dec( addrmode_zp_x() );                       break;
    case 0xd8: op_cld( addrmode_implicit() );                   break;
    case 0xd9: op_cmp( addrmode_abs_y() );                      break;
    case 0xdd: op_cmp( addrmode_abs_x() );                      break;
    case 0xde: op_dec( addrmode_abs_x() );                      break;
    case 0xe0: op_cpx( addrmode_immediate() );                  break;
    case 0xe1: op_sbc( addrmode_zp_x_ind() );                   break;
    case 0xe4: op_cpx( addrmode_zp() );                         break;
    case 0xe5: op_sbc( addrmode_zp() );                         break;
    case 0xe6: op_inc( addrmode_zp() );                         break;
    case 0xe8: op_inx( addrmode_implicit() );                   break;
    case 0xe9: op_sbc( addrmode_immediate() );                  break;
    case 0xea: op_nop( addrmode_implicit() );                   break;
    case 0xec: op_cpx( addrmode_abs() );                        break;
    case 0xed: op_sbc( addrmode_abs() );                        break;
    case 0xee: op_inc( addrmode_abs() );                        break;
    case 0xf0: op_beq( addrmode_immediate() );                  break;
    case 0xf1: op_sbc( addrmode_zp_ind_y() );                   break;
    case 0xf5: op_sbc( addrmode_zp_x() );                       break;
    case 0xf6: op_inc( addrmode_zp_x() );                       break;
    case 0xf8: op_sed( addrmode_implicit() );                   break;
    case 0xf9: op_sbc( addrmode_abs_y() );                      break;
    case 0xfd: op_sbc( addrmode_abs_x() );                      break;
    case 0xfe: op_inc( addrmode_abs_x() );                      break;
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

Addr c6502::addrmode_abs_ind() {
    uint8_t addrL = read( pc() );
    advance_pc();
    uint8_t addrH = read( pc() );
    advance_pc();

    Addr res = read( compose( addrH, addrL++ ) );
    res |= read( compose( addrH, addrL ) )<<8;

    return res;
}

Addr c6502::addrmode_abs_x(bool always_waste_cycle) {
    Addr resL = read( pc() );
    advance_pc();
    Addr resH = read( pc() ) << 8;
    resL += regX;
    advance_pc();

    if( resL>>8 != 0 || always_waste_cycle ) {
        read( compose( resH>>8, resL&0xff ) );
    }

    return resL + resH;
}

Addr c6502::addrmode_abs_y(bool always_waste_cycle) {
    Addr resL = read( pc() );
    advance_pc();
    Addr resH = read( pc() ) << 8;
    resL += regY;
    advance_pc();

    if( always_waste_cycle || resL>>8 != 0 ) {
        read( compose( resH>>8, resL&0xff ) );
    }

    return resL + resH;
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

Addr c6502::addrmode_zp_ind() {
    uint8_t addr = read( pc() );
    advance_pc();

    Addr res_lsb = read(addr);

    Addr res_msb = read( (addr+1) & 0xff );

    return res_lsb + res_msb*256;
}

Addr c6502::addrmode_zp_ind_y(bool always_waste_cycle) {
    uint8_t addr = read( pc() );
    advance_pc();

    Addr res_lsb = read(addr);

    res_lsb += regY;
    Addr res_msb = read( (addr+1) & 0xff );

    if( always_waste_cycle || res_lsb>>8 != 0 ) {
        read( compose( res_msb, res_lsb&0xff ) );
    }

    return res_lsb + res_msb*256;
}

Addr c6502::addrmode_zp_x() {
    uint8_t addr = read( pc() );
    advance_pc();

    read(addr);

    return (addr + regX) & 0xff;
}

Addr c6502::addrmode_zp_x_ind() {
    uint8_t zp = read( pc() );
    advance_pc();

    read(zp);
    zp += regX;
    zp &= 0xff;

    Addr addr = read(zp);

    zp++;
    zp &= 0xff;

    addr |= read(zp) << 8;

    return addr;
}

Addr c6502::addrmode_zp_y() {
    uint8_t addr = read( pc() );
    advance_pc();

    read(addr);

    return (addr + regY) & 0xff;
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

void c6502::op_adc(Addr addr) {
    uint16_t val = read(addr);

    bool sameSign = (val & 0x80) == (regA & 0x80);

    val += regA + (ccGet( CC::Carry ) ? 1 : 0);

    bool stillSameSign = (val & 0x80) == (regA & 0x80);

    ccSet( CC::oVerflow, sameSign && !stillSameSign );

    regA = val;

    ccSet( CC::Carry, val & 0x100 );
    updateNZ( val );
}

void c6502::op_and(Addr addr) {
    regA &= read(addr);

    updateNZ( regA );
}

void c6502::op_asl(Addr addr) {
    uint16_t val = read(addr);
    write(addr, val);
    val <<= 1;

    ccSet( CC::Carry, val&0x100 );

    updateNZ( val );

    write(addr, val);
}

void c6502::op_aslA() {
    read( pc() );

    uint16_t val = regA;

    val<<=1;

    ccSet( CC::Carry, val&0x100 );

    updateNZ( val );

    regA = val;
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

void c6502::op_bit(Addr addr) {
    uint8_t mem = read(addr);
    uint8_t result = regA ^ mem;

    ccSet( CC::Negative, mem & 0x80 );
    ccSet( CC::oVerflow, mem & 0x40 );
    ccSet( CC::Zero, result==0 );
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


void c6502::op_brk(Addr addr) {
    read(addr);

    write( compose( 0x01, regSp--), regPcH );
    write( compose( 0x01, regSp--), regPcL );
    write( compose( 0x01, regSp--), regStatus );

    regPcL = read( 0xfffe );
    regPcH = read( 0xffff );

    ccSet( CC::IntMask, true );
}

void c6502::op_bvc(Addr addr) {
    branch_helper(addr, ! ccGet(CC::oVerflow));
}

void c6502::op_bvs(Addr addr) {
    branch_helper(addr, ccGet(CC::oVerflow));
}

void c6502::op_clc(Addr addr) {
    ccSet( CC::Carry, false );
}

void c6502::op_cld(Addr addr) {
    ccSet( CC::Decimal, false );
}

void c6502::op_cli(Addr addr) {
    ccSet( CC::IntMask, false );
}

void c6502::op_clv(Addr addr) {
    ccSet( CC::oVerflow, false );
}

void c6502::op_cmp(Addr addr) {
    uint16_t calc = 0x100 | regA;
    calc -= read(addr);

    ccSet( CC::Carry, calc & 0x100 );
    updateNZ( calc );
}

void c6502::op_cpx(Addr addr) {
    uint16_t calc = 0x100 | regX;
    calc -= read(addr);

    updateNZ( calc );
    ccSet( CC::Carry, calc & 0x100 );
}

void c6502::op_cpy(Addr addr) {
    uint16_t calc = 0x100 | regY;
    calc -= read(addr);

    updateNZ( calc );
    ccSet( CC::Carry, calc & 0x100 );
}

void c6502::op_dec(Addr addr) {
    uint8_t res = read(addr);
    write(addr, res);

    res--;
    write(addr, res);

    updateNZ( res );
}

void c6502::op_decA(Addr addr) {
    regX--;

    updateNZ( regX );
}

void c6502::op_dex(Addr addr) {
    regX--;

    updateNZ( regX );
}

void c6502::op_dey(Addr addr) {
    regY--;

    updateNZ( regY );
}

void c6502::op_eor(Addr addr) {
    regA ^= read(addr);

    updateNZ( regA );
}

void c6502::op_inc(Addr addr) {
    uint8_t val = read(addr);

    write(addr, val);
    write(addr, ++val);

    updateNZ( val );
}

void c6502::op_incA(Addr addr) {
    regA++;

    updateNZ( regA );
}

void c6502::op_inx(Addr addr) {
    regX++;

    updateNZ( regX );
}

void c6502::op_iny(Addr addr) {
    regY++;

    updateNZ( regY );
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

    updateNZ( regA );
}

void c6502::op_ldx(Addr addr) {
    regX = read( addr );

    updateNZ( regX );
}

void c6502::op_ldy(Addr addr) {
    regY = read( addr );

    updateNZ( regY );
}

void c6502::op_lsr(Addr addr) {
    uint8_t val = read(addr);
    write(addr, val);

    ccSet( CC::Carry, val&0x01 );
    val >>= 1;

    updateNZ( val );

    write(addr, val);
}

void c6502::op_lsrA() {
    read( pc() );

    ccSet( CC::Carry, regA&0x01 );

    regA >>= 1;

    updateNZ( regA );
}

void c6502::op_nop(Addr addr) {
}

void c6502::op_ora(Addr addr) {
    regA |= read(addr);

    updateNZ( regA );
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

    updateNZ( regA );
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
    updateNZ( value );

    write( addr, value );
}

void c6502::op_rolA() {
    read( pc() );

    bool newC = regA & 0x80;
    regA <<= 1;
    if( ccGet( CC::Carry ) )
        regA |= 0x01;

    ccSet( CC::Carry, newC );
    updateNZ( regA );
}

void c6502::op_ror(Addr addr) {
    uint8_t value = read( addr );
    write( addr, value );

    bool newC = value & 0x01;
    value >>= 1;
    value |= ccGet( CC::Carry ) ? 0x80 : 0x00;
    ccSet( CC::Carry, newC );
    updateNZ( value );

    write( addr, value );
}

void c6502::op_rorA() {
    read( pc() );

    bool newC = regA & 0x01;
    regA >>= 1;
    regA |= ccGet( CC::Carry ) ? 0x80 : 0x00;
    ccSet( CC::Carry, newC );
    updateNZ( regA );

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

void c6502::op_sbc(Addr addr) {
    uint8_t val = read(addr);

    uint16_t res = regA - val;
    if( !ccGet( CC::Carry ) )
        res--;

    bool sameSign = (val & 0x80) == (regA & 0x80);
    bool stillSameSign = (val & 0x80) == (res & 0x80);

    regA = res;

    ccSet( CC::oVerflow, !sameSign && stillSameSign );

    updateNZ( regA );
    ccSet( CC::Carry, (res & 0x100)==0 );
}

void c6502::op_sec(Addr addr) {
    ccSet( CC::Carry, true );
}

void c6502::op_sed(Addr addr) {
    ccSet( CC::Decimal, true );
}

void c6502::op_sei(Addr addr) {
    ccSet( CC::IntMask, true );
}

void c6502::op_sta(Addr addr) {
    write( addr, regA );
}

void c6502::op_stx(Addr addr) {
    write( addr, regX );
}

void c6502::op_sty(Addr addr) {
    write( addr, regY );
}

void c6502::op_tax() {
    read( pc() );

    regX = regA;

    updateNZ( regA );
}

void c6502::op_tay() {
    read( pc() );

    regY = regA;

    updateNZ( regA );
}

void c6502::op_txa(Addr addr) {
    regA = regX;

    updateNZ( regA );
}

void c6502::op_tya(Addr addr) {
    regA = regY;

    updateNZ( regA );
}

void c6502::op_txs(Addr addr) {
    regSp = regX;
}

void c6502::updateNZ(uint8_t val) {
    ccSet( CC::Negative, val & 0x80 );
    ccSet( CC::Zero, val == 0 );
}
