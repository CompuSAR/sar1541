#include "readmem.h"

#include "Bus.h"

#include "c6502.h"

#include <assert.h>

class TestBus : public Bus {
    static constexpr size_t StartGraceCycles = 50;

    std::array<std::uint8_t, 65536> memory;
    ReadMem<8,8,16,4>   test_plan;
    size_t              cycles_until_start = StartGraceCycles;
    bool                cpuInReset = true;
    size_t              cycle_num = 0;


public:
    TestBus(const char *memory_image_file, const char *test_plan_file) : test_plan(test_plan_file) {
        ReadMem<8> memory_image(memory_image_file);

        while( memory_image.read_line() ) {
            memory[memory_image.address()] = memory_image[0];
        }
    }

    virtual uint8_t read( c6502 *cpu, Addr address, bool sync = false ) override {
        uint8_t ret = memory[address]; 
        if( cycles_until_start>0 ) {
            if( StartGraceCycles-cycles_until_start == 2 ) {
                cpu->setReset(false);
                cpuInReset = false;
            }

            if( address==0xfffc && !cpuInReset ) {
                cycles_until_start = 0;
                cycle_num = 1;
                std::cout<<"Reset vector read detected\n";
            } else {
                if( --cycles_until_start == 0 ) {
                    std::cerr<<"CPU failed to read the reset vector\n";
                    abort();
                }
            }
        }

        if( cycles_until_start==0 ) {
            std::cout<<std::dec<<cycle_num<<" R: "<<std::hex<<address<<" "<<int(ret)<<"\n";
            cycle_num++;
            bool valid = test_plan.read_line();
            assert(valid);

            check( test_plan[0], 1, "Read operation where write was expected", address, ret );
            check( test_plan[2], address, "Read from wrong address", address, ret );
            check( test_plan[1], ret, "Read wrong value from memory", address, ret );
        }

        return ret;
    }
    virtual void write( c6502 *cpu, Addr address, uint8_t value ) override {
        std::cout<<std::dec<<cycle_num<<" W: "<<std::hex<<address<<" "<<int(value)<<"\n";
        cycle_num++;

        bool valid = test_plan.read_line();
        if( valid ) {
            check( test_plan[0], 0, "Write operation where read was expected", address, value );
            check( test_plan[2], address, "Write to wrong address", address, value );
            check( test_plan[1], value, "Write of wrong value to memory", address, value );
        }

        memory[address] = value;
    }

private:
    void check(
            ReadMemSupport::DataContainer expected,
            ReadMemSupport::DataContainer actual,
            const char *message,
            Addr address,
            uint8_t data )
    {
        if( actual==expected )
            return;

        std::cerr<<"Validation failed on plan line "<<std::dec<<test_plan.lineNumber()<<
                ": "<<message<<" Expected "<<std::hex<<expected<<" got "<<actual<<
                " @"<<address<<" data "<<int(data)<<"\n";
        abort();
    }
};

int main(int argc, char *argv[]) {
    TestBus bus(argv[1], argv[2]);

    c6502 cpu(bus);

    cpu.setReset(true);

    cpu.runCpu();
}
