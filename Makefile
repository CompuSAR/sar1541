TH_DIR=../test_harness/perfect6502

CC=$(CXX)
CPPFLAGS=-I$(TH_DIR)
CXXFLAGS=-std=c++20 -g

verify_cpu: verify_cpu.o c6502.o $(TH_DIR)/readmem.o

c6502.o: c6502.h

c6502.h: Bus.h


clean:
	$(RM) *.o verify_cpu
