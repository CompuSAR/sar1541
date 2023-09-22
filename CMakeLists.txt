cmake_minimum_required(VERSION 3.12.0)
project(sar1541)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_COMPILER clang++)
set(TH_DIR ../test_harness/perfect6502)

include_directories(${TH_DIR})

add_executable(verify_cpu c6502.cpp verify_cpu.cpp ${TH_DIR}/readmem.cpp)
