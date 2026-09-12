cmake_minimum_required(VERSION 3.22)
project(cbench C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

# The reference tree is not vendored here: it is Box2D v3.1.1, cloned
# wherever you like and named on the command line. The default is a sibling
# checkout of this repository, so `git clone --branch v3.1.1
# https://github.com/erincatto/box2d` next to it needs no flag.
set(BOX2D_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../box2d" CACHE PATH "Box2D v3.1.1 source directory")

if(NOT EXISTS "${BOX2D_SOURCE_DIR}/src/world.c")
	message(FATAL_ERROR "BOX2D_SOURCE_DIR=${BOX2D_SOURCE_DIR} does not hold a Box2D source tree. Clone Box2D v3.1.1 and pass -DBOX2D_SOURCE_DIR=<path>.")
endif()

# The port never fuses a multiply with an add, on any lane path, so the
# reference is held to the same rule. Without this the comparison would
# credit the C side with an instruction the port refuses to emit.
if(MSVC)
	add_compile_options(/fp:precise /fp:contract-)
elseif(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
	add_compile_options(-ffp-contract=off -fno-fast-math)
endif()

# The wide arm: b2FloatW becomes __m256 and B2_SIMD_WIDTH becomes 8, which
# is the width the port's avx2 lane path uses.
set(BOX2D_AVX2 ON CACHE BOOL "" FORCE)
set(BOX2D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX2D_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BOX2D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX2D_DOCS OFF CACHE BOOL "" FORCE)
set(BOX2D_VALIDATE OFF CACHE BOOL "" FORCE)

add_subdirectory("${BOX2D_SOURCE_DIR}" box2d)
add_subdirectory("${BOX2D_SOURCE_DIR}/shared" shared)

add_executable(cbench main.c)
target_include_directories(cbench PRIVATE "${BOX2D_SOURCE_DIR}/shared" "${BOX2D_SOURCE_DIR}/src")
target_link_libraries(cbench PRIVATE box2d shared)

# src/CMakeLists.txt sets BOX2D_AVX2 PRIVATE on the box2d target, so the
# harness includes core.h without it and would label an AVX2 library as
# sse2. The label and B2_SIMD_WIDTH come from that header, so the harness
# is told the same thing the library was told.
if(BOX2D_AVX2)
	target_compile_definitions(cbench PRIVATE BOX2D_AVX2)
endif()
if(BOX2D_DISABLE_SIMD)
	target_compile_definitions(cbench PRIVATE BOX2D_DISABLE_SIMD)
endif()

# taskpool.h supplies the enqueueTask/finishTask callbacks that b2CreateWorld
# requires before it honours workerCount. It uses Win32 condition variables on
# Windows and pthreads elsewhere.
if(NOT WIN32)
	find_package(Threads REQUIRED)
	target_link_libraries(cbench PRIVATE Threads::Threads)
endif()
