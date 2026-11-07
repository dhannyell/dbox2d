cmake_minimum_required(VERSION 3.22)
project(conformance C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(BOX2D_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../dbox2d-ref" CACHE PATH "Frozen Box2D reference source directory")

# The v3.2 headers add math_types.h; its presence selects the v2 trace format.
if(EXISTS "${BOX2D_SOURCE_DIR}/include/box2d/math_types.h")
	set(CONFORMANCE_V32 ON)
else()
	set(CONFORMANCE_V32 OFF)
endif()

# The trace header names the reference commit, so a trace says what produced it.
execute_process(
	COMMAND git -C "${BOX2D_SOURCE_DIR}" rev-parse --short=7 HEAD
	OUTPUT_VARIABLE CONFORMANCE_REF
	OUTPUT_STRIP_TRAILING_WHITESPACE
	RESULT_VARIABLE CONFORMANCE_REF_RESULT)
if(NOT CONFORMANCE_REF_RESULT EQUAL 0)
	message(FATAL_ERROR "cannot read the reference commit of ${BOX2D_SOURCE_DIR}")
endif()
message(STATUS "conformance reference ${CONFORMANCE_REF}, v3.2 format ${CONFORMANCE_V32}")

if(MSVC)
	add_compile_options(/fp:precise /fp:contract-)
elseif(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
	add_compile_options(-ffp-contract=off -fno-fast-math)
endif()

set(BOX2D_DISABLE_SIMD ON CACHE BOOL "" FORCE)
set(BOX2D_SAMPLES OFF CACHE BOOL "" FORCE)
set(BOX2D_BENCHMARKS OFF CACHE BOOL "" FORCE)
set(BOX2D_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(BOX2D_DOCS OFF CACHE BOOL "" FORCE)
set(BOX2D_VALIDATE OFF CACHE BOOL "" FORCE)

add_subdirectory("${BOX2D_SOURCE_DIR}" box2d)
add_subdirectory("${BOX2D_SOURCE_DIR}/shared" shared)

add_executable(conformance main.c)
target_include_directories(conformance PRIVATE "${BOX2D_SOURCE_DIR}/shared")
target_compile_definitions(conformance PRIVATE CONFORMANCE_REF="${CONFORMANCE_REF}")
if(CONFORMANCE_V32)
	target_compile_definitions(conformance PRIVATE CONFORMANCE_V32)
endif()
target_link_libraries(conformance PRIVATE box2d shared)
