cmake_minimum_required(VERSION 3.22)
project(recordgolden C)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(BOX2D_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../dbox2d-ref" CACHE PATH "Frozen Box2D reference source directory")

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

add_executable(recordgolden main.c)
target_link_libraries(recordgolden PRIVATE box2d)
