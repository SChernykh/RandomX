# Copyright (c) 2019, The Monero Project
# 
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without modification, are
# permitted provided that the following conditions are met:
# 
# 1. Redistributions of source code must retain the above copyright notice, this list of
#    conditions and the following disclaimer.
# 
# 2. Redistributions in binary form must reproduce the above copyright notice, this list
#    of conditions and the following disclaimer in the documentation and/or other
#    materials provided with the distribution.
# 
# 3. Neither the name of the copyright holder nor the names of its contributors may be
#    used to endorse or promote products derived from this software without specific
#    prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
# EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
# THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
# PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
# THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.5)

project(RandomX)

set(randomx_sources
src/aes_hash.cpp
src/aes_hash.hpp
src/allocator.cpp
src/allocator.hpp
src/argon2.h
src/argon2_avx2.c
src/argon2_core.c
src/argon2_core.h
src/argon2_ref.c
src/argon2_ssse3.c
src/blake2_generator.cpp
src/blake2_generator.hpp
src/bytecode_machine.cpp
src/bytecode_machine.hpp
src/common.hpp
src/configuration.h
src/cpu.cpp
src/cpu.hpp
src/dataset.cpp
src/dataset.hpp
src/instruction.cpp
src/instruction.hpp
src/instructions_portable.cpp
src/instruction_weights.hpp
src/intrin_portable.h
src/jit_compiler.hpp
src/jit_compiler_fallback.hpp
src/program.hpp
src/randomx.cpp
src/randomx.h
src/reciprocal.c
src/reciprocal.h
src/soft_aes.cpp
src/soft_aes.h
src/superscalar.cpp
src/superscalar.hpp
src/superscalar_program.hpp
src/virtual_machine.cpp
src/virtual_machine.hpp
src/virtual_memory.c
src/virtual_memory.h
src/vm_compiled.cpp
src/vm_compiled.hpp
src/vm_compiled_light.cpp
src/vm_compiled_light.hpp
src/vm_interpreted.cpp
src/vm_interpreted.hpp
src/vm_interpreted_light.cpp
src/vm_interpreted_light.hpp
src/blake2/blake2-impl.h
src/blake2/blake2.h
src/blake2/blake2b.c
src/blake2/blamka-round-avx2.h
src/blake2/blamka-round-ref.h
src/blake2/blamka-round-ssse3.h
src/blake2/endian.h
)
if(NOT ARCH_ID)
  # allow cross compiling
  if(CMAKE_SYSTEM_PROCESSOR STREQUAL "")
    set(CMAKE_SYSTEM_PROCESSOR ${CMAKE_HOST_SYSTEM_PROCESSOR})
  endif()
  string(TOLOWER "${CMAKE_SYSTEM_PROCESSOR}" ARCH_ID)
endif()

if(NOT ARM_ID)
  set(ARM_ID "${ARCH_ID}")
endif()

if(NOT ARCH)
  set(ARCH "default")
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
  message(STATUS "Setting default build type: ${CMAKE_BUILD_TYPE}")
endif()

include(CheckCXXCompilerFlag)
include(CheckCCompilerFlag)
include(CheckSymbolExists)

function(add_flag flag)
  string(REPLACE "-" "_" supported_cxx ${flag}_cxx)
  check_cxx_compiler_flag(${flag} ${supported_cxx})
  if(${${supported_cxx}})
    message(STATUS "Setting CXX flag ${flag}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${flag}" PARENT_SCOPE)
  endif()
  string(REPLACE "-" "_" supported_c ${flag}_c)
  check_c_compiler_flag(${flag} ${supported_c})
  if(${${supported_c}})
    message(STATUS "Setting C flag ${flag}")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${flag}" PARENT_SCOPE)
  endif()
endfunction()

# x86-64
if ((CMAKE_SIZEOF_VOID_P EQUAL 8) AND (ARCH_ID STREQUAL "x86_64" OR ARCH_ID STREQUAL "x86-64" OR ARCH_ID STREQUAL "amd64"))
  list(APPEND randomx_sources
    src/jit_compiler_x86.cpp
    src/jit_compiler_x86.hpp
    src/jit_compiler_x86_static.hpp
)

  if(MSVC)
    set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} /MP /W0 /MTd")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MP /W0 /MTd")

    set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} /MP /W0 /MT /O2 /Oi /Ob2 /Os /DNDEBUG /GL")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MP /W0 /MT /O2 /Oi /Ob2 /Os /DNDEBUG /GL")

    set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} /MP /W0 /MT /DRELWITHDEBINFO")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /MP /W0 /MT /DRELWITHDEBINFO")

    enable_language(ASM_MASM)
    list(APPEND randomx_sources src/jit_compiler_x86_static.asm)

    set_property(SOURCE src/jit_compiler_x86_static.asm PROPERTY LANGUAGE ASM_MASM)

    set_source_files_properties(src/argon2_avx2.c COMPILE_FLAGS /arch:AVX2)

    set(CMAKE_C_FLAGS_RELWITHDEBINFO "${CMAKE_C_FLAGS_RELWITHDEBINFO} /DRELWITHDEBINFO")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /DRELWITHDEBINFO")

    add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/src/asm/configuration.asm
      COMMAND powershell -ExecutionPolicy Bypass -File h2inc.ps1 ..\\src\\configuration.h > ..\\src\\asm\\configuration.asm SET ERRORLEVEL = 0
      COMMENT "Generating configuration.asm at ${CMAKE_CURRENT_SOURCE_DIR}"
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/vcxproj)
  else()
    add_flag("-Os")
    if (WITH_LTO)
      add_flag("-flto")
      add_flag("-fuse-linker-plugin")
    endif()

    list(APPEND randomx_sources src/jit_compiler_x86_static.S)

    # cheat because cmake and ccache hate each other
    set_property(SOURCE src/jit_compiler_x86_static.S PROPERTY LANGUAGE C)
    set_property(SOURCE src/jit_compiler_x86_static.S PROPERTY XCODE_EXPLICIT_FILE_TYPE sourcecode.asm)

    if(ARCH STREQUAL "native")
      add_flag("-march=native")
    else()
      # default build has hardware AES enabled (software AES can be selected at runtime)
      add_flag("-maes")
      check_c_compiler_flag(-mssse3 HAVE_SSSE3)
      if(HAVE_SSSE3)
        set_source_files_properties(src/argon2_ssse3.c COMPILE_FLAGS -mssse3)
      endif()
      check_c_compiler_flag(-mavx2 HAVE_AVX2)
      if(HAVE_AVX2)
        set_source_files_properties(src/argon2_avx2.c COMPILE_FLAGS -mavx2)
      endif()
    endif()
  endif()
endif()

# PowerPC
if(ARCH_ID STREQUAL "ppc64" OR ARCH_ID STREQUAL "ppc64le")
  if(ARCH STREQUAL "native")
    add_flag("-mcpu=native")
  endif()
  # PowerPC AES requires ALTIVEC (POWER7+), so it cannot be enabled in the default build
endif()

# ARMv8
if(ARM_ID STREQUAL "aarch64" OR ARM_ID STREQUAL "arm64" OR ARM_ID STREQUAL "armv8-a")
  list(APPEND randomx_sources
    src/jit_compiler_a64_static.S
    src/jit_compiler_a64.cpp
    src/jit_compiler_a64.hpp
    src/jit_compiler_a64_static.hpp
  )
  # cheat because cmake and ccache hate each other
  set_property(SOURCE src/jit_compiler_a64_static.S PROPERTY LANGUAGE C)
  set_property(SOURCE src/jit_compiler_a64_static.S PROPERTY XCODE_EXPLICIT_FILE_TYPE sourcecode.asm)

  # not sure if this check is needed
  include(CheckIncludeFile)
  check_include_file(asm/hwcap.h HAVE_HWCAP)
  if(HAVE_HWCAP)
    add_definitions(-DHAVE_HWCAP)
  endif()

  if(ARCH STREQUAL "native")
    add_flag("-march=native")
  else()
    # default build has hardware AES enabled (software AES can be selected at runtime)
    add_flag("-march=armv8-a+crypto")
  endif()
endif()

check_symbol_exists("__builtin___clear_cache" "stdlib.h" HAVE_BUILTIN_CLEAR_CACHE)
if (HAVE_BUILTIN_CLEAR_CACHE)
  add_definitions(-DHAVE_BUILTIN_CLEAR_CACHE)
endif()

# RISC-V
if(ARCH_ID STREQUAL "riscv64")
  list(APPEND randomx_sources
    src/jit_compiler_rv64_static.S
    src/jit_compiler_rv64.cpp
    src/jit_compiler_rv64.hpp
    src/jit_compiler_rv64_static.hpp
  )
  # cheat because cmake and ccache hate each other
  set_property(SOURCE src/jit_compiler_rv64_static.S PROPERTY LANGUAGE C)
  set_property(SOURCE src/jit_compiler_rv64_static.S PROPERTY XCODE_EXPLICIT_FILE_TYPE sourcecode.asm)

  # default build uses the RV64GC baseline
  set(RVARCH "rv64gc")

  # for native builds, enable Zba and Zbb if supported by the CPU
  if(ARCH STREQUAL "native")
    enable_language(ASM)
    try_run(RANDOMX_ZBA_RUN_FAIL
        RANDOMX_ZBA_COMPILE_OK
        ${CMAKE_CURRENT_BINARY_DIR}/
        ${CMAKE_CURRENT_SOURCE_DIR}/src/tests/riscv64_zba.s
        COMPILE_DEFINITIONS "-march=rv64gc_zba")
    if (RANDOMX_ZBA_COMPILE_OK AND NOT RANDOMX_ZBA_RUN_FAIL)
      set(RVARCH "${RVARCH}_zba")
    endif()
    try_run(RANDOMX_ZBB_RUN_FAIL
        RANDOMX_ZBB_COMPILE_OK
        ${CMAKE_CURRENT_BINARY_DIR}/
        ${CMAKE_CURRENT_SOURCE_DIR}/src/tests/riscv64_zbb.s
        COMPILE_DEFINITIONS "-march=rv64gc_zbb")
    if (RANDOMX_ZBB_COMPILE_OK AND NOT RANDOMX_ZBB_RUN_FAIL)
      set(RVARCH "${RVARCH}_zbb")
    endif()
  endif()

  add_flag("-march=${RVARCH}")
endif()

set(RANDOMX_INCLUDE "${CMAKE_CURRENT_SOURCE_DIR}/src" CACHE STRING "RandomX Include path")

add_library(randomx ${randomx_sources})

set_property(TARGET randomx PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET randomx PROPERTY CXX_STANDARD 17)
set_property(TARGET randomx PROPERTY CXX_STANDARD_REQUIRED ON)
set_property(TARGET randomx PROPERTY PUBLIC_HEADER src/randomx.h)

include(GNUInstallDirs)
install(TARGETS randomx
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

add_executable(randomx-tests
  src/tests/tests.cpp)
target_link_libraries(randomx-tests
  PRIVATE randomx)
set_property(TARGET randomx-tests PROPERTY POSITION_INDEPENDENT_CODE ON)
set_property(TARGET randomx-tests PROPERTY CXX_STANDARD 17)
set_property(TARGET randomx-tests PROPERTY CXX_STANDARD_REQUIRED ON)
