/*
Copyright (c) 2018-2019, tevador <tevador@gmail.com>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
	* Redistributions of source code must retain the above copyright
	  notice, this list of conditions and the following disclaimer.
	* Redistributions in binary form must reproduce the above copyright
	  notice, this list of conditions and the following disclaimer in the
	  documentation and/or other materials provided with the distribution.
	* Neither the name of the copyright holder nor the
	  names of its contributors may be used to endorse or promote products
	  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "jit_compiler_a64.hpp"
#include "program.hpp"
#include "virtual_memory.hpp"

namespace ARMV8A {

constexpr uint32_t B     = 0x14000000;
constexpr uint32_t EOR   = 0xCA000000;
constexpr uint32_t EOR32 = 0x4A000000;
constexpr uint32_t ADD   = 0x8B000000;
constexpr uint32_t MOVZ  = 0xD2800000;
constexpr uint32_t MOVN  = 0x92800000;
constexpr uint32_t MOVK  = 0xF2800000;

}

namespace randomx {

static const size_t CodeSize = ((uint8_t*)randomx_program_aarch64_end) - ((uint8_t*)randomx_program_aarch64);
static const size_t PrologueSize = ((uint8_t*)randomx_program_aarch64_vm_instructions) - ((uint8_t*)randomx_program_aarch64);

constexpr uint32_t IntRegMap[8] = { 4, 5, 6, 7, 12, 13, 14, 15 };

JitCompilerA64::JitCompilerA64()
	: code((uint8_t*) allocMemoryPages(CodeSize))
{
	memcpy(code, (void*) randomx_program_aarch64, CodeSize);
	enableAll();
}

JitCompilerA64::~JitCompilerA64()
{
	freePagedMemory(code, CodeSize);
}

void JitCompilerA64::enableAll()
{
	setPagesRWX(code, CodeSize);
}

void JitCompilerA64::generateProgram(Program& program, ProgramConfiguration& config)
{
	uint32_t codePos = PrologueSize;

	for (uint32_t i = 0; i < program.getSize(); ++i)
	{
		Instruction& instr = program(i);
		instr.src %= RegistersCount;
		instr.dst %= RegistersCount;
		(this->*engine[instr.opcode])(instr, i, codePos);
	}

	// Update spMix2
	// eor w11, config.readReg2, config.readReg3
	emit32(ARMV8A::EOR32 | 11 | (IntRegMap[config.readReg2] << 5) | (IntRegMap[config.readReg3] << 16), code, codePos);

	// Jump back to the main loop
	const uint32_t offset = (((uint8_t*)randomx_program_aarch64_vm_instructions_end) - ((uint8_t*)randomx_program_aarch64)) - codePos;
	emit32(ARMV8A::B | (offset / 4), code, codePos);

	// Update spMix1
	// eor x10, config.readReg0, config.readReg1
	codePos = ((uint8_t*)randomx_program_aarch64_update_spMix1) - ((uint8_t*)randomx_program_aarch64);
	emit32(ARMV8A::EOR | 10 | (IntRegMap[config.readReg0] << 5) | (IntRegMap[config.readReg1] << 16), code, codePos);

#ifdef __GNUC__
	__builtin___clear_cache(reinterpret_cast<char*>(code + PrologueSize), reinterpret_cast<char*>(code + codePos));
#endif
}

size_t JitCompilerA64::getCodeSize()
{
	return CodeSize;
}

void JitCompilerA64::h_IADD_RS(Instruction& instr, int i, uint32_t& codePos)
{
	uint32_t k = codePos;

	const uint32_t src = IntRegMap[instr.src];
	const uint32_t dst = IntRegMap[instr.dst];
	const uint32_t shift = instr.getModShift();

	// add dst, src << shift
	emit32(ARMV8A::ADD | dst | (dst << 5) | (shift << 10) | (src << 16), code, k);

	if (instr.dst == RegisterNeedsDisplacement)
	{
		const uint32_t imm32 = instr.getImm32();
		if (static_cast<int32_t>(imm32) < 0)
		{
			// movn x21, ~imm32 (16 high bits)
			emit32(ARMV8A::MOVN | 21 | (1 << 21) | ((~imm32 >> 16) << 5), code, k);
		}
		else
		{
			// movz x21, imm32 (16 high bits)
			emit32(ARMV8A::MOVZ | 21 | (1 << 21) | ((imm32 >> 16) << 5), code, k);
		}
		// movk x21, imm32 (16 low bits)
		emit32(ARMV8A::MOVK | 21 | ((imm32 & 0xFFFF) << 5), code, k);
		// add dst, x21
		emit32(ARMV8A::ADD | dst | (dst << 5) | (21 << 16), code, k);
	}

	codePos = k;
}

void JitCompilerA64::h_NOP(Instruction& instr, int i, uint32_t& codePos)
{
}

#include "instruction_weights.hpp"
#define INST_HANDLE(x) REPN(&JitCompilerA64::h_##x, WT(x))

	InstructionGeneratorA64 JitCompilerA64::engine[256] = {
		INST_HANDLE(IADD_RS)
		INST_HANDLE(IADD_M)
		INST_HANDLE(ISUB_R)
		INST_HANDLE(ISUB_M)
		INST_HANDLE(IMUL_R)
		INST_HANDLE(IMUL_M)
		INST_HANDLE(IMULH_R)
		INST_HANDLE(IMULH_M)
		INST_HANDLE(ISMULH_R)
		INST_HANDLE(ISMULH_M)
		INST_HANDLE(IMUL_RCP)
		INST_HANDLE(INEG_R)
		INST_HANDLE(IXOR_R)
		INST_HANDLE(IXOR_M)
		INST_HANDLE(IROR_R)
		INST_HANDLE(IROL_R)
		INST_HANDLE(ISWAP_R)
		INST_HANDLE(FSWAP_R)
		INST_HANDLE(FADD_R)
		INST_HANDLE(FADD_M)
		INST_HANDLE(FSUB_R)
		INST_HANDLE(FSUB_M)
		INST_HANDLE(FSCAL_R)
		INST_HANDLE(FMUL_R)
		INST_HANDLE(FDIV_M)
		INST_HANDLE(FSQRT_R)
		INST_HANDLE(CBRANCH)
		INST_HANDLE(CFROUND)
		INST_HANDLE(ISTORE)
		INST_HANDLE(NOP)
	};

}
