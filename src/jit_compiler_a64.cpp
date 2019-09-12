/*
Copyright (c) 2018-2019, tevador <tevador@gmail.com>
Copyright (c) 2019, SChernykh    <https://github.com/SChernykh>

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

constexpr uint32_t B          = 0x14000000;
constexpr uint32_t EOR        = 0xCA000000;
constexpr uint32_t EOR32      = 0x4A000000;
constexpr uint32_t ADD        = 0x8B000000;
constexpr uint32_t MOVZ       = 0xD2800000;
constexpr uint32_t MOVN       = 0x92800000;
constexpr uint32_t MOVK       = 0xF2800000;
constexpr uint32_t ADD_IMM_LO = 0x91000000;
constexpr uint32_t ADD_IMM_HI = 0x91400000;

}

namespace randomx {

static const size_t CodeSize = ((uint8_t*)randomx_program_aarch64_end) - ((uint8_t*)randomx_program_aarch64);
static const size_t PrologueSize = ((uint8_t*)randomx_program_aarch64_vm_instructions) - ((uint8_t*)randomx_program_aarch64);

constexpr uint32_t IntRegMap[8] = { 4, 5, 6, 7, 12, 13, 14, 15 };

template<typename T> static constexpr size_t Log2(T value) { return (value > 1) ? (Log2(value / 2) + 1) : 0; }

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

void JitCompilerA64::emitMovImmediate(uint32_t dst, uint32_t imm, uint8_t* code, uint32_t& codePos)
{
	uint32_t k = codePos;

	if (imm < (1 << 16))
	{
		// movz tmp_reg, imm32 (16 low bits)
		emit32(ARMV8A::MOVZ | dst | (imm << 5), code, k);
	}
	else
	{
		if (static_cast<int32_t>(imm) < 0)
		{
			// movn tmp_reg, ~imm32 (16 high bits)
			emit32(ARMV8A::MOVN | dst | (1 << 21) | ((~imm >> 16) << 5), code, k);
		}
		else
		{
			// movz tmp_reg, imm32 (16 high bits)
			emit32(ARMV8A::MOVZ | dst | (1 << 21) | ((imm >> 16) << 5), code, k);
		}

		// movk tmp_reg, imm32 (16 low bits)
		emit32(ARMV8A::MOVK | dst | ((imm & 0xFFFF) << 5), code, k);
	}

	codePos = k;
}

void JitCompilerA64::emitAddImmediate(uint32_t dst, uint32_t src, uint32_t imm, uint8_t* code, uint32_t& codePos)
{
	uint32_t k = codePos;

	if (imm < (1 << 24))
	{
		const uint32_t imm_lo = imm & ((1 << 12) - 1);
		const uint32_t imm_hi = imm >> 12;

		if (imm_lo && imm_hi)
		{
			emit32(ARMV8A::ADD_IMM_LO | dst | (src << 5) | (imm_lo << 10), code, k);
			emit32(ARMV8A::ADD_IMM_HI | dst | (dst << 5) | (imm_hi << 10), code, k);
		}
		else if (imm_lo)
		{
			emit32(ARMV8A::ADD_IMM_LO | dst | (src << 5) | (imm_lo << 10), code, k);
		}
		else
		{
			emit32(ARMV8A::ADD_IMM_HI | dst | (src << 5) | (imm_hi << 10), code, k);
		}
	}
	else
	{
		constexpr uint32_t tmp_reg = 21;
		emitMovImmediate(tmp_reg, imm, code, k);

		// add dst, src, tmp_reg
		emit32(ARMV8A::ADD | dst | (src << 5) | (tmp_reg << 16), code, k);
	}

	codePos = k;
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
		emitAddImmediate(dst, dst, instr.getImm32(), code, k);

	codePos = k;
}

void JitCompilerA64::h_IADD_M(Instruction& instr, int i, uint32_t& codePos)
{
	uint32_t k = codePos;

	const uint32_t src = IntRegMap[instr.src];
	const uint32_t dst = IntRegMap[instr.dst];
	uint32_t imm = instr.getImm32();

	constexpr uint32_t tmp_reg = 21;

	if (src != dst)
	{
		imm &= instr.getModMem() ? (RANDOMX_SCRATCHPAD_L1 - 1) : (RANDOMX_SCRATCHPAD_L2 - 1);
		emitAddImmediate(tmp_reg, src, imm, code, k);

		constexpr uint32_t t = 0x927d0000 | tmp_reg | (tmp_reg << 5);
		constexpr uint32_t andInstrL1 = t | ((Log2(RANDOMX_SCRATCHPAD_L1) - 4) << 10);
		constexpr uint32_t andInstrL2 = t | ((Log2(RANDOMX_SCRATCHPAD_L2) - 4) << 10);

		emit32(instr.getModMem() ? andInstrL1 : andInstrL2, code, k);

		// ldr tmp_reg, [x2, tmp_reg]
		emit32(0xf8606840 | tmp_reg | (tmp_reg << 16), code, k);
	}
	else
	{
		imm = (imm & ScratchpadL3Mask) >> 3;
		emitMovImmediate(tmp_reg, imm, code, k);

		// ldr tmp_reg, [x2, tmp_reg, lsl 3]
		emit32(0xf8607840 | tmp_reg | (tmp_reg << 16), code, k);
	}

	// add dst, dst, tmp_reg
	emit32(ARMV8A::ADD | dst | (dst << 5) | (tmp_reg << 16), code, k);

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
