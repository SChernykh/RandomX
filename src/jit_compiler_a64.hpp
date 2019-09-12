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

#pragma once

#include <cstdint>
#include <vector>
#include <stdexcept>
#include "common.hpp"
#include "jit_compiler_a64_static.hpp"

namespace randomx {

	class Program;
	class ProgramConfiguration;
	class SuperscalarProgram;
	class Instruction;

	typedef void(JitCompilerA64::*InstructionGeneratorA64)(Instruction&, int, uint32_t&);

	class JitCompilerA64 {
	public:
		JitCompilerA64();
		~JitCompilerA64();

		void generateProgram(Program&, ProgramConfiguration&);

		void generateProgramLight(Program&, ProgramConfiguration&, uint32_t) {
			
		}
		template<size_t N>
		void generateSuperscalarHash(SuperscalarProgram(&programs)[N], std::vector<uint64_t> &) {

		}
		void generateDatasetInitCode() {

		}
		ProgramFunc* getProgramFunc() {
			return reinterpret_cast<ProgramFunc*>(code);
		}
		DatasetInitFunc* getDatasetInitFunc() {
			return nullptr;
		}
		uint8_t* getCode() {
			return code;
		}
		size_t getCodeSize();

		void enableWriting() {}
		void enableExecution() {}
		void enableAll();

	private:
		static InstructionGeneratorA64 engine[256];
		uint8_t* code;

		static void emit32(uint32_t val, uint8_t* code, uint32_t& codePos)
		{
			*(uint32_t*)(code + codePos) = val;
			codePos += sizeof(val);
		}

		void emitMovImmediate(uint32_t dst, uint32_t imm, uint8_t* code, uint32_t& codePos);
		void emitAddImmediate(uint32_t dst, uint32_t src, uint32_t imm, uint8_t* code, uint32_t& codePos);

		void h_IADD_RS(Instruction&, int, uint32_t&);
		void h_IADD_M(Instruction&, int, uint32_t&);
		void h_NOP(Instruction&, int, uint32_t&);
	};
}
