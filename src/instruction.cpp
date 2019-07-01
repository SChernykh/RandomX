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

#include "instruction.hpp"
#include "common.hpp"

namespace randomx {

	void Instruction::print(std::ostream& os) const {
		os << names[opcode] << " ";
		auto handler = engine[opcode];
		(this->*handler)(os);
	}

	void Instruction::genAddressReg(std::ostream& os, int srcIndex) const {
		os << (getModMem() ? "L1" : "L2") << "[r" << srcIndex << std::showpos << (int32_t)getImm32() << std::noshowpos << "]";
	}

	void Instruction::genAddressRegDst(std::ostream& os, int dstIndex) const {
		if (getModCond() < StoreL3Condition)
			os << (getModMem() ? "L1" : "L2");
		else
			os << "L3";
		os << "[r" << dstIndex << std::showpos << (int32_t)getImm32() << std::noshowpos << "]";
	}

	void Instruction::genAddressImm(std::ostream& os) const {
		os << "L3" << "[" << (getImm32() & ScratchpadL3Mask) << "]";
	}

	void Instruction::h_IADD_RS(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		os << "r" << dstIndex << ", r" << srcIndex;
		if(dstIndex == RegisterNeedsDisplacement) {
			os << ", " << (int32_t)getImm32();
		}
		os << ", SHFT " << getModShift() << std::endl;
	}

	void Instruction::h_IADD_M(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		if (dstIndex != srcIndex) {
			os << "r" << dstIndex << ", ";
			genAddressReg(os, srcIndex);
			os << std::endl;
		}
		else {
			os << "r" << dstIndex << ", ";
			genAddressImm(os);
			os << std::endl;
		}
	}

	void Instruction::h_ISUB_R(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		if (dstIndex != srcIndex) {
			os << "r" << dstIndex << ", r" << srcIndex << std::endl;
		}
		else {
			os << "r" << dstIndex << ", " << (int32_t)getImm32() << std::endl;
		}
	}

	void Instruction::h_ISUB_M(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		if (dstIndex != srcIndex) {
			os << "r" << dstIndex << ", ";
			genAddressReg(os, srcIndex);
			os << std::endl;
		}
		else {
			os << "r" << dstIndex << ", ";
			genAddressImm(os);
			os << std::endl;
		}
	}

	void Instruction::h_IMUL_R(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		if (dstIndex != srcIndex) {
			os << "r" << dstIndex << ", r" << srcIndex << std::endl;
		}
		else {
			os << "r" << dstIndex << ", " << (int32_t)getImm32() << std::endl;
		}
	}

	void Instruction::h_IMUL_M(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		if (dstIndex != srcIndex) {
			os << "r" << dstIndex << ", ";
			genAddressReg(os, srcIndex);
			os << std::endl;
		}
		else {
			os << "r" << dstIndex << ", ";
			genAddressImm(os);
			os << std::endl;
		}
	}

	void Instruction::h_IMULH_R(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		os << "r" << dstIndex << ", r" << srcIndex << std::endl;
	}

	void Instruction::h_IMULH_M(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		if (dstIndex != srcIndex) {
			os << "r" << dstIndex << ", ";
			genAddressReg(os, srcIndex);
			os << std::endl;
		}
		else {
			os << "r" << dstIndex << ", ";
			genAddressImm(os);
			os << std::endl;
		}
	}

	void Instruction::h_ISMULH_R(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		os << "r" << dstIndex << ", r" << srcIndex << std::endl;
	}

	void Instruction::h_ISMULH_M(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		if (dstIndex != srcIndex) {
			os << "r" << dstIndex << ", ";
			genAddressReg(os, srcIndex);
			os << std::endl;
		}
		else {
			os << "r" << dstIndex << ", ";
			genAddressImm(os);
			os << std::endl;
		}
	}

	void Instruction::h_INEG_R(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		os << "r" << dstIndex << std::endl;
	}

	void Instruction::h_IXOR_R(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		if (dstIndex != srcIndex) {
			os << "r" << dstIndex << ", r" << srcIndex << std::endl;
		}
		else {
			os << "r" << dstIndex << ", " << (int32_t)getImm32() << std::endl;
		}
	}

	void Instruction::h_IXOR_M(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		if (dstIndex != srcIndex) {
			os << "r" << dstIndex << ", ";
			genAddressReg(os, srcIndex);
			os << std::endl;
		}
		else {
			os << "r" << dstIndex << ", ";
			genAddressImm(os);
			os << std::endl;
		}
	}

	void Instruction::h_IROR_R(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		if (dstIndex != srcIndex) {
			os << "r" << dstIndex << ", r" << srcIndex << std::endl;
		}
		else {
			os << "r" << dstIndex << ", " << (getImm32() & 63) << std::endl;
		}
	}

	void Instruction::h_IROL_R(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		if (dstIndex != srcIndex) {
			os << "r" << dstIndex << ", r" << srcIndex << std::endl;
		}
		else {
			os << "r" << dstIndex << ", " << (getImm32() & 63) << std::endl;
		}
	}

	void Instruction::h_IMUL_RCP(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		os << "r" << dstIndex << ", " << getImm32() << std::endl;
	}

	void Instruction::h_ISWAP_R(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		os << "r" << dstIndex << ", r" << srcIndex << std::endl;
	}

	void Instruction::h_FSWAP_R(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		const char reg = (dstIndex >= RegisterCountFlt) ? 'e' : 'f';
		dstIndex %= RegisterCountFlt;
		os << reg << dstIndex << std::endl;
	}

	void Instruction::h_FADD_R(std::ostream& os) const {
		auto dstIndex = dst % RegisterCountFlt;
		auto srcIndex = src % RegisterCountFlt;
		os << "f" << dstIndex << ", a" << srcIndex << std::endl;
	}

	void Instruction::h_FADD_M(std::ostream& os) const {
		auto dstIndex = dst % RegisterCountFlt;
		auto srcIndex = src % RegistersCount;
		os << "f" << dstIndex << ", ";
		genAddressReg(os, srcIndex);
		os << std::endl;
	}

	void Instruction::h_FSUB_R(std::ostream& os) const {
		auto dstIndex = dst % RegisterCountFlt;
		auto srcIndex = src % RegisterCountFlt;
		os << "f" << dstIndex << ", a" << srcIndex << std::endl;
	}

	void Instruction::h_FSUB_M(std::ostream& os) const {
		auto dstIndex = dst % RegisterCountFlt;
		auto srcIndex = src % RegistersCount;
		os << "f" << dstIndex << ", ";
		genAddressReg(os, srcIndex);
		os << std::endl;
	}

	void Instruction::h_FSCAL_R(std::ostream& os) const {
		auto dstIndex = dst % RegisterCountFlt;
		os << "f" << dstIndex << std::endl;
	}

	void Instruction::h_FMUL_R(std::ostream& os) const {
		auto dstIndex = dst % RegisterCountFlt;
		auto srcIndex = src % RegisterCountFlt;
		os << "e" << dstIndex << ", a" << srcIndex << std::endl;
	}

	void Instruction::h_FDIV_M(std::ostream& os) const {
		auto dstIndex = dst % RegisterCountFlt;
		auto srcIndex = src % RegistersCount;
		os << "e" << dstIndex << ", ";
		genAddressReg(os, srcIndex);
		os << std::endl;
	}

	void Instruction::h_FSQRT_R(std::ostream& os) const {
		auto dstIndex = dst % RegisterCountFlt;
		os << "e" << dstIndex << std::endl;
	}

	void Instruction::h_CFROUND(std::ostream& os) const {
		auto srcIndex = src % RegistersCount;
		os << "r" << srcIndex << ", " << (getImm32() & 63) << std::endl;
	}

	void Instruction::h_CBRANCH(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		os << "r" << dstIndex << ", " << (int32_t)getImm32() << ", COND " << (int)(getModCond()) << std::endl;
	}

	void  Instruction::h_ISTORE(std::ostream& os) const {
		auto dstIndex = dst % RegistersCount;
		auto srcIndex = src % RegistersCount;
		genAddressRegDst(os, dstIndex);
		os << ", r" << srcIndex << std::endl;
	}

	void  Instruction::h_NOP(std::ostream& os) const {
		os << std::endl;
	}

	const char* Instruction::names[256] = {};
	InstructionFormatter Instruction::engine[256] = {};

}