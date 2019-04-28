/*
Copyright (c) 2019 tevador

This file is part of RandomX.

RandomX is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

RandomX is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with RandomX.  If not, see<http://www.gnu.org/licenses/>.
*/

#pragma once

#include <cstdint>
#include "instruction.hpp"
#include "configuration.h"

namespace randomx {

	class SuperscalarProgram {
	public:
		Instruction& operator()(int pc) {
			return programBuffer[pc];
		}
		friend std::ostream& operator<<(std::ostream& os, const SuperscalarProgram& p) {
			p.print(os);
			return os;
		}
		uint32_t getSize() {
			return size;
		}
		void setSize(uint32_t val) {
			size = val;
		}
		int getAddressRegister() {
			return addrReg;
		}
		void setAddressRegister(uint32_t val) {
			addrReg = val;
		}

		Instruction programBuffer[RANDOMX_SUPERSCALAR_MAX_SIZE];
		uint32_t size;
		int addrReg;
		double ipc;
		int codeSize;
		int macroOps;
		int decodeCycles;
		int cpuLatency;
		int asicLatency;
		int mulCount;
		int cpuLatencies[8];
		int asicLatencies[8];
	private:
		void print(std::ostream& os) const {
			for (unsigned i = 0; i < size; ++i) {
				auto instr = programBuffer[i];
				os << instr;
			}
		}
	};

}