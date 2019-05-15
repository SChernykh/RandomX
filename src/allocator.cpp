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

#include <new>
#include "allocator.hpp"
#include "intrin_portable.h"
#include "virtual_memory.hpp"
#include "common.hpp"

namespace randomx {

	template<size_t alignment>
	void* AlignedAllocator<alignment>::allocMemory(size_t count) {
		void *mem = rx_aligned_alloc(count, alignment);
		if (mem == nullptr)
			throw std::bad_alloc();
		return mem;
	}

	template<size_t alignment>
	void AlignedAllocator<alignment>::freeMemory(void* ptr, size_t count) {
		rx_aligned_free(ptr);
	}

	template class AlignedAllocator<CacheLineSize>;

	void* LargePageAllocator::allocMemory(size_t count) {
		return allocLargePagesMemory(count);
	}

	void LargePageAllocator::freeMemory(void* ptr, size_t count) {
		freePagedMemory(ptr, count);
	};

}