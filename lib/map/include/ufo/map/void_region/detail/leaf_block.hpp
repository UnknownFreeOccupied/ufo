/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef UFO_MAP_VOID_REGION_DETAIL_LEAF_BLOCK_HPP
#define UFO_MAP_VOID_REGION_DETAIL_LEAF_BLOCK_HPP

// UFO
#include <ufo/container/tree/index.hpp>
#include <ufo/utility/bit_set.hpp>

// STL
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>

namespace ufo::detail
{
template <std::size_t Dim, std::size_t BF>
struct VoidRegionLeafBlock {
	using value_type  = std::uint8_t;
	using offset_type = TreeIndex::offset_type;

	std::array<value_type, BF> nodes;

	[[nodiscard]] BitSet<BF> voidRegion() const
	{
		using T = typename BitSet<BF>::value_type;
		T data{};
		for (offset_type i{}; BF > i; ++i) {
			data |= static_cast<T>(voidRegion(i)) << i;
		}
		return BitSet<BF>(data);
	}

	[[nodiscard]] bool voidRegion(offset_type offset) const
	{
		assert(BF > offset);
		return !!nodes[offset];
	}

	void voidRegionSet(bool value) { nodes.fill(static_cast<std::uint8_t>(value)); }

	void voidRegionSet(offset_type offset, bool value)
	{
		assert(BF > offset);
		nodes[offset] = static_cast<std::uint8_t>(value);
	}
};

template <std::size_t Dim, std::size_t BF>
bool operator==(VoidRegionLeafBlock<Dim, BF> const& lhs,
                VoidRegionLeafBlock<Dim, BF> const& rhs)
{
	return lhs.nodes == rhs.nodes;
}

template <std::size_t Dim, std::size_t BF>
bool operator!=(VoidRegionLeafBlock<Dim, BF> const& lhs,
                VoidRegionLeafBlock<Dim, BF> const& rhs)
{
	return !(lhs == rhs);
};
}  // namespace ufo::detail

#endif  // UFO_MAP_VOID_REGION_DETAIL_LEAF_BLOCK_HPP