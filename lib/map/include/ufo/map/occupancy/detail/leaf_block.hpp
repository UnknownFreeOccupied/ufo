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
#ifndef UFO_MAP_OCCUPANCY_DETAIL_LEAF_BLOCK_HPP
#define UFO_MAP_OCCUPANCY_DETAIL_LEAF_BLOCK_HPP

// UFO
#include <ufo/map/occupancy/state.hpp>

// STL
#include <array>
#include <cassert>
#include <cstddef>

namespace ufo::detail
{
template <class T>
struct OccupancyLeafNode {
	T occupancy;
};

template <std::size_t Dim, std::size_t BF, class T>
struct OccupancyLeafBlock {
	std::array<OccupancyLeafNode<T>, BF> nodes;

	[[nodiscard]] T occupancy(std::size_t index) const
	{
		assert(BF > index);
		return nodes[index].occupancy;
	}

	[[nodiscard]] std::array<T, BF> occupancy() const
	{
		std::array<T, BF> occupancies;
		for (std::size_t i{}; BF > i; ++i) {
			occupancies[i] = occupancy(i);
		}
		return occupancies;
	}

	void occupancySet(std::size_t index, T value)
	{
		assert(BF > index);
		nodes[index].occupancy = value;
	}

	void occupancySet(T value)
	{
		for (std::size_t i{}; BF > i; ++i) {
			occupancySet(i, value);
		}
	}
};

template <class T>
bool operator==(OccupancyLeafNode<T> const& lhs, OccupancyLeafNode<T> const& rhs)
{
	return lhs.occupancy == rhs.occupancy;
}

template <class T>
bool operator!=(OccupancyLeafNode<T> const& lhs, OccupancyLeafNode<T> const& rhs)
{
	return !(lhs == rhs);
};

template <std::size_t Dim, std::size_t BF, class T>
bool operator==(OccupancyLeafBlock<Dim, BF, T> const& lhs,
                OccupancyLeafBlock<Dim, BF, T> const& rhs)
{
	return lhs.nodes == rhs.nodes;
}

template <std::size_t Dim, std::size_t BF, class T>
bool operator!=(OccupancyLeafBlock<Dim, BF, T> const& lhs,
                OccupancyLeafBlock<Dim, BF, T> const& rhs)
{
	return !(lhs == rhs);
};
}  // namespace ufo::detail

#endif  // UFO_MAP_OCCUPANCY_DETAIL_LEAF_BLOCK_HPP