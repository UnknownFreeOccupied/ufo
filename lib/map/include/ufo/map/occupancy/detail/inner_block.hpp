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
#ifndef UFO_MAP_OCCUPANCY_DETAIL_INNER_BLOCK_HPP
#define UFO_MAP_OCCUPANCY_DETAIL_INNER_BLOCK_HPP

// UFO
#include <ufo/map/occupancy/state.hpp>

// STL
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace ufo::detail
{
template <class T>
struct OccupancyInnerNode {
	static constexpr std::uint8_t const CONT_FRE_IDX = 2u;
	static constexpr std::uint8_t const CONT_UNK_IDX = 3u;
	static constexpr std::uint8_t const CONT_OCC_IDX = 4u;
	static constexpr std::uint8_t const CONT_FRE_BIT = 1u << CONT_FRE_IDX;
	static constexpr std::uint8_t const CONT_UNK_BIT = 1u << CONT_UNK_IDX;
	static constexpr std::uint8_t const CONT_OCC_BIT = 1u << CONT_OCC_IDX;

	T            occupancy;
	std::uint8_t data;
};

template <>
struct OccupancyInnerNode<OccupancyState> {
	static constexpr std::uint8_t const CONT_FRE_IDX = 2u;
	static constexpr std::uint8_t const CONT_UNK_IDX = 3u;
	static constexpr std::uint8_t const CONT_OCC_IDX = 4u;
	static constexpr std::uint8_t const OCC_BITS     = 0b11u;
	static constexpr std::uint8_t const CONT_FRE_BIT = 1u << CONT_FRE_IDX;
	static constexpr std::uint8_t const CONT_UNK_BIT = 1u << CONT_UNK_IDX;
	static constexpr std::uint8_t const CONT_OCC_BIT = 1u << CONT_OCC_IDX;

	std::uint8_t data;
};

template <std::size_t Dim, std::size_t BF, class T>
struct OccupancyInnerBlock {
	std::array<OccupancyInnerNode<T>, BF> nodes;

	[[nodiscard]] T occupancy(std::size_t index) const
	{
		assert(BF > index);
		if constexpr (std::is_same_v<T, OccupancyState>) {
			int v = static_cast<int>(nodes[index].data & OccupancyInnerNode<T>::OCC_BITS);
			return static_cast<OccupancyState>(v - 1);
		} else {
			return nodes[index].occupancy;
		}
	}

	[[nodiscard]] std::array<T, BF> occupancy() const
	{
		std::array<T, BF> occupancies;
		for (std::size_t i{}; BF > i; ++i) {
			occupancies[i] = occupancy(i);
		}
		return occupancies;
	}

	[[nodiscard]] bool occupancyContains(std::size_t index, OccupancyState state) const
	{
		assert(BF > index);
		auto const x = static_cast<std::uint8_t>(
		    static_cast<int>(OccupancyInnerNode<T>::CONT_UNK_BIT) + to_underlying(state));
		return !!(nodes[index].data & x);
	}

	[[nodiscard]] std::array<bool, BF> occupancyContains(OccupancyState state) const
	{
		std::array<bool, BF> contains;
		for (std::size_t i{}; BF > i; ++i) {
			contains[i] = occupancyContains(i, state);
		}
		return contains;
	}

	void occupancySet(std::size_t index, T value, bool contains_unknown, bool contains_free,
	                  bool contains_occupied)
	{
		assert(BF > index);

		using U = std::uint8_t;

		U const cont =
		    (static_cast<U>(contains_unknown) << OccupancyInnerNode<T>::CONT_UNK_IDX) |
		    (static_cast<U>(contains_free) << OccupancyInnerNode<T>::CONT_FRE_IDX) |
		    (static_cast<U>(contains_occupied) << OccupancyInnerNode<T>::CONT_OCC_IDX);

		if constexpr (std::is_same_v<T, OccupancyState>) {
			U const occ       = static_cast<U>(to_underlying(value) + 1);
			nodes[index].data = occ | cont;
		} else {
			nodes[index].data      = cont;
			nodes[index].occupancy = value;
		}
	}

	void occupancySet(T value, bool contains_unknown, bool contains_free,
	                  bool contains_occupied)
	{
		for (std::size_t i{}; BF > i; ++i) {
			occupancySet(i, value, contains_unknown, contains_free, contains_occupied);
		}
	}
};

template <class T>
bool operator==(OccupancyInnerNode<T> const& lhs, OccupancyInnerNode<T> const& rhs)
{
	if constexpr (std::is_same_v<T, OccupancyState>) {
		return lhs.data == rhs.data;
	} else {
		return lhs.occupancy == rhs.occupancy && lhs.contains == rhs.contains;
	}
}

template <class T>
bool operator!=(OccupancyInnerNode<T> const& lhs, OccupancyInnerNode<T> const& rhs)
{
	return !(lhs == rhs);
};

template <std::size_t Dim, std::size_t BF, class T>
bool operator==(OccupancyInnerBlock<Dim, BF, T> const& lhs,
                OccupancyInnerBlock<Dim, BF, T> const& rhs)
{
	return lhs.nodes == rhs.nodes;
}

template <std::size_t Dim, std::size_t BF, class T>
bool operator!=(OccupancyInnerBlock<Dim, BF, T> const& lhs,
                OccupancyInnerBlock<Dim, BF, T> const& rhs)
{
	return !(lhs == rhs);
};
}  // namespace ufo::detail

#endif  // UFO_MAP_OCCUPANCY_DETAIL_INNER_BLOCK_HPP