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
#ifndef UFO_MAP_OCCUPANCY_COLOR_VOID_REGION_DETAIL_INNER_BLOCK_HPP
#define UFO_MAP_OCCUPANCY_COLOR_VOID_REGION_DETAIL_INNER_BLOCK_HPP

// UFO
#include <ufo/map/occupancy/state.hpp>
#include <ufo/utility/bit_set.hpp>
#include <ufo/vision/color.hpp>

// STL
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <tuple>

namespace ufo::detail
{
template <bool Small>
struct OCVRInnerNode {
	using occupancy_type = OccupancyState;
	using color_type     = SmallLRGB;

	static constexpr std::uint8_t const VR_IDX       = 2u;
	static constexpr std::uint8_t const CONT_FRE_IDX = 3u;
	static constexpr std::uint8_t const CONT_UNK_IDX = 4u;
	static constexpr std::uint8_t const CONT_OCC_IDX = 5u;
	static constexpr std::uint8_t const CONT_VR_IDX  = 6u;
	static constexpr std::uint8_t const OCC_BITS     = 0b11u;
	static constexpr std::uint8_t const VR_BIT       = 1u << VR_IDX;
	static constexpr std::uint8_t const CONT_FRE_BIT = 1u << CONT_FRE_IDX;
	static constexpr std::uint8_t const CONT_UNK_BIT = 1u << CONT_UNK_IDX;
	static constexpr std::uint8_t const CONT_OCC_BIT = 1u << CONT_OCC_IDX;
	static constexpr std::uint8_t const CONT_VR_BIT  = 1u << CONT_VR_IDX;
	static constexpr std::uint8_t const CONT_OCC_BITS =
	    CONT_UNK_BIT | CONT_FRE_BIT | CONT_OCC_BIT;
	static constexpr std::uint8_t const ALL_OCC_BITS = OCC_BITS | CONT_OCC_BITS;
	static constexpr std::uint8_t const ALL_VR_BITS  = VR_BIT | CONT_VR_BIT;

	color_type   color;
	std::uint8_t data;  // Occupancy and void region
};

template <>
struct OCVRInnerNode<false> {
	using occupancy_type = float;
	using color_type     = SmallLabW;

	static constexpr std::uint8_t const VR_IDX       = 2u;
	static constexpr std::uint8_t const CONT_FRE_IDX = 3u;
	static constexpr std::uint8_t const CONT_UNK_IDX = 4u;
	static constexpr std::uint8_t const CONT_OCC_IDX = 5u;
	static constexpr std::uint8_t const CONT_VR_IDX  = 6u;
	static constexpr std::uint8_t const VR_BIT       = 1u << VR_IDX;
	static constexpr std::uint8_t const CONT_FRE_BIT = 1u << CONT_FRE_IDX;
	static constexpr std::uint8_t const CONT_UNK_BIT = 1u << CONT_UNK_IDX;
	static constexpr std::uint8_t const CONT_OCC_BIT = 1u << CONT_OCC_IDX;
	static constexpr std::uint8_t const CONT_VR_BIT  = 1u << CONT_VR_IDX;
	static constexpr std::uint8_t const CONT_OCC_BITS =
	    CONT_UNK_BIT | CONT_FRE_BIT | CONT_OCC_BIT;
	static constexpr std::uint8_t const ALL_OCC_BITS = CONT_OCC_BITS;
	static constexpr std::uint8_t const ALL_VR_BITS  = VR_BIT | CONT_VR_BIT;

	remove_weight_t<color_type> color;
	std::uint8_t                data;  // Occupancy indicators and void region
	weight_type_t<color_type>   color_weight;
	occupancy_type              occupancy;
};

static_assert(4 == sizeof(OCVRInnerNode<true>));
static_assert(12 == sizeof(OCVRInnerNode<false>));
static_assert(std::is_standard_layout_v<OCVRInnerNode<true>>);
static_assert(std::is_standard_layout_v<OCVRInnerNode<false>>);

template <std::size_t Dim, std::size_t BF, bool Small>
struct OCVRInnerBlock {
	using value_type     = OCVRInnerNode<Small>;
	using occupancy_type = typename value_type::occupancy_type;
	using color_type     = typename value_type::color_type;

	std::array<value_type, BF> nodes;

	/**************************************************************************************
	|                                                                                     |
	|                                      Occupancy                                      |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] occupancy_type occupancy(std::size_t index) const
	{
		assert(BF > index);
		if constexpr (Small) {
			int v = static_cast<int>(nodes[index].data & OCVRInnerNode<Small>::OCC_BITS);
			return static_cast<OccupancyState>(v - 1);
		} else {
			return nodes[index].occupancy;
		}
	}

	[[nodiscard]] std::array<occupancy_type, BF> occupancy() const
	{
		std::array<occupancy_type, BF> occupancies;
		for (std::size_t i{}; BF > i; ++i) {
			occupancies[i] = occupancy(i);
		}
		return occupancies;
	}

	[[nodiscard]] bool occupancyContains(std::size_t index, OccupancyState state) const
	{
		assert(BF > index);
		auto const x = static_cast<std::uint8_t>(
		    static_cast<int>(OCVRInnerNode<Small>::CONT_UNK_BIT) + to_underlying(state));
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

	void occupancySet(std::size_t index, occupancy_type value, bool contains_unknown,
	                  bool contains_free, bool contains_occupied)
	{
		assert(BF > index);

		using T = std::uint8_t;

		T const vr = nodes[index].data & OCVRInnerNode<Small>::ALL_VR_BITS;
		T const cont =
		    (static_cast<T>(contains_unknown) << OCVRInnerNode<Small>::CONT_UNK_IDX) |
		    (static_cast<T>(contains_free) << OCVRInnerNode<Small>::CONT_FRE_IDX) |
		    (static_cast<T>(contains_occupied) << OCVRInnerNode<Small>::CONT_OCC_IDX);

		if constexpr (Small) {
			T const occ       = static_cast<T>(to_underlying(value) + 1);
			nodes[index].data = occ | cont | vr;
		} else {
			nodes[index].data      = cont | vr;
			nodes[index].occupancy = value;
		}
	}

	void occupancySet(occupancy_type value, bool contains_unknown, bool contains_free,
	                  bool contains_occupied)
	{
		for (std::size_t i{}; BF > i; ++i) {
			occupancySet(i, value, contains_unknown, contains_free, contains_occupied);
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                        Color                                        |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] color_type color(std::size_t index) const
	{
		assert(BF > index);
		if constexpr (Small) {
			return nodes[index].color;
		} else {
			return addWeight(nodes[index].color, nodes[index].color_weight);
		}
	}

	[[nodiscard]] std::array<color_type, BF> color() const
	{
		std::array<color_type, BF> colors;
		for (std::size_t i{}; BF > i; ++i) {
			colors[i] = color(i);
		}
		return colors;
	}

	void colorSet(std::size_t index, color_type const& value)
	{
		assert(BF > index);
		if constexpr (Small) {
			nodes[index].color = value;
		} else {
			nodes[index].color        = removeWeight(value);
			nodes[index].color_weight = weight(value);
		}
	}

	void colorSet(color_type const& value)
	{
		for (std::size_t i{}; BF > i; ++i) {
			colorSet(i, value);
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Void region                                     |
	|                                                                                     |
	**************************************************************************************/

	[[nodiscard]] bool voidRegion(std::size_t index) const
	{
		assert(BF > index);
		return !!(nodes[index].data & OCVRInnerNode<Small>::VR_BIT);
	}

	[[nodiscard]] BitSet<BF> voidRegion() const
	{
		using T = typename BitSet<BF>::value_type;
		T data{};
		for (std::size_t i{}; BF > i; ++i) {
			data |= static_cast<T>(voidRegion(i)) << i;
		}
		return BitSet<BF>(data);
	}

	[[nodiscard]] bool voidRegionContains(std::size_t index) const
	{
		assert(BF > index);
		return !!(nodes[index].data & OCVRInnerNode<Small>::CONT_VR_BIT);
	}

	[[nodiscard]] BitSet<BF> voidRegionContains() const
	{
		using T = typename BitSet<BF>::value_type;
		T data{};
		for (std::size_t i{}; BF > i; ++i) {
			data |= static_cast<T>(voidRegionContains(i)) << i;
		}
		return BitSet<BF>(data);
	}

	void voidRegionSet(std::size_t index, bool value, bool contains)
	{
		assert(BF > index);

		using T = std::uint8_t;

		T const x         = nodes[index].data & OCVRInnerNode<Small>::ALL_OCC_BITS;
		T const y         = static_cast<T>(value) << OCVRInnerNode<Small>::VR_IDX;
		T const z         = static_cast<T>(contains) << OCVRInnerNode<Small>::CONT_VR_IDX;
		nodes[index].data = x | y | z;
	}

	void voidRegionSet(bool value, bool contains)
	{
		for (std::size_t i{}; BF > i; ++i) {
			voidRegionSet(i, value, contains);
		}
	}
};

template <bool Small>
bool operator==(OCVRInnerNode<Small> const& lhs, OCVRInnerNode<Small> const& rhs)
{
	auto tied = [](auto const& x) {
		if constexpr (Small) {
			return std::tie(x.color, x.data);
		} else {
			return std::tie(x.color, x.void_region, x.color_weight, x.occupancy);
		}
	};
	return tied(lhs) == tied(rhs);
}

template <bool Small>
bool operator!=(OCVRInnerNode<Small> const& lhs, OCVRInnerNode<Small> const& rhs)
{
	return !(lhs == rhs);
};

template <std::size_t Dim, std::size_t BF, bool Small>
bool operator==(OCVRInnerBlock<Dim, BF, Small> const& lhs,
                OCVRInnerBlock<Dim, BF, Small> const& rhs)
{
	return lhs.nodes == rhs.nodes;
}

template <std::size_t Dim, std::size_t BF, bool Small>
bool operator!=(OCVRInnerBlock<Dim, BF, Small> const& lhs,
                OCVRInnerBlock<Dim, BF, Small> const& rhs)
{
	return !(lhs == rhs);
};
}  // namespace ufo::detail

#endif  // UFO_MAP_OCCUPANCY_COLOR_VOID_REGION_DETAIL_INNER_BLOCK_HPP