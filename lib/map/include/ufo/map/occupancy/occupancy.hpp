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

#ifndef UFO_MAP_OCCUPANCY_OCCUPANCY_HPP
#define UFO_MAP_OCCUPANCY_OCCUPANCY_HPP

// UFO
#include <ufo/map/occupancy/state.hpp>

// STL
#include <cstdint>
#include <type_traits>

namespace ufo
{
template <class T = float>
struct Occupancy;

template <>
struct Occupancy<float> {
	using value_type = float;

	value_type logit;
	// First 4 bits indicate contains,
	// 		bit #0 = unknown, #1 = free, #2 = occupied, and #4 = void region.
	std::uint8_t data;
};

template <>
struct Occupancy<std::uint8_t> {
	using value_type = std::uint8_t;

	value_type logit;
	// First 4 bits indicate contains,
	// 		bit #0 = unknown, #1 = free, #2 = occupied, and #4 = void region.
	std::uint8_t data;
};

template <>
struct Occupancy<OccupancyState> {
	using value_type = OccupancyState;

	// First 4 bits indicate contains,
	// 		Bit #0 = unknown, #1 = free, #2 = occupied, and #3 = void region.
	// Next 2 bits indicate state
	std::uint8_t data;
};

namespace detail
{
template <class T>
[[nodiscard]] constexpr auto occupancy(Occupancy<T> occupancy)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr auto logit(Occupancy<T> occupancy)
{
	if constexpr (std::is_same_v<bool, T>) {
		return (0b01110000u & occupancy.data) >> 4u;
	} else {
		return occupancy.logit;
	}
}

template <class T>
[[nodiscard]] constexpr bool containsUnknown(Occupancy<T> occupancy)
{
	return static_cast<bool>(0b00000001u & occupancy.data);
}

template <class T>
[[nodiscard]] constexpr bool containsFree(Occupancy<T> occupancy)
{
	return static_cast<bool>(0b00000010u & occupancy.data);
}

template <class T>
[[nodiscard]] constexpr bool containsOccupied(Occupancy<T> occupancy)
{
	return static_cast<bool>(0b00000100u & occupancy.data);
}

template <class T>
[[nodiscard]] constexpr bool containsVoidRegion(Occupancy<T> occupancy)
{
	return static_cast<bool>(0b00001000u & occupancy.data);
}

template <class T>
[[nodiscard]] constexpr bool unknown(Occupancy<T> occupancy, T min_threshold,
                                     T max_threshold)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool free(Occupancy<T> occupancy, T min_threshold,
                                  T max_threshold)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] bool occupied(Occupancy<T> occupancy, T threshold)
{
	// TODO: Implement
}
};  // namespace detail
}  // namespace ufo

#endif  // UFO_MAP_OCCUPANCY_OCCUPANCY_HPP