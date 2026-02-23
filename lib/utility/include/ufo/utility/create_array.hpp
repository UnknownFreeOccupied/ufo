/**
 * @author Daniel Duberg (danielduberg@gmail.com)
 * @see https://github.com/UnknownFreeOccupied/ufo
 * @version 1.0
 * @date 2026-02-22
 *
 * @copyright Copyright (c) 2020-2026, Daniel Duberg
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2020-2026, Daniel Duberg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
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
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_UTILITY_CREATE_ARRAY_HPP
#define UFO_UTILITY_CREATE_ARRAY_HPP

// STL
#include <array>
#include <utility>

namespace ufo
{
// Utility functions for creating std::array filled with a repeated value.
// Source: https://stackoverflow.com/a/57757301

namespace detail
{
/**
 * @brief Helper function to create a std::array filled with a repeated value.
 *
 * @details
 * Uses index_sequence to expand the value N times at compile time.
 *
 * @tparam T The type of the array elements
 * @tparam Is Index sequence for expansion
 * @param value The value to fill the array with
 * @return std::array<T, N> filled with value
 */
template <class T, std::size_t... Is>
[[nodiscard]] constexpr std::array<T, sizeof...(Is)> createArray(
    T value, std::index_sequence<Is...>)
{
	// cast Is to void to remove the warning: unused value
	// Each element is initialized to 'value' using parameter pack expansion
	return {{(static_cast<void>(Is), value)...}};
}
}  // namespace detail

/**
 * @brief Creates a std::array of size N, filled with the given value.
 *
 * @tparam N The size of the array
 * @tparam T The type of the array elements
 * @param value The value to fill the array with
 * @return std::array<T, N> filled with value
 *
 * @details
 * Example:
 * @code{.cpp}
 *   auto arr = ufo::createArray<5>(42); // arr = {42, 42, 42, 42, 42}
 * @endcode
 */
template <std::size_t N, class T>
[[nodiscard]] constexpr std::array<T, N> createArray(T const& value)
{
	// Calls the helper with an index sequence to expand the value N times
	return detail::createArray(value, std::make_index_sequence<N>());
}
}  // namespace ufo

#endif  // UFO_UTILITY_CREATE_ARRAY_HPP