/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (danielduberg@gmail.com)
 * @see https://github.com/UnknownFreeOccupied/ufo
 * @version 1.0.0
 * @date 2026-02-21
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

#ifndef UFO_MATH_MATH_HPP
#define UFO_MATH_MATH_HPP

// STL
#include <algorithm>
#include <cassert>
#include <cmath>
#include <concepts>
#include <limits>
#include <numbers>
#include <ranges>

namespace ufo
{
/**
 * @brief Returns the sign of a value.
 * @tparam T Numeric type (e.g., int, float).
 * @param [in] val The value to check.
 * @retval -1 `val < 0`
 * @retval 0 `val == 0`
 * @retval 1 `val > 0`
 */
template <class T>
[[nodiscard]] constexpr int sign(T val) noexcept
{
	if constexpr (std::is_unsigned_v<T>) {
		return T(0) < val;
	} else {
		return (T(0) < val) - (val < T(0));
	}
}

/**
 * @brief Converts degrees to radians.
 * @tparam T Floating point type (e.g., float, double).
 * @param [in] deg The angle in degrees.
 * @return The angle in radians.
 */
template <std::floating_point T>
[[nodiscard]] constexpr T radians(T deg) noexcept
{
	return deg * std::numbers::pi_v<T> / T(180);
}

/**
 * @brief Converts radians to degrees.
 * @tparam T Floating point type (e.g., float, double).
 * @param [in] rad The angle in radians.
 * @return The angle in degrees.
 */
template <std::floating_point T>
[[nodiscard]] constexpr T degrees(T rad) noexcept
{
	return rad * T(180) / std::numbers::pi_v<T>;
}

/**
 * @brief Computes integer power of a base.
 * @tparam T Numeric type (e.g., int, float).
 * @param [in] base The base value.
 * @param [in] exp The exponent (can be negative).
 * @return The result of raising the base to the exponent.
 */
template <class T>
  requires std::is_arithmetic_v<T>
[[nodiscard]] constexpr T ipow(T base, int exp) noexcept
{
	T result = std::ranges::fold_left(std::views::repeat(base, std::abs(exp)), T(1),
	                                  std::multiplies<>());
	return 0 <= exp ? result : T(1) / result;
}

/**
 * @brief Converts probability to logit value.
 * @tparam T Floating point type (e.g., float, double).
 * @param [in] probability The probability value in the range [0, 1].
 * @return The corresponding logit value.
 *
 * @details
 * Handles edge cases for `probability == 0` or `probability == 1`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr T probabilityToLogit(T probability)
{
	assert(T(0) <= probability && T(1) >= probability);
	if (T(0) >= probability) {
		return -std::numeric_limits<T>::infinity();
	}
	if (T(1) <= probability) {
		return std::numeric_limits<T>::infinity();
	}
	return std::log(probability / (T(1) - probability));
}

/**
 * @brief Converts logit value to probability.
 * @tparam T Floating point type (e.g., float, double).
 * @param [in] logit The logit value.
 * @return The corresponding probability value in the range [0, 1].
 */
template <std::floating_point T>
[[nodiscard]] constexpr T logitToProbability(T logit)
{
	return T(1) / (T(1) + std::exp(-logit));
}
}  // namespace ufo

#endif  // UFO_MATH_MATH_HPP