/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the
 * Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of
 * Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
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

#ifndef UFO_VISION_COLOR_AVERAGE_HPP
#define UFO_VISION_COLOR_AVERAGE_HPP

// UFO
#include <ufo/vision/color/arithmetic.hpp>
#include <ufo/vision/color/concepts.hpp>
#include <ufo/vision/color/convert.hpp>
#include <ufo/vision/color/type_traits.hpp>
#include <ufo/vision/color/weight.hpp>

// STL
#include <algorithm>
#include <concepts>
#include <functional>
#include <iterator>
#include <ranges>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

/**
 * @brief Computes the average of two colors.
 * @tparam C The color type.
 * @param [in] a The first color.
 * @param [in] b The second color.
 * @return The average of the two colors.
 */
template <Color C>
[[nodiscard]] constexpr C average(C a, C const& b)
{
	if constexpr (ColorWithWeight<C>) {
		return a + b;
	} else {
		return removeWeight(addWeight(a) + addWeight(b));
	}
}

/**
 * @brief Computes the average of a range of colors.
 * @tparam R The range type.
 * @param [in] r The range of colors.
 * @return The average of the range of colors.
 */
template <std::ranges::input_range R>
  requires Color<std::ranges::range_value_t<R>>
[[nodiscard]] constexpr std::ranges::range_value_t<R> average(R&& r)
{
	using C = std::ranges::range_value_t<R>;
	using D = typename color_traits<C>::template rebind<float, color_traits<C>::flags |
	                                                               ColorFlags::Weight>;

	if constexpr (std::same_as<C, D>) {
		return std::ranges::fold_left(r, D{}, std::plus{});
	} else if constexpr (!ColorWithWeight<C> && FloatingPointColor<C>) {
		auto sum = std::ranges::fold_left(
		    r | std::views::transform([](C c) { return addWeight(c); }), D{}, std::plus{});
		return removeWeight(sum);
	} else {
		// Normalise to weighted float, sum, then convert back to target type
		auto sum = std::ranges::fold_left(
		    r | std::views::transform([](C c) { return convert<D>(c); }), D{}, std::plus{});
		return convert<C>(sum);
	}
}

/**
 * @brief Computes the average of a range of colors.
 * @tparam InputIt The input iterator type.
 * @tparam Sentinel The sentinel type.
 * @param [in] first The beginning of the range.
 * @param [in] last The end of the range.
 * @return The average of the range of colors.
 */
template <std::input_iterator InputIt, std::sentinel_for<InputIt> Sentinel>
  requires Color<std::iter_value_t<InputIt>>
[[nodiscard]] constexpr std::iter_value_t<InputIt> average(InputIt first, Sentinel last)
{
	return average(std::ranges::subrange(std::move(first), std::move(last)));
}

/**
 * @brief Computes the average of an initializer list of colors.
 * @tparam C The color type.
 * @param [in] il The initializer list of colors.
 * @return The average of the initializer list of colors.
 */
template <Color C>
[[nodiscard]] constexpr C average(std::initializer_list<C> il)
{
	return average(std::ranges::subrange(il));
}

/**
 * @}
 */
}  // namespace ufo

#endif  // UFO_VISION_COLOR_AVERAGE_HPP