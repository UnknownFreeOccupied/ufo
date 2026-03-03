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

#ifndef UFO_VISION_COLOR_BLEND_HPP
#define UFO_VISION_COLOR_BLEND_HPP

// UFO
#include <ufo/vision/color/arithmetic.hpp>
#include <ufo/vision/color/concepts.hpp>
#include <ufo/vision/color/convert.hpp>
#include <ufo/vision/color/type_traits.hpp>

// STL
#include <algorithm>
#include <concepts>
#include <iterator>
#include <ranges>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

/**
 * @brief Blends two colors using a factor `t`.
 * @tparam C The color type.
 * @param [in] a The first color.
 * @param [in] b The second color.
 * @param [in] t The blending factor in `[0, 1]`.
 * @return The blended color.
 * @details
 * Performs linear interpolation: `a + t * (b - a)`.
 */
template <Color C>
[[nodiscard]] constexpr C blend(C a, C const& b, float t)
{
	using D = typename color_traits<C>::template rebind_value<float>;

	if constexpr (std::same_as<C, D>) {
		return a + t * (b - a);
	} else {
		return convert<C>(blend(convert<D>(a), convert<D>(b), t));
	}
}

/**
 * @brief Blends multiple colors using a range of weights.
 * @tparam R The color range type.
 * @tparam W The weight range type.
 * @param [in] r The range of colors.
 * @param [in] w The range of weights.
 * @return The resulting blended color.
 * @details
 * Computes the weighted sum of the colors in `r` using weights in `w`.
 */
template <std::ranges::input_range R, std::ranges::input_range W>
  requires Color<std::ranges::range_value_t<R>> &&
           std::floating_point<std::ranges::range_value_t<W>>
[[nodiscard]] constexpr std::ranges::range_value_t<R> blend(R&& r, W&& w)
{
	using C = std::ranges::range_value_t<R>;
	using D = typename color_traits<C>::template rebind_value<float>;

	if constexpr (std::same_as<C, D>) {
		return std::ranges::fold_left(
		    std::views::zip(r, w) | std::views::transform([](auto&& z) {
			    auto [c, weight] = z;
			    return c * weight;
		    }),
		    D{}, std::plus{});
	} else {
		return convert<C>(std::ranges::fold_left(
		    std::views::zip(r, w) | std::views::transform([](auto&& z) {
			    auto [c, weight] = z;
			    return convert<D>(c) * weight;
		    }),
		    D{}, std::plus{}));
	}
}

/**
 * @brief Blends multiple colors using a range of weights.
 * @tparam InputIt1 The color iterator type.
 * @tparam Sentinel  The color sentinel type.
 * @tparam InputIt2 The weight iterator type.
 * @param [in] first The beginning of the color range.
 * @param [in] last The end of the color range.
 * @param [in] first_weight The beginning of the weight range.
 * @return The resulting blended color.
 */
template <std::input_iterator InputIt1, std::sentinel_for<InputIt1> Sentinel,
          std::input_iterator InputIt2>
  requires Color<std::iter_value_t<InputIt1>> &&
           std::floating_point<std::iter_value_t<InputIt2>>
[[nodiscard]] constexpr std::iter_value_t<InputIt1> blend(InputIt1 first, Sentinel last,
                                                          InputIt2 first_weight)
{
	auto r = std::ranges::subrange(std::move(first), std::move(last));
	auto w = std::views::repeat(0.0f) |
	         std::views::transform([first_weight, i = 0](auto) mutable {
		         return *std::next(first_weight, i++);
	         });
	return blend(r, w);
}

/**
 * @brief Blends multiple colors using an initializer list of weights.
 * @tparam R The color range type.
 * @tparam T The weight type.
 * @param [in] r The range of colors.
 * @param [in] w The initializer list of weights.
 * @return The resulting blended color.
 */
template <std::ranges::input_range R, std::floating_point T>
  requires Color<std::ranges::range_value_t<R>>
[[nodiscard]] constexpr std::ranges::range_value_t<R> blend(R&&                      r,
                                                            std::initializer_list<T> w)
{
	return blend(std::forward<R>(r), std::ranges::subrange(w));
}

/**
 * @brief Blends multiple colors using an initializer list of colors and weights.
 * @tparam C The color type.
 * @tparam T The weight type.
 * @param [in] r The initializer list of colors.
 * @param [in] w The initializer list of weights.
 * @return The resulting blended color.
 */
template <Color C, std::floating_point T>
[[nodiscard]] constexpr C blend(std::initializer_list<C> r, std::initializer_list<T> w)
{
	return blend(std::ranges::subrange(r), std::ranges::subrange(w));
}

/**
 * @}
 */
}  // namespace ufo

#endif  // UFO_VISION_COLOR_BLEND_HPP