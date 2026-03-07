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

#ifndef UFO_VISION_COLOR_WEIGHT_HPP
#define UFO_VISION_COLOR_WEIGHT_HPP

// UFO
#include <ufo/vision/color/concepts.hpp>

// STL
#include <concepts>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

/**
 * @brief Returns the weight of `color`, or `1` if `color` has no weight field.
 * @param [in] color The color to get the weight of.
 * @return The weight of `color`, or `1` if `color` has no weight field.
 */
template <Color C>
[[nodiscard]] constexpr weight_type_t<C> weight(C color) noexcept
{
	if constexpr (ColorWithWeight<C>) {
		return color.weight;
	} else {
		return weight_type_t<C>(1);
	}
}

/**
 * @brief Returns a copy of `color` with a weight field set to `w`.
 * @param [in] color The color to add a weight to.
 * @param [in] w The weight to add to `color`.
 * @return A copy of `color` with a weight field set to `w`.
 * @details
 * If `color` already carries a weight field it is returned unchanged and `w` is
 * ignored. For floating-point channel types the stored channels are pre-multiplied
 * by `w`.
 */
template <Color C>
[[nodiscard]] constexpr add_weight_t<C> addWeight(C color, float w = 1.0f) noexcept
{
	if constexpr (ColorWithWeight<C>) {
		return color;
	} else {
		using T = value_type_t<C>;

		if constexpr (std::floating_point<T>) {
			color *= w;
		}

		add_weight_t<C> r;

		if constexpr (GrayColor<C>) {
			r.gray = color.gray;
		} else if constexpr (RgbFamilyColor<C>) {
			r.red   = color.red;
			r.green = color.green;
			r.blue  = color.blue;
		} else if constexpr (LabColor<C>) {
			r.lightness = color.lightness;
			r.a         = color.a;
			r.b         = color.b;
		} else if constexpr (LchColor<C>) {
			r.lightness = color.lightness;
			r.chroma    = color.chroma;
			r.hue       = color.hue;
		} else {
			static_assert(false, "unhandled ColorModel — update addWeight");
		}

		if constexpr (ColorWithAlpha<C>) {
			r.alpha = color.alpha;
		}

		r.weight = w;
		return r;
	}
}

/**
 * @brief Returns a copy of `color` with the weight field removed.
 * @param [in] color The color to remove the weight from.
 * @return A copy of `color` with the weight field removed.
 * @details
 * If `color` has no weight field it is returned unchanged.
 * For floating-point channel types the stored channels are divided by the weight,
 * yielding un-premultiplied values; a zero weight returns a default-initialised color.
 */
template <Color C>
[[nodiscard]] constexpr remove_weight_t<C> removeWeight(C color) noexcept
{
	if constexpr (!ColorWithWeight<C>) {
		return color;
	} else {
		if (weight_type_t<C>(0) == color.weight) {
			return remove_weight_t<C>{};
		}

		remove_weight_t<C> r;

		if constexpr (GrayColor<C>) {
			r.gray = color.gray;
		} else if constexpr (RgbFamilyColor<C>) {
			r.red   = color.red;
			r.green = color.green;
			r.blue  = color.blue;
		} else if constexpr (LabColor<C>) {
			r.lightness = color.lightness;
			r.a         = color.a;
			r.b         = color.b;
		} else if constexpr (LchColor<C>) {
			r.lightness = color.lightness;
			r.chroma    = color.chroma;
			r.hue       = color.hue;
		} else {
			static_assert(false, "unhandled ColorModel — update removeWeight");
		}

		if constexpr (ColorWithAlpha<C>) {
			r.alpha = color.alpha;
		}

		using T = value_type_t<C>;
		if constexpr (std::floating_point<T>) {
			r /= color.weight;
		}

		return r;
	}
}

/**
 * @}
 */
}  // namespace ufo

#endif  // UFO_VISION_COLOR_WEIGHT_HPP
