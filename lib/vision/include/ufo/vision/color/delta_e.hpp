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

#ifndef UFO_VISION_COLOR_DELTA_E_HPP
#define UFO_VISION_COLOR_DELTA_E_HPP

// UFO
#include <ufo/utility/type_traits.hpp>
#include <ufo/vision/color/concepts.hpp>
#include <ufo/vision/color/convert.hpp>
#include <ufo/vision/color/space.hpp>
#include <ufo/vision/color/type_traits.hpp>

// STL
#include <cassert>
#include <cmath>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

enum class ColorDeltaE {
	EUCLIDEAN,
	// CIE76,
	// CMC,
	// CIE94,
	// CIEDE2000
	OK,
};

/**
 * @brief Computes the squared Euclidean distance between two colors.
 * @tparam C The color type.
 * @param [in] color The first color.
 * @param [in] sample The second color.
 * @return The squared Euclidean distance.
 * @details
 * The distance is computed in the color space of `C` using floating-point
 * precision.
 */
template <Color C>
[[nodiscard]] constexpr float deltaEEuclideanSquared(C const& color, C const& sample)
{
	using F = typename color_traits<C>::template rebind<float, ColorFlags::None>;

	auto const c = convert<F>(color);
	auto const s = convert<F>(sample);

	if constexpr (GrayColor<C>) {
		float const dg = c.gray - s.gray;
		return dg * dg;
	} else if constexpr (RgbFamilyColor<C>) {
		float const dr = c.red - s.red;
		float const dg = c.green - s.green;
		float const db = c.blue - s.blue;
		return dr * dr + dg * dg + db * db;
	} else if constexpr (LabColor<C>) {
		float const dl = c.lightness - s.lightness;
		float const da = c.a - s.a;
		float const db = c.b - s.b;
		return dl * dl + da * da + db * db;
	} else if constexpr (LchColor<C>) {
		float const dl = c.lightness - s.lightness;
		float const dc = c.chroma - s.chroma;
		float const dh = c.hue - s.hue;
		return dl * dl + dc * dc + dh * dh;
	} else {
		static_assert(dependent_false_v<C>, "Unknown color model");
	}
}

/**
 * @brief Computes the Euclidean distance between two colors.
 * @tparam C The color type.
 * @param [in] color The first color.
 * @param [in] sample The second color.
 * @return The Euclidean distance.
 */
template <Color C>
[[nodiscard]] constexpr float deltaEEuclidean(C const& color, C const& sample)
{
	return std::sqrt(deltaEEuclideanSquared(color, sample));
}

/**
 * @brief Computes the squared Ok delta E distance between two colors.
 * @tparam C The color type.
 * @param [in] color The first color.
 * @param [in] sample The second color.
 * @param [in] ab_scale The scale factor for the chromaticity channels (default: 2.0).
 * @return The squared Ok delta E distance.
 */
template <Color C>
[[nodiscard]] constexpr float deltaEOkSquared(C const& color, C const& sample,
                                              float const ab_scale = 2.0f)
{
	auto const c = convert<FineLab>(color);
	auto const s = convert<FineLab>(sample);

	float const dl = c.lightness - s.lightness;
	float const da = ab_scale * (c.a - s.a);
	float const db = ab_scale * (c.b - s.b);
	return dl * dl + da * da + db * db;
}

/**
 * @brief Computes the Ok delta E distance between two colors.
 * @tparam C The color type.
 * @param [in] color The first color.
 * @param [in] sample The second color.
 * @param [in] ab_scale The scale factor for the chromaticity channels (default: 2.0).
 * @return The Ok delta E distance.
 */
template <Color C>
[[nodiscard]] constexpr float deltaEOk(C const& color, C const& sample,
                                       float const ab_scale = 2.0f)
{
	return std::sqrt(deltaEOkSquared(color, sample, ab_scale));
}

/**
 * @brief Computes the squared distance between two colors using a specified method.
 * @tparam C The color type.
 * @param [in] color The first color.
 * @param [in] sample The second color.
 * @param [in] method The delta E method to use (default: EUCLIDEAN).
 * @param [in] space The color space to perform the calculation in (default: NATIVE).
 * @return The squared distance.
 */
template <Color C>
[[nodiscard]] constexpr float deltaESquared(C const& color, C const& sample,
                                            ColorDeltaE method = ColorDeltaE::EUCLIDEAN,
                                            ColorSpace  space  = ColorSpace::NATIVE)
{
	if (ColorDeltaE::OK == method || ColorSpace::Oklab2 == space) {
		return deltaEOkSquared(color, sample);
	}

	switch (space) {
		case ColorSpace::NATIVE: return deltaEEuclideanSquared(color, sample);
		case ColorSpace::Rgb:
			return deltaEEuclideanSquared(convert<FineRgb>(color), convert<FineRgb>(sample));
		case ColorSpace::Oklab:
			return deltaEEuclideanSquared(convert<FineLab>(color), convert<FineLab>(sample));
		case ColorSpace::Oklch:
			return deltaEEuclideanSquared(convert<FineLch>(color), convert<FineLch>(sample));
		default: break;
	}

	assert(false && "Not implemented.");
	return 0.0f;
}

/**
 * @brief Computes the distance between two colors using a specified method.
 * @tparam C The color type.
 * @param [in] color The first color.
 * @param [in] sample The second color.
 * @param [in] method The delta E method to use (default: EUCLIDEAN).
 * @param [in] space The color space to perform the calculation in (default: NATIVE).
 * @return The distance.
 */
template <Color C>
[[nodiscard]] constexpr float deltaE(C const& color, C const& sample,
                                     ColorDeltaE method = ColorDeltaE::EUCLIDEAN,
                                     ColorSpace  space  = ColorSpace::NATIVE)
{
	return std::sqrt(deltaESquared(color, sample, method, space));
}

/**
 * @}
 */
}  // namespace ufo

#endif  // UFO_VISION_COLOR_DELTA_E_HPP