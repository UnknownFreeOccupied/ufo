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

#ifndef UFO_VISION_COLOR_ALPHA_HPP
#define UFO_VISION_COLOR_ALPHA_HPP

// UFO
#include <ufo/vision/color/concepts.hpp>

// STL
#include <concepts>
#include <limits>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

namespace detail
{

/**
 * @brief Converts an alpha value from `From` to `To`.
 *
 * - Same type: identity.
 * - Unsigned integer -> float: normalise to [0, 1].
 * - Float -> unsigned integer: denormalise from [0, 1].
 * - Float -> float: direct cast.
 * - Unsigned integer -> unsigned integer: round-trip normalise/denormalise.
 */
template <typename To, typename From>
[[nodiscard]] constexpr To convertAlpha(From value) noexcept
{
	if constexpr (std::same_as<To, From>) {
		return value;
	} else if constexpr (std::floating_point<To> && std::unsigned_integral<From>) {
		return static_cast<To>(value) / static_cast<To>(std::numeric_limits<From>::max());
	} else if constexpr (std::unsigned_integral<To> && std::floating_point<From>) {
		return static_cast<To>(value * static_cast<From>(std::numeric_limits<To>::max()));
	} else if constexpr (std::floating_point<To> && std::floating_point<From>) {
		return static_cast<To>(value);
	} else if constexpr (std::unsigned_integral<To> && std::unsigned_integral<From>) {
		auto const normalized = static_cast<double>(value) /
		                        static_cast<double>(std::numeric_limits<From>::max());
		return static_cast<To>(normalized *
		                       static_cast<double>(std::numeric_limits<To>::max()));
	} else {
		static_assert(false, "unsupported alpha value type conversion");
	}
}

}  // namespace detail

/**
 * @brief Returns the un-weighted alpha of `color`.
 *
 * If `color` has no alpha channel the default alpha is returned (`1` for
 * floating-point, `std::numeric_limits<value_type>::max()` for integral types).
 * For weighted floating-point colors the stored field is divided by the weight.
 */
template <Color C>
[[nodiscard]] constexpr alpha_type_t<C> alpha(C color) noexcept
{
	if constexpr (!ColorWithAlpha<C>) {
		return init_alpha_v<C>;
	} else if constexpr (ColorWithWeight<C> && FloatingPointColor<C>) {
		return color.alpha / static_cast<alpha_type_t<C>>(color.weight);
	} else {
		return color.alpha;
	}
}

/**
 * @brief Returns the alpha of `color` converted to `Target`.
 *
 * If `color` has no alpha channel the default alpha for the equivalent color
 * with `Target` channels is returned.
 * Integer <-> float conversions normalise/denormalise through [0, 1].
 */
template <typename Target, Color C>
[[nodiscard]] constexpr Target alpha(C color) noexcept
{
	if constexpr (!ColorWithAlpha<C>) {
		using TargetColor = typename color_traits<C>::template rebind_value<Target>;
		return detail::convertAlpha<Target>(init_alpha_v<TargetColor>);
	} else {
		return detail::convertAlpha<Target>(alpha(color));
	}
}

/**
 * @brief Returns a copy of `color` with an alpha channel set to `a`.
 *
 * If `color` already carries an alpha channel it is returned unchanged and
 * `a` is ignored.  For weighted floating-point colors the stored alpha field
 * is pre-multiplied by the existing weight.
 */
template <Color C>
[[nodiscard]] constexpr add_alpha_t<C> addAlpha(
    C color, alpha_type_t<C> a = init_alpha_v<C>) noexcept
{
	if constexpr (ColorWithAlpha<C>) {
		return color;
	} else {
		add_alpha_t<C> r;

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
			static_assert(false, "unhandled ColorModel — update addAlpha");
		}

		if constexpr (ColorWithWeight<C> && FloatingPointColor<C>) {
			r.alpha = a * static_cast<alpha_type_t<C>>(color.weight);
		} else {
			r.alpha = a;
		}

		if constexpr (ColorWithWeight<C>) {
			r.weight = color.weight;
		}

		return r;
	}
}

/**
 * @brief Returns a copy of `color` with the alpha channel removed.
 *
 * If `color` has no alpha channel it is returned unchanged.
 */
template <Color C>
[[nodiscard]] constexpr remove_alpha_t<C> removeAlpha(C color) noexcept
{
	if constexpr (!ColorWithAlpha<C>) {
		return color;
	} else {
		remove_alpha_t<C> r;

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
			static_assert(false, "unhandled ColorModel — update removeAlpha");
		}

		if constexpr (ColorWithWeight<C>) {
			r.weight = color.weight;
		}

		return r;
	}
}

/**
 * @brief Blends `fg` over `bg` using standard Porter-Duff "over" compositing.
 *
 * Both colors must satisfy `ColorWithAlpha` and `FloatingPointColor`.
 * Returns a transparent default-initialised color if the result alpha is zero.
 */
template <Color C>
  requires ColorWithAlpha<C> && FloatingPointColor<C>
[[nodiscard]] constexpr C alphaBlend(C fg, C bg) noexcept
{
	using T = value_type_t<C>;

	T const fg_a  = alpha(fg);
	T const bg_a  = alpha(bg);
	T const inv   = T(1) - fg_a;
	T const res_a = fg_a + bg_a * inv;

	if (res_a <= T(0)) {
		return C{};
	}

	C r = fg;

	if constexpr (GrayColor<C>) {
		r.gray = (fg.gray * fg_a + bg.gray * bg_a * inv) / res_a;
	} else if constexpr (RgbFamilyColor<C>) {
		r.red   = (fg.red * fg_a + bg.red * bg_a * inv) / res_a;
		r.green = (fg.green * fg_a + bg.green * bg_a * inv) / res_a;
		r.blue  = (fg.blue * fg_a + bg.blue * bg_a * inv) / res_a;
	} else if constexpr (LabColor<C>) {
		r.lightness = (fg.lightness * fg_a + bg.lightness * bg_a * inv) / res_a;
		r.a         = (fg.a * fg_a + bg.a * bg_a * inv) / res_a;
		r.b         = (fg.b * fg_a + bg.b * bg_a * inv) / res_a;
	} else if constexpr (LchColor<C>) {
		r.lightness = (fg.lightness * fg_a + bg.lightness * bg_a * inv) / res_a;
		r.chroma    = (fg.chroma * fg_a + bg.chroma * bg_a * inv) / res_a;
		r.hue       = (fg.hue * fg_a + bg.hue * bg_a * inv) / res_a;
	} else {
		static_assert(false, "unhandled ColorModel — update alphaBlend");
	}

	if constexpr (ColorWithWeight<C>) {
		r.alpha = res_a * static_cast<T>(r.weight);
	} else {
		r.alpha = res_a;
	}

	return r;
}

/**
 * @brief Returns a copy of `color` with each channel multiplied by its alpha value.
 *
 * Requires `ColorWithAlpha` and `FloatingPointColor`.
 */
template <Color C>
  requires ColorWithAlpha<C> && FloatingPointColor<C>
[[nodiscard]] constexpr C premultiplyAlpha(C color) noexcept
{
	value_type_t<C> const a = alpha(color);
	C                     r = color;

	if constexpr (GrayColor<C>) {
		r.gray *= a;
	} else if constexpr (RgbFamilyColor<C>) {
		r.red *= a;
		r.green *= a;
		r.blue *= a;
	} else if constexpr (LabColor<C>) {
		r.lightness *= a;
		r.a *= a;
		r.b *= a;
	} else if constexpr (LchColor<C>) {
		r.lightness *= a;
		r.chroma *= a;
		r.hue *= a;
	} else {
		static_assert(false, "unhandled ColorModel — update premultiplyAlpha");
	}

	return r;
}

/**
 * @brief Returns a copy of `color` with each channel divided by its alpha value.
 *
 * Requires `ColorWithAlpha` and `FloatingPointColor`.
 * If the alpha value is zero or negative `color` is returned unchanged.
 */
template <Color C>
  requires ColorWithAlpha<C> && FloatingPointColor<C>
[[nodiscard]] constexpr C unpremultiplyAlpha(C color) noexcept
{
	value_type_t<C> const a = alpha(color);

	if (a <= value_type_t<C>(0)) {
		return color;
	}

	C r = color;

	if constexpr (GrayColor<C>) {
		r.gray /= a;
	} else if constexpr (RgbFamilyColor<C>) {
		r.red /= a;
		r.green /= a;
		r.blue /= a;
	} else if constexpr (LabColor<C>) {
		r.lightness /= a;
		r.a /= a;
		r.b /= a;
	} else if constexpr (LchColor<C>) {
		r.lightness /= a;
		r.chroma /= a;
		r.hue /= a;
	} else {
		static_assert(false, "unhandled ColorModel — update unpremultiplyAlpha");
	}

	return r;
}

/**
 * @}
 */
}  // namespace ufo

#endif  // UFO_VISION_COLOR_ALPHA_HPP
