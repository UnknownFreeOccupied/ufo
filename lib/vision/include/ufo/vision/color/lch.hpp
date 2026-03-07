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

#ifndef UFO_VISION_COLOR_LCH_HPP
#define UFO_VISION_COLOR_LCH_HPP

// UFO
#include <ufo/vision/color/flags.hpp>

// STL
#include <concepts>
#include <cstdint>
#include <format>
#include <ostream>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

/**
 * @brief OkLch color.
 * @tparam T Type of the color channels (default: `float`).
 * @tparam Flags Color flags (default: `ColorFlags::None`).
 * @details
 * `T` must satisfy `std::integral` or `std::floating_point`.
 * Cylindrical representation of Oklab (Björn Ottosson, 2020): lightness,
 * chroma (radius), and hue (angle in degrees).
 * Channel values are in `[0, 1]` for floating-point types and
 * `[TYPE_MIN, TYPE_MAX]` for integer types.
 */
template <class T = float, ColorFlags Flags = ColorFlags::None>
  requires(std::integral<T> || std::floating_point<T>)
struct Lch;

/**
 * @brief OkLch color with neither alpha nor weight.
 * @details
 * `T` must satisfy `std::integral` or `std::floating_point`.
 * Channel values are in `[0, 1]` for floating-point types and
 * `[TYPE_MIN, TYPE_MAX]` for integer types.
 */
template <class T>
  requires(std::integral<T> || std::floating_point<T>)
struct Lch<T, ColorFlags::None> {
	/**
	 * @brief Lightness channel.
	 * @details
	 * `[0, 1]` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T lightness;

	/**
	 * @brief Chroma (radius).
	 * @details
	 * `[0, 0.4]` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T chroma;

	/**
	 * @brief Hue angle in degrees.
	 * @details
	 * `[0, 360)` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T hue;

	/**
	 * @brief Returns true if `a` and `b` are equal.
	 * @param [in] a First color.
	 * @param [in] b Second color.
	 * @retval true if `a` and `b` are equal.
	 * @retval false if `a` and `b` are not equal.
	 */
	[[nodiscard]] friend constexpr bool operator==(Lch const& a,
	                                               Lch const& b) noexcept = default;
};

/**
 * @brief OkLch color with alpha, without weight.
 * @details
 * `T` must satisfy `std::integral` or `std::floating_point`.
 * Channel values are in `[0, 1]` for floating-point types and
 * `[TYPE_MIN, TYPE_MAX]` for integer types.
 */
template <class T>
  requires(std::integral<T> || std::floating_point<T>)
struct Lch<T, ColorFlags::Alpha> {
	/**
	 * @brief Lightness channel.
	 * @details
	 * `[0, 1]` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T lightness;

	/**
	 * @brief Chroma (radius).
	 * @details
	 * `[0, 0.4]` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T chroma;

	/**
	 * @brief Hue angle in degrees.
	 * @details
	 * `[0, 360)` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T hue;

	/**
	 * @brief Alpha channel.
	 * @details
	 * `[0, 1]` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T alpha;

	/**
	 * @brief Returns true if `a` and `b` are equal.
	 * @param [in] a First color.
	 * @param [in] b Second color.
	 * @retval true if `a` and `b` are equal.
	 * @retval false if `a` and `b` are not equal.
	 */
	[[nodiscard]] friend constexpr bool operator==(Lch const& a,
	                                               Lch const& b) noexcept = default;
};

/**
 * @brief OkLch color with weight, without alpha.
 * @details
 * `T` must satisfy `std::integral` or `std::floating_point`.
 * When `T` is a floating-point type the stored channels are pre-multiplied by
 * `weight`; use the `lightness()`, `chroma()`, and `hue()` free functions to
 * retrieve the un-weighted values.
 */
template <class T>
  requires(std::integral<T> || std::floating_point<T>)
struct Lch<T, ColorFlags::Weight> {
	/**
	 * @brief Lightness channel (possibly pre-multiplied by `weight`).
	 * @details
	 * `[0, 1] * weight` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T lightness;

	/**
	 * @brief Chroma (possibly pre-multiplied by `weight`).
	 * @details
	 * `[0, 0.4] * weight` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T chroma;

	/**
	 * @brief Hue angle (possibly pre-multiplied by `weight`).
	 * @details
	 * `[0, 360) * weight` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T hue;

	/**
	 * @brief Accumulation weight (positive).
	 */
	float weight;

	/**
	 * @brief Returns true if `a` and `b` are equal.
	 * @param [in] a First color.
	 * @param [in] b Second color.
	 * @retval true if `a` and `b` are equal.
	 * @retval false if `a` and `b` are not equal.
	 */
	[[nodiscard]] friend constexpr bool operator==(Lch const& a,
	                                               Lch const& b) noexcept = default;
};

/**
 * @brief OkLch color with both alpha and weight.
 * @details
 * `T` must satisfy `std::integral` or `std::floating_point`.
 * When `T` is a floating-point type the stored channels are pre-multiplied by
 * `weight`; use the `lightness()`, `chroma()`, and `hue()` free functions to
 * retrieve the un-weighted values.
 */
template <class T>
  requires(std::integral<T> || std::floating_point<T>)
struct Lch<T, ColorFlags::Alpha | ColorFlags::Weight> {
	/**
	 * @brief Lightness channel (possibly pre-multiplied by `weight`).
	 * @details
	 * `[0, 1] * weight` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T lightness;

	/**
	 * @brief Chroma (possibly pre-multiplied by `weight`).
	 * @details
	 * `[0, 0.4] * weight` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T chroma;

	/**
	 * @brief Hue angle (possibly pre-multiplied by `weight`).
	 * @details
	 * `[0, 360) * weight` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T hue;

	/**
	 * @brief Alpha channel.
	 * @details
	 * `[0, 1]` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T alpha;

	/**
	 * @brief Accumulation weight (positive).
	 */
	float weight;

	/**
	 * @brief Returns true if `a` and `b` are equal.
	 * @param [in] a First color.
	 * @param [in] b Second color.
	 * @retval true if `a` and `b` are equal.
	 * @retval false if `a` and `b` are not equal.
	 */
	[[nodiscard]] friend constexpr bool operator==(Lch const& a,
	                                               Lch const& b) noexcept = default;
};

/**
 * @brief Returns the un-weighted lightness value.
 * @details
 * For floating-point weighted colors the raw `lightness` field is divided by
 * `weight`; for all other combinations the raw field is returned as-is.
 */
template <class T, ColorFlags Flags>
[[nodiscard]] constexpr T lightness(Lch<T, Flags> color) noexcept
{
	if constexpr (weightset(Flags) && std::floating_point<T>) {
		return color.lightness / static_cast<T>(color.weight);
	} else {
		return color.lightness;
	}
}

/**
 * @brief Returns the un-weighted chroma value.
 * @details
 * For floating-point weighted colors the raw `chroma` field is divided by
 * `weight`; for all other combinations the raw field is returned as-is.
 */
template <class T, ColorFlags Flags>
[[nodiscard]] constexpr T chroma(Lch<T, Flags> color) noexcept
{
	if constexpr (weightset(Flags) && std::floating_point<T>) {
		return color.chroma / static_cast<T>(color.weight);
	} else {
		return color.chroma;
	}
}

/**
 * @brief Returns the un-weighted hue angle value.
 * @details
 * For floating-point weighted colors the raw `hue` field is divided by
 * `weight`; for all other combinations the raw field is returned as-is.
 */
template <class T, ColorFlags Flags>
[[nodiscard]] constexpr T hue(Lch<T, Flags> color) noexcept
{
	if constexpr (weightset(Flags) && std::floating_point<T>) {
		return color.hue / static_cast<T>(color.weight);
	} else {
		return color.hue;
	}
}

/**
 * @brief 8-bit OkLch color.
 */
using SmallLch = Lch<std::uint8_t>;

/**
 * @brief 8-bit OkLch color with alpha.
 */
using SmallLchA = Lch<std::uint8_t, ColorFlags::Alpha>;

/**
 * @brief 8-bit OkLch color with weight.
 */
using SmallLchW = Lch<std::uint8_t, ColorFlags::Weight>;

/**
 * @brief 8-bit OkLch color with alpha and weight.
 */
using SmallLchAW = Lch<std::uint8_t, ColorFlags::Alpha | ColorFlags::Weight>;

/**
 * @brief Floating-point OkLch color.
 */
using FineLch = Lch<float>;

/**
 * @brief Floating-point OkLch color with alpha.
 */
using FineLchA = Lch<float, ColorFlags::Alpha>;

/**
 * @brief Floating-point OkLch color with weight.
 */
using FineLchW = Lch<float, ColorFlags::Weight>;

/**
 * @brief Floating-point OkLch color with alpha and weight.
 */
using FineLchAW = Lch<float, ColorFlags::Alpha | ColorFlags::Weight>;

/**
 * @brief Writes a human-readable representation of `c` to `out`.
 * @param [out] out Output stream.
 * @param [in] c Color to write.
 * @return `out`.
 */
template <class T, ColorFlags Flags>
inline std::ostream& operator<<(std::ostream& out, Lch<T, Flags> const& c)
{
	out << "Lightness: " << c.lightness << " Chroma: " << c.chroma << " Hue: " << c.hue;
	if constexpr (alphaset(Flags)) {
		out << " Alpha: " << +c.alpha;
	}
	if constexpr (weightset(Flags)) {
		out << " Weight: " << c.weight;
	}
	return out;
}

/**
 * @}
 */
}  // namespace ufo

/**
 * @brief `std::format` / `std::formatter` specialization for `ufo::Lch<T, Flags>`.
 */
template <class T, ufo::ColorFlags Flags>
struct std::formatter<ufo::Lch<T, Flags>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::Lch<T, Flags> const& c, std::format_context& ctx) const
	{
		auto out = std::format_to(ctx.out(), "Lightness: {} Chroma: {} Hue: {}", c.lightness,
		                          c.chroma, c.hue);
		if constexpr (ufo::alphaset(Flags)) {
			out = std::format_to(out, " Alpha: {}", +c.alpha);
		}
		if constexpr (ufo::weightset(Flags)) {
			out = std::format_to(out, " Weight: {}", c.weight);
		}
		return out;
	}
};

#endif  // UFO_VISION_COLOR_LCH_HPP
