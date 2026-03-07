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

#ifndef UFO_VISION_COLOR_RGB_HPP
#define UFO_VISION_COLOR_RGB_HPP

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
 * @brief Gamma-encoded sRGB color.
 * @tparam T Type of the color channels (default: `float`).
 * @tparam Flags Color flags (default: `ColorFlags::None`).
 * @details
 * `T` must satisfy `std::integral` or `std::floating_point`.
 * Channel values are in `[0, 1]` for floating-point types and
 * `[TYPE_MIN, TYPE_MAX]` for integer types.
 */
template <class T = float, ColorFlags Flags = ColorFlags::None>
  requires(std::integral<T> || std::floating_point<T>)
struct Rgb;

/**
 * @brief Gamma-encoded sRGB color with neither alpha nor weight.
 * @tparam T Type of the color channels (default: `float`).
 * @details
 * `T` must satisfy `std::integral` or `std::floating_point`.
 * Channel values are in `[0, 1]` for floating-point types and
 * `[TYPE_MIN, TYPE_MAX]` for integer types.
 */
template <class T>
  requires(std::integral<T> || std::floating_point<T>)
struct Rgb<T, ColorFlags::None> {
	/**
	 * @brief Red channel.
	 * @details
	 * `[0, 1]` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T red;

	/**
	 * @brief Green channel.
	 * @details
	 * `[0, 1]` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T green;

	/**
	 * @brief Blue channel.
	 * @details
	 * `[0, 1]` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T blue;

	/**
	 * @brief Returns true if `a` and `b` are equal.
	 * @param [in] a First color.
	 * @param [in] b Second color.
	 * @retval true if `a` and `b` are equal.
	 * @retval false if `a` and `b` are not equal.
	 */
	[[nodiscard]] friend constexpr bool operator==(Rgb const& a,
	                                               Rgb const& b) noexcept = default;
};

/**
 * @brief Gamma-encoded sRGB color with alpha, without weight.
 * @details
 * `T` must satisfy `std::integral` or `std::floating_point`.
 * Channel values are in `[0, 1]` for floating-point types and
 * `[TYPE_MIN, TYPE_MAX]` for integer types.
 */
template <class T>
  requires(std::integral<T> || std::floating_point<T>)
struct Rgb<T, ColorFlags::Alpha> {
	/**
	 * @brief Red channel.
	 * @details
	 * `[0, 1]` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T red;

	/**
	 * @brief Green channel.
	 * @details
	 * `[0, 1]` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T green;

	/**
	 * @brief Blue channel.
	 * @details
	 * `[0, 1]` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T blue;

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
	[[nodiscard]] friend constexpr bool operator==(Rgb const& a,
	                                               Rgb const& b) noexcept = default;
};

/**
 * @brief Gamma-encoded sRGB color with weight, without alpha.
 * @details
 * `T` must satisfy `std::integral` or `std::floating_point`.
 * When `T` is a floating-point type the stored channels are pre-multiplied by
 * `weight`; use the `red()`, `green()`, and `blue()` free functions to retrieve
 * the un-weighted values.
 */
template <class T>
  requires(std::integral<T> || std::floating_point<T>)
struct Rgb<T, ColorFlags::Weight> {
	/**
	 * @brief Red channel (possibly pre-multiplied by `weight`).
	 * @details
	 * `[0, 1] * weight` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T red;

	/**
	 * @brief Green channel (possibly pre-multiplied by `weight`).
	 * @details
	 * `[0, 1] * weight` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T green;

	/**
	 * @brief Blue channel (possibly pre-multiplied by `weight`).
	 * @details
	 * `[0, 1] * weight` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T blue;

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
	[[nodiscard]] friend constexpr bool operator==(Rgb const& a,
	                                               Rgb const& b) noexcept = default;
};

/**
 * @brief Gamma-encoded sRGB color with both alpha and weight.
 * @details
 * `T` must satisfy `std::integral` or `std::floating_point`.
 * When `T` is a floating-point type the stored channels are pre-multiplied by
 * `weight`; use the `red()`, `green()`, and `blue()` free functions to retrieve
 * the un-weighted values.
 */
template <class T>
  requires(std::integral<T> || std::floating_point<T>)
struct Rgb<T, ColorFlags::Alpha | ColorFlags::Weight> {
	/**
	 * @brief Red channel (possibly pre-multiplied by `weight`).
	 * @details
	 * `[0, 1] * weight` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T red;

	/**
	 * @brief Green channel (possibly pre-multiplied by `weight`).
	 * @details
	 * `[0, 1] * weight` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T green;

	/**
	 * @brief Blue channel (possibly pre-multiplied by `weight`).
	 * @details
	 * `[0, 1] * weight` for floating-point types.
	 * `[TYPE_MIN, TYPE_MAX]` for integer types.
	 */
	T blue;

	/**
	 * @brief Alpha channel (possibly pre-multiplied by `weight`).
	 * @details
	 * `[0, 1] * weight` for floating-point types.
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
	[[nodiscard]] friend constexpr bool operator==(Rgb const& a,
	                                               Rgb const& b) noexcept = default;
};

/**
 * @brief Returns the un-weighted red channel value.
 * @details
 * For floating-point weighted colors the raw `red` field is divided by
 * `weight`; for all other combinations the raw field is returned as-is.
 */
template <class T, ColorFlags Flags>
[[nodiscard]] constexpr T red(Rgb<T, Flags> color) noexcept
{
	if constexpr (weightset(Flags) && std::floating_point<T>) {
		return color.red / static_cast<T>(color.weight);
	} else {
		return color.red;
	}
}

/**
 * @brief Returns the un-weighted green channel value.
 * @details
 * For floating-point weighted colors the raw `green` field is divided by
 * `weight`; for all other combinations the raw field is returned as-is.
 */
template <class T, ColorFlags Flags>
[[nodiscard]] constexpr T green(Rgb<T, Flags> color) noexcept
{
	if constexpr (weightset(Flags) && std::floating_point<T>) {
		return color.green / static_cast<T>(color.weight);
	} else {
		return color.green;
	}
}

/**
 * @brief Returns the un-weighted blue channel value.
 * @details
 * For floating-point weighted colors the raw `blue` field is divided by
 * `weight`; for all other combinations the raw field is returned as-is.
 */
template <class T, ColorFlags Flags>
[[nodiscard]] constexpr T blue(Rgb<T, Flags> color) noexcept
{
	if constexpr (weightset(Flags) && std::floating_point<T>) {
		return color.blue / static_cast<T>(color.weight);
	} else {
		return color.blue;
	}
}

/**
 * @brief 8-bit sRGB color.
 */
using SmallRgb = Rgb<std::uint8_t>;

/**
 * @brief 8-bit sRGB color with alpha.
 */
using SmallRgbA = Rgb<std::uint8_t, ColorFlags::Alpha>;

/**
 * @brief 8-bit sRGB color with weight.
 */
using SmallRgbW = Rgb<std::uint8_t, ColorFlags::Weight>;

/**
 * @brief 8-bit sRGB color with alpha and weight.
 */
using SmallRgbAW = Rgb<std::uint8_t, ColorFlags::Alpha | ColorFlags::Weight>;

/**
 * @brief Floating-point sRGB color.
 */
using FineRgb = Rgb<float>;

/**
 * @brief Floating-point sRGB color with alpha.
 */
using FineRgbA = Rgb<float, ColorFlags::Alpha>;

/**
 * @brief Floating-point sRGB color with weight.
 */
using FineRgbW = Rgb<float, ColorFlags::Weight>;

/**
 * @brief Floating-point sRGB color with alpha and weight.
 */
using FineRgbAW = Rgb<float, ColorFlags::Alpha | ColorFlags::Weight>;

/**
 * @brief Writes a human-readable representation of `c` to `out`.
 * @param [out] out Output stream.
 * @param [in] c Color to write.
 * @return `out`.
 */
template <class T, ColorFlags Flags>
inline std::ostream& operator<<(std::ostream& out, Rgb<T, Flags> const& c)
{
	out << "Red: " << +c.red << " Green: " << +c.green << " Blue: " << +c.blue;
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
 * @brief `std::format` / `std::formatter` specialization for `ufo::Rgb<T, Flags>`.
 */
template <class T, ufo::ColorFlags Flags>
struct std::formatter<ufo::Rgb<T, Flags>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::Rgb<T, Flags> const& c, std::format_context& ctx) const
	{
		auto out = std::format_to(ctx.out(), "Red: {} Green: {} Blue: {}", +c.red, +c.green,
		                          +c.blue);
		if constexpr (ufo::alphaset(Flags)) {
			out = std::format_to(out, " Alpha: {}", +c.alpha);
		}
		if constexpr (ufo::weightset(Flags)) {
			out = std::format_to(out, " Weight: {}", c.weight);
		}
		return out;
	}
};

#endif  // UFO_VISION_COLOR_RGB_HPP
