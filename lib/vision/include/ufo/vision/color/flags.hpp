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

#ifndef UFO_VISION_COLOR_FLAGS_HPP
#define UFO_VISION_COLOR_FLAGS_HPP

// STL
#include <cstdint>
#include <format>
#include <ostream>
#include <string_view>
#include <utility>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

/**
 * @brief Bitmask controlling optional fields in a `Color` specialisation.
 * @details
 * Flags may be combined with `operator|`:
 * @code
 *   Color<ColorModel::Rgb, float, ColorFlags::Alpha | ColorFlags::Weight> c;
 * @endcode
 */
enum class ColorFlags : std::uint8_t {
	None   = 0,              ///< No optional fields.
	Alpha  = 1 << 0,         ///< Include an alpha channel.
	Weight = 1 << 1,         ///< Include an accumulation weight.
	All    = Alpha | Weight  ///< All optional fields.
};

/**
 * @brief Bitwise OR for combining `ColorFlags`.
 * @param [in] a The first `ColorFlags` value.
 * @param [in] b The second `ColorFlags` value.
 * @return The result of the bitwise OR operation.
 */
[[nodiscard]] constexpr ColorFlags operator|(ColorFlags a, ColorFlags b) noexcept
{
	return static_cast<ColorFlags>(std::to_underlying(a) | std::to_underlying(b));
}

/**
 * @brief Bitwise AND for testing `ColorFlags`.
 * @param [in] a The first `ColorFlags` value.
 * @param [in] b The second `ColorFlags` value.
 * @return The result of the bitwise AND operation.
 */
[[nodiscard]] constexpr ColorFlags operator&(ColorFlags a, ColorFlags b) noexcept
{
	return static_cast<ColorFlags>(std::to_underlying(a) & std::to_underlying(b));
}

/**
 * @brief Bitwise complement - inverts all valid `ColorFlags` bits.
 * @param [in] a The `ColorFlags` value to invert.
 * @return The result of the bitwise complement operation.
 */
[[nodiscard]] constexpr ColorFlags operator~(ColorFlags a) noexcept
{
	// Mask to the two defined bits so the result is always a valid combination.
	return static_cast<ColorFlags>(~std::to_underlying(a) &
	                               std::to_underlying(ColorFlags::All));
}

/**
 * @brief Returns `true` if the `Alpha` flag is set.
 */
[[nodiscard]] consteval bool alphaset(ColorFlags flags) noexcept
{
	return (flags & ColorFlags::Alpha) != ColorFlags::None;
}

/**
 * @brief Returns `true` if the `Weight` flag is set.
 */
[[nodiscard]] consteval bool weightset(ColorFlags flags) noexcept
{
	return (flags & ColorFlags::Weight) != ColorFlags::None;
}

/**
 * @brief Returns a human-readable string for a `ColorFlags` value.
 * @param [in] flags A valid combination of `ColorFlags` enumerators.
 * @return A null-terminated `std::string_view` with a stable lifetime.
 */
[[nodiscard]] constexpr std::string_view toString(ColorFlags flags) noexcept
{
	using enum ColorFlags;
	switch (flags) {
		case None: return "None";
		case Alpha: return "Alpha";
		case Weight: return "Weight";
		case (Alpha | Weight): return "Alpha|Weight";
	}
	std::unreachable();
}

/**
 * @brief Stream-insertion operator for `ColorFlags`.
 */
inline std::ostream& operator<<(std::ostream& os, ColorFlags flags)
{
	return os << toString(flags);
}

/**
 * @}
 */
}  // namespace ufo

template <>
struct std::formatter<ufo::ColorFlags> : std::formatter<std::string_view> {
	auto format(ufo::ColorFlags flags, std::format_context& ctx) const
	{
		return std::formatter<std::string_view>::format(ufo::toString(flags), ctx);
	}
};

#endif  // UFO_VISION_COLOR_FLAGS_HPP
