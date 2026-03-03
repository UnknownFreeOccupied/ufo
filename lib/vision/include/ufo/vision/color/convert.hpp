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

#ifndef UFO_VISION_COLOR_CONVERT_HPP
#define UFO_VISION_COLOR_CONVERT_HPP

// UFO
#include <ufo/vision/color/concepts.hpp>
#include <ufo/vision/color/detail/convert.hpp>

// STL
#include <concepts>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

/**
 * @brief Convert a color from one type to another.
 * @tparam To   The destination color type (must satisfy the Color concept).
 * @tparam From The source color type (must satisfy the Color concept).
 * @param [in] color The color to convert.
 * @return The converted color of type To.
 * @details
 * Performs a full color model and channel type conversion, including handling of
 * alpha and weight fields.
 */
template <Color To, Color From>
[[nodiscard]] constexpr To convert(From const& color)
{
	return detail::convertColor<To>(color);
}

/**
 * @brief Convert a color from one type to another.
 * @tparam T    The destination value type (must satisfy the floating_point or
 *              integral concept).
 * @tparam From The source color type (must satisfy the Color concept).
 * @param [in] color The color to convert.
 * @return The converted color of type To.
 * @details
 * Converts the value type of the color to T, keeping the same color model.
 */
template <typename T, Color From>
  requires(std::floating_point<T> || std::integral<T>)
[[nodiscard]] constexpr auto convert(From const& color)
{
	return convert<typename color_traits<From>::template rebind_value<T>>(color);
}

/**
 * @brief Convert a color from one type to another.
 * @tparam F    The destination color flags (must satisfy the ColorFlags concept).
 * @tparam From The source color type (must satisfy the Color concept).
 * @param [in] color The color to convert.
 * @return The converted color of type To.
 * @details
 * Converts the color alpha/weight flags, keeping the same color model and value
 * type.
 */
template <ColorFlags F, Color From>
[[nodiscard]] constexpr auto convert(From const& color)
{
	return convert<typename color_traits<From>::template rebind_flags<F>>(color);
}

/**
 * @brief Convert a color from one type to another.
 * @tparam T    The destination value type (must satisfy the floating_point or
 *              integral concept).
 * @tparam F    The destination color flags (must satisfy the ColorFlags concept).
 * @tparam From The source color type (must satisfy the Color concept).
 * @param [in] color The color to convert.
 * @return The converted color of type To.
 * @details
 * Converts the value type and alpha/weight flags, keeping the same color model.
 */
template <typename T, ColorFlags F, Color From>
  requires(std::floating_point<T> || std::integral<T>)
[[nodiscard]] constexpr auto convert(From const& color)
{
	return convert<typename color_traits<From>::template rebind<T, F>>(color);
}

/**
 * @brief Convert a color from one type to another and assign to an output variable.
 * @tparam From The source color type (must satisfy the Color concept).
 * @tparam To   The destination color type (must satisfy the Color concept).
 * @param [in] in The color to convert.
 * @param [out] out The output variable to store the converted color.
 * @details
 * This is a convenience overload that assigns the result of the conversion to the output
 * parameter.
 */
template <Color From, Color To>
constexpr void convert(From const& in, To& out)
{
	out = convert<To>(in);
}

/**
 * @}
 */
}  // namespace ufo

#endif  // UFO_VISION_COLOR_CONVERT_HPP