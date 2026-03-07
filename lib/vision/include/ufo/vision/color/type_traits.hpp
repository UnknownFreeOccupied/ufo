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

#ifndef UFO_VISION_COLOR_TYPE_TRAITS_HPP
#define UFO_VISION_COLOR_TYPE_TRAITS_HPP

// UFO
#include <ufo/vision/color/gray.hpp>
#include <ufo/vision/color/lab.hpp>
#include <ufo/vision/color/lch.hpp>
#include <ufo/vision/color/lrgb.hpp>
#include <ufo/vision/color/rgb.hpp>

// STL
#include <concepts>
#include <limits>
#include <type_traits>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

/**************************************************************************************
|                                                                                     |
|                                    ColorModel                                       |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Tag identifying the color model of a color type.
 */
enum class ColorModel { Gray, Lab, Lch, Lrgb, Rgb };

/**************************************************************************************
|                                                                                     |
|                                   color_traits                                      |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Primary template - intentionally undefined.
 * @details
 * Only the per-color-type specialisations below are valid.
 * Provides:
 *   - `value_type`      - channel scalar type
 *   - `weight_type`     - accumulation weight scalar type (always `float`)
 *   - `flags`           - `ColorFlags` bitmask for this instantiation
 *   - `model`           - `ColorModel` tag identifying the color space
 *   - `rebind_flags<F>` - the same color type with flags replaced by `F`
 *   - `rebind_value<U>` - the same color type with the channel type replaced by `U`
 */
template <class C>
struct color_traits;

template <class T, ColorFlags Flags>
struct color_traits<Gray<T, Flags>> {
	using value_type  = T;
	using weight_type = float;

	static constexpr ColorFlags flags = Flags;
	static constexpr ColorModel model = ColorModel::Gray;

	template <class U, ColorFlags F>
	using rebind = Gray<U, F>;
	template <ColorFlags F>
	using rebind_flags = Gray<T, F>;
	template <class U>
	using rebind_value = Gray<U, Flags>;
};

template <class T, ColorFlags Flags>
struct color_traits<Lab<T, Flags>> {
	using value_type  = T;
	using weight_type = float;

	static constexpr ColorFlags flags = Flags;
	static constexpr ColorModel model = ColorModel::Lab;

	template <class U, ColorFlags F>
	using rebind = Lab<U, F>;
	template <ColorFlags F>
	using rebind_flags = Lab<T, F>;
	template <class U>
	using rebind_value = Lab<U, Flags>;
};

template <class T, ColorFlags Flags>
struct color_traits<Lch<T, Flags>> {
	using value_type  = T;
	using weight_type = float;

	static constexpr ColorFlags flags = Flags;
	static constexpr ColorModel model = ColorModel::Lch;

	template <class U, ColorFlags F>
	using rebind = Lch<U, F>;
	template <ColorFlags F>
	using rebind_flags = Lch<T, F>;
	template <class U>
	using rebind_value = Lch<U, Flags>;
};

template <class T, ColorFlags Flags>
struct color_traits<Lrgb<T, Flags>> {
	using value_type  = T;
	using weight_type = float;

	static constexpr ColorFlags flags = Flags;
	static constexpr ColorModel model = ColorModel::Lrgb;

	template <class U, ColorFlags F>
	using rebind = Lrgb<U, F>;
	template <ColorFlags F>
	using rebind_flags = Lrgb<T, F>;
	template <class U>
	using rebind_value = Lrgb<U, Flags>;
};

template <class T, ColorFlags Flags>
struct color_traits<Rgb<T, Flags>> {
	using value_type  = T;
	using weight_type = float;

	static constexpr ColorFlags flags = Flags;
	static constexpr ColorModel model = ColorModel::Rgb;

	template <class U, ColorFlags F>
	using rebind = Rgb<U, F>;
	template <ColorFlags F>
	using rebind_flags = Rgb<T, F>;
	template <class U>
	using rebind_value = Rgb<U, Flags>;
};

/**************************************************************************************
|                                                                                     |
|                                      is_color                                       |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Detects whether `T` is a color type.
 */
template <class T>
struct is_color : std::false_type {
};

template <class T, ColorFlags F>
struct is_color<Gray<T, F>> : std::true_type {
};

template <class T, ColorFlags F>
struct is_color<Lab<T, F>> : std::true_type {
};

template <class T, ColorFlags F>
struct is_color<Lch<T, F>> : std::true_type {
};

template <class T, ColorFlags F>
struct is_color<Lrgb<T, F>> : std::true_type {
};

template <class T, ColorFlags F>
struct is_color<Rgb<T, F>> : std::true_type {
};

template <class T>
constexpr inline bool is_color_v = is_color<T>::value;

/**************************************************************************************
|                                                                                     |
|                                   contains_color                                    |
|                                                                                     |
**************************************************************************************/

/**
 * @brief True if any type in `Ts…` is a color type.
 */
template <class... Ts>
struct contains_color : std::disjunction<is_color<Ts>...> {
};

template <class... Ts>
constexpr inline bool contains_color_v = contains_color<Ts...>::value;

/**************************************************************************************
|                                                                                     |
|                                    first_color                                      |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Yields the first color type found in `Ts…`.
 */
template <class T, class... Ts>
struct first_color : first_color<Ts...> {
};

template <class T, class... Ts>
  requires(is_color_v<T>)
struct first_color<T, Ts...> {
	using type = T;
};

template <class... Ts>
using first_color_t = typename first_color<Ts...>::type;

/**************************************************************************************
|                                                                                     |
|                                    Value type                                       |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Extracts the channel scalar type of a color.
 */
template <class T>
using value_type_t = typename color_traits<T>::value_type;

/**************************************************************************************
|                                                                                     |
|                                       Alpha                                         |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Extracts the alpha scalar type of a color.
 */
template <class T>
using alpha_type_t = typename color_traits<T>::value_type;

/**
 * @brief True if `T` includes an alpha channel.
 */
template <class T>
struct has_alpha : std::bool_constant<alphaset(color_traits<T>::flags)> {
};

template <class T>
constexpr inline bool has_alpha_v = has_alpha<T>::value;

/**
 * @brief Default-initialised alpha value for `T`.
 * @details
 * `1` for floating-point types, `std::numeric_limits<value_type>::max()` for
 * integral types.
 */
template <class T>
struct init_alpha {
	using type = alpha_type_t<T>;

	static constexpr type value =
	    std::floating_point<type> ? type(1) : std::numeric_limits<type>::max();
};

template <class T>
constexpr inline auto init_alpha_v = init_alpha<T>::value;

/**
 * @brief Adds `ColorFlags::Alpha` to a color type.
 */
template <class T>
using add_alpha_t =
    typename color_traits<T>::template rebind_flags<color_traits<T>::flags |
                                                    ColorFlags::Alpha>;

/**
 * @brief Removes `ColorFlags::Alpha` from a color type.
 */
template <class T>
using remove_alpha_t =
    typename color_traits<T>::template rebind_flags<color_traits<T>::flags &
                                                    ~ColorFlags::Alpha>;

/**************************************************************************************
|                                                                                     |
|                                      Weight                                         |
|                                                                                     |
**************************************************************************************/

/**
 * @brief True if `T` includes a weight field.
 */
template <class T>
struct has_weight : std::bool_constant<weightset(color_traits<T>::flags)> {
};

template <class T>
constexpr inline bool has_weight_v = has_weight<T>::value;

/**
 * @brief Extracts the weight scalar type of a color.
 */
template <class T>
using weight_type_t = typename color_traits<T>::weight_type;

/**
 * @brief Adds `ColorFlags::Weight` to a color type.
 */
template <class T>
using add_weight_t =
    typename color_traits<T>::template rebind_flags<color_traits<T>::flags |
                                                    ColorFlags::Weight>;

/**
 * @brief Removes `ColorFlags::Weight` from a color type.
 */
template <class T>
using remove_weight_t =
    typename color_traits<T>::template rebind_flags<color_traits<T>::flags &
                                                    ~ColorFlags::Weight>;

/**
 * @}
 */
}  // namespace ufo

#endif  // UFO_VISION_COLOR_TYPE_TRAITS_HPP
