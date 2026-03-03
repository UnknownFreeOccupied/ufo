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

#ifndef UFO_VISION_COLOR_CONCEPTS_HPP
#define UFO_VISION_COLOR_CONCEPTS_HPP

// UFO
#include <ufo/vision/color/flags.hpp>
#include <ufo/vision/color/type_traits.hpp>

// STL
#include <concepts>
#include <cstdint>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

/**************************************************************************************
|                                                                                     |
|                                  Core Color Concept                                 |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Identifies any UFO color instantiation (`Gray`, `Lab`, `Lch`, `Lrgb`, `Rgb`).
 */
template <typename T>
concept Color = is_color_v<T>;

/**************************************************************************************
|                                                                                     |
|                               Value Type Concepts                                   |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Colors with a floating-point channel type.
 */
template <typename T>
concept FloatingPointColor = Color<T> && std::floating_point<value_type_t<T>>;

/**
 * @brief Colors with an integral channel type.
 */
template <typename T>
concept IntegralColor = Color<T> && std::integral<value_type_t<T>>;

/**************************************************************************************
|                                                                                     |
|                               Flag-Based Concepts                                   |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Colors that include an alpha channel.
 */
template <typename T>
concept ColorWithAlpha = Color<T> && has_alpha_v<T>;

/**
 * @brief Colors that include a weight channel.
 */
template <typename T>
concept ColorWithWeight = Color<T> && has_weight_v<T>;

/**
 * @brief Colors with both alpha and weight channels.
 */
template <typename T>
concept ColorWithAlphaAndWeight = ColorWithAlpha<T> && ColorWithWeight<T>;

/**
 * @brief Colors without optional channels (no alpha, no weight).
 */
template <typename T>
concept BasicColor = Color<T> && (color_traits<T>::flags == ColorFlags::None);

/**************************************************************************************
|                                                                                     |
|                              Color Model Concepts                                   |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Grayscale colors (`Gray<T, Flags>`).
 */
template <typename T>
concept GrayColor = Color<T> && (color_traits<T>::model == ColorModel::Gray);

/**
 * @brief Gamma-encoded sRgb colors (`Rgb<T, Flags>`).
 */
template <typename T>
concept RgbColor = Color<T> && (color_traits<T>::model == ColorModel::Rgb);

/**
 * @brief Linear-light Rgb colors (`Lrgb<T, Flags>`).
 */
template <typename T>
concept LinearRgbColor = Color<T> && (color_traits<T>::model == ColorModel::Lrgb);

/**
 * @brief Oklab perceptual colors (`Lab<T, Flags>`).
 */
template <typename T>
concept LabColor = Color<T> && (color_traits<T>::model == ColorModel::Lab);

/**
 * @brief OkLch cylindrical colors (`Lch<T, Flags>`).
 */
template <typename T>
concept LchColor = Color<T> && (color_traits<T>::model == ColorModel::Lch);

/**************************************************************************************
|                                                                                     |
|                           Grouped Color Model Concepts                              |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Rgb-family colors (gamma-encoded or linear: `Rgb` or `Lrgb`).
 */
template <typename T>
concept RgbFamilyColor = RgbColor<T> || LinearRgbColor<T>;

/**
 * @brief Perceptual color models (`Lab` or `Lch`).
 */
template <typename T>
concept PerceptualColor = LabColor<T> || LchColor<T>;

/**
 * @brief Color models suitable for display (`Rgb`, `Lrgb`, or `Gray`).
 */
template <typename T>
concept DisplayColor = RgbFamilyColor<T> || GrayColor<T>;

/**
 * @brief Color models with a hue component (`Lch`).
 */
template <typename T>
concept HueBasedColor = LchColor<T>;

/**
 * @brief Color models with a lightness component (`Lab` or `Lch`).
 */
template <typename T>
concept LightnessBasedColor = LabColor<T> || LchColor<T>;

/**************************************************************************************
|                                                                                     |
|                           Specialized Use Case Concepts                             |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Colors that support weighted accumulation (floating-point with weight).
 */
template <typename T>
concept AccumulableColor = ColorWithWeight<T> && FloatingPointColor<T>;

/**
 * @brief Colors suitable for image processing (integral, or floating-point display
 * colors).
 */
template <typename T>
concept ImageColor =
    Color<T> && (IntegralColor<T> || (FloatingPointColor<T> && DisplayColor<T>));

/**
 * @brief Colors suitable for high dynamic range operations (floating-point display
 * colors).
 */
template <typename T>
concept HDRColor = FloatingPointColor<T> && DisplayColor<T>;

/**
 * @brief Colors with transparency support.
 */
template <typename T>
concept TransparentColor = ColorWithAlpha<T>;

/**
 * @brief Colors optimized for compact storage (integral, at most 16 bits per channel).
 */
template <typename T>
concept CompactColor = IntegralColor<T> && (sizeof(value_type_t<T>) <= 2);

/**************************************************************************************
|                                                                                     |
|                              Multi-Type Concepts                                    |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Two colors with the same underlying channel type.
 */
template <typename T1, typename T2>
concept SameValueType =
    Color<T1> && Color<T2> && std::same_as<value_type_t<T1>, value_type_t<T2>>;

/**
 * @brief Two colors with the same color model.
 */
template <typename T1, typename T2>
concept SameColorModel =
    Color<T1> && Color<T2> && (color_traits<T1>::model == color_traits<T2>::model);

/**
 * @brief Two colors with the same flags.
 */
template <typename T1, typename T2>
concept SameColorFlags =
    Color<T1> && Color<T2> && (color_traits<T1>::flags == color_traits<T2>::flags);

/**
 * @brief Two colors that are equivalent (same model, channel type, and flags).
 */
template <typename T1, typename T2>
concept EquivalentColors =
    SameColorModel<T1, T2> && SameValueType<T1, T2> && SameColorFlags<T1, T2>;

/**
 * @brief Colors with compatible channel types for conversion.
 */
template <typename From, typename To>
concept ConvertibleColor =
    Color<From> && Color<To> && std::convertible_to<value_type_t<From>, value_type_t<To>>;

/**
 * @brief Colors that can be blended together (same model and mutually convertible types).
 */
template <typename T1, typename T2>
concept BlendableColors =
    SameColorModel<T1, T2> && ConvertibleColor<T1, T2> && ConvertibleColor<T2, T1>;

/**************************************************************************************
|                                                                                     |
|                         Value Type Extraction Concepts                              |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Colors with a specific channel type.
 */
template <typename T, typename ValueType>
concept ColorWithValueType = Color<T> && std::same_as<value_type_t<T>, ValueType>;

/**
 * @brief Colors with an 8-bit channel type.
 */
template <typename T>
concept Color8Bit = ColorWithValueType<T, std::uint8_t>;

/**
 * @brief Colors with a 16-bit channel type.
 */
template <typename T>
concept Color16Bit = ColorWithValueType<T, std::uint16_t>;

/**
 * @brief Colors with a 32-bit float channel type.
 */
template <typename T>
concept ColorFloat = ColorWithValueType<T, float>;

/**
 * @brief Colors with a 64-bit double channel type.
 */
template <typename T>
concept ColorDouble = ColorWithValueType<T, double>;

/**************************************************************************************
|                                                                                     |
|                         Predicate Concepts for Algorithms                           |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Colors that support gamma correction (Rgb family).
 */
template <typename T>
concept GammaCorrectable = RgbFamilyColor<T>;

/**
 * @brief Colors that support color space conversion between display color models.
 */
template <typename From, typename To>
concept ColorSpaceConvertible =
    Color<From> && Color<To> && DisplayColor<From> && DisplayColor<To>;

/**
 * @brief Colors that operate in a perceptually uniform space.
 */
template <typename T>
concept PerceptuallyUniform = PerceptualColor<T>;

/**
 * @brief Convenience alias: any UFO color instantiation.
 */
template <typename T>
concept IsColor = Color<T>;

/**
 * @}
 */
}  // namespace ufo

#endif  // UFO_VISION_COLOR_CONCEPTS_HPP
