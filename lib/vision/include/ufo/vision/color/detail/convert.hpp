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

#ifndef UFO_VISION_COLOR_DETAIL_CONVERT_HPP
#define UFO_VISION_COLOR_DETAIL_CONVERT_HPP

// UFO
#include <ufo/vision/color/alpha.hpp>
#include <ufo/vision/color/weight.hpp>

// STL
#include <cmath>
#include <concepts>
#include <limits>

namespace ufo::detail
{

/**************************************************************************************
|                                                                                     |
|                                Forward declarations                                 |
|                                                                                     |
**************************************************************************************/

template <FloatingPointColor In>
  requires BasicColor<In>
[[nodiscard]] constexpr Gray<float> toGray(In in) noexcept;

template <FloatingPointColor In>
  requires BasicColor<In>
[[nodiscard]] constexpr Rgb<float> toRgb(In in) noexcept;

template <FloatingPointColor In>
  requires BasicColor<In>
[[nodiscard]] constexpr Lrgb<float> toLrgb(In in) noexcept;

template <FloatingPointColor In>
  requires BasicColor<In>
[[nodiscard]] constexpr Lab<float> toLab(In in) noexcept;

template <FloatingPointColor In>
  requires BasicColor<In>
[[nodiscard]] constexpr Lch<float> toLch(In in) noexcept;

/**************************************************************************************
|                                                                                     |
|                                 Type conversion                                     |
|                                                                                     |
**************************************************************************************/

/**
 * Unified type conversion for color models.
 */
template <class U, BasicColor In>
[[nodiscard]] constexpr auto convertType(In in) noexcept ->
    typename color_traits<In>::template rebind_value<U>
{
	using T = value_type_t<In>;
	if constexpr (std::same_as<U, T>) {
		return in;
	} else if constexpr (GrayColor<In>) {
		if constexpr (std::floating_point<U> && std::unsigned_integral<T>) {
			constexpr U m = U(1) / static_cast<U>(std::numeric_limits<T>::max());
			return {m * static_cast<U>(in.gray)};
		} else if constexpr (std::unsigned_integral<U> && std::floating_point<T>) {
			constexpr T m = static_cast<T>(std::numeric_limits<U>::max());
			return {static_cast<U>(m * in.gray)};
		} else if constexpr (std::floating_point<U> && std::floating_point<T>) {
			return {static_cast<U>(in.gray)};
		} else {
			return convertType<U>(convertType<float>(in));
		}
	} else if constexpr (RgbFamilyColor<In>) {
		if constexpr (std::floating_point<U> && std::unsigned_integral<T>) {
			constexpr U m = U(1) / static_cast<U>(std::numeric_limits<T>::max());
			return {m * static_cast<U>(in.red), m * static_cast<U>(in.green),
			        m * static_cast<U>(in.blue)};
		} else if constexpr (std::unsigned_integral<U> && std::floating_point<T>) {
			constexpr T m = static_cast<T>(std::numeric_limits<U>::max());
			return {static_cast<U>(m * in.red), static_cast<U>(m * in.green),
			        static_cast<U>(m * in.blue)};
		} else if constexpr (std::floating_point<U> && std::floating_point<T>) {
			return {static_cast<U>(in.red), static_cast<U>(in.green), static_cast<U>(in.blue)};
		} else {
			return convertType<U>(convertType<float>(in));
		}
	} else if constexpr (LabColor<In>) {
		if constexpr (std::floating_point<U> && std::unsigned_integral<T>) {
			// lightness [0, MAX] -> [0, 1]; a, b [0, MAX] -> [-0.4, 0.4]
			constexpr U m = U(1) / static_cast<U>(std::numeric_limits<T>::max());
			return {m * static_cast<U>(in.lightness),
			        U(0.8) * m * static_cast<U>(in.a) - U(0.4),
			        U(0.8) * m * static_cast<U>(in.b) - U(0.4)};
		} else if constexpr (std::unsigned_integral<U> && std::floating_point<T>) {
			// lightness [0, 1] -> [0, MAX]; a, b [-0.4, 0.4] -> [0, MAX]
			constexpr T m = static_cast<T>(std::numeric_limits<U>::max());
			return {static_cast<U>(m * in.lightness),
			        static_cast<U>(m * ((in.a + T(0.4)) / T(0.8))),
			        static_cast<U>(m * ((in.b + T(0.4)) / T(0.8)))};
		} else if constexpr (std::floating_point<U> && std::floating_point<T>) {
			return {static_cast<U>(in.lightness), static_cast<U>(in.a), static_cast<U>(in.b)};
		} else {
			return convertType<U>(convertType<float>(in));
		}
	} else if constexpr (LchColor<In>) {
		if constexpr (std::floating_point<U> && std::unsigned_integral<T>) {
			// lightness [0, MAX] -> [0, 1]; chroma [0, MAX] -> [0, 0.4]; hue [0, MAX] -> [0,
			// 360)
			constexpr U m = U(1) / static_cast<U>(std::numeric_limits<T>::max());
			return {m * static_cast<U>(in.lightness), U(0.4) * m * static_cast<U>(in.chroma),
			        U(360) * m * static_cast<U>(in.hue)};
		} else if constexpr (std::unsigned_integral<U> && std::floating_point<T>) {
			// lightness [0, 1] -> [0, MAX]; chroma [0, 0.4] -> [0, MAX]; hue [0, 360) ->
			// [0, MAX]
			constexpr T m = static_cast<T>(std::numeric_limits<U>::max());
			return {static_cast<U>(m * in.lightness), static_cast<U>(m * (in.chroma / T(0.4))),
			        static_cast<U>(m * (in.hue / T(360)))};
		} else if constexpr (std::floating_point<U> && std::floating_point<T>) {
			return {static_cast<U>(in.lightness), static_cast<U>(in.chroma),
			        static_cast<U>(in.hue)};
		} else {
			return convertType<U>(convertType<float>(in));
		}
	} else {
		static_assert(false, "unsupported type conversion for Color");
	}
}

/**************************************************************************************
|                                                                                     |
|                        Model conversion: -> Gray (float)                            |
|                                                                                     |
**************************************************************************************/

template <FloatingPointColor In>
  requires BasicColor<In>
[[nodiscard]] constexpr Gray<float> toGray(In in) noexcept
{
	if constexpr (GrayColor<In>) {
		return in;
	} else if constexpr (LabColor<In>) {
		return {in.lightness};
	} else {
		return toGray(toLab(in));
	}
}

/**************************************************************************************
|                                                                                     |
|                        Model conversion: -> Rgb (float)                             |
|                                                                                     |
**************************************************************************************/

template <FloatingPointColor In>
  requires BasicColor<In>
[[nodiscard]] constexpr Rgb<float> toRgb(In in) noexcept
{
	if constexpr (RgbColor<In>) {
		return in;
	} else if constexpr (LinearRgbColor<In>) {
		// Source: https://bottosson.github.io/posts/colorwrong/#what-can-we-do%3F
		auto f = [](float x) {
			return .0031308f >= x ? 12.92f * x
			                      : 1.055f * std::pow(x, .4166666666666667f) - .055f;
		};
		return {f(in.red), f(in.green), f(in.blue)};
	} else {
		return toRgb(toLrgb(in));
	}
}

/**************************************************************************************
|                                                                                     |
|                        Model conversion: -> Lrgb (float)                            |
|                                                                                     |
**************************************************************************************/

template <FloatingPointColor In>
  requires BasicColor<In>
[[nodiscard]] constexpr Lrgb<float> toLrgb(In in) noexcept
{
	if constexpr (LinearRgbColor<In>) {
		return in;
	} else if constexpr (RgbColor<In>) {
		// Source: https://bottosson.github.io/posts/colorwrong/#what-can-we-do%3F
		auto f = [](float x) {
			return .04045f < x ? std::pow((x + .055f) / 1.055f, 2.4f) : x / 12.92f;
		};
		return {f(in.red), f(in.green), f(in.blue)};
	} else if constexpr (LabColor<In>) {
		// Source:
		// https://bottosson.github.io/posts/oklab/#converting-from-linear-srgb-to-oklab

		float l_ = in.lightness + 0.3963377774f * in.a + 0.2158037573f * in.b;
		float m_ = in.lightness - 0.1055613458f * in.a - 0.0638541728f * in.b;
		float s_ = in.lightness - 0.0894841775f * in.a - 1.2914855480f * in.b;

		float l = l_ * l_ * l_;
		float m = m_ * m_ * m_;
		float s = s_ * s_ * s_;

		return {
		    +4.0767416621f * l - 3.3077115913f * m + 0.2309699292f * s,
		    -1.2684380046f * l + 2.6097574011f * m - 0.3413193965f * s,
		    -0.0041960863f * l - 0.7034186147f * m + 1.7076147010f * s,
		};
	} else {
		return toLrgb(toLab(in));
	}
}

/**************************************************************************************
|                                                                                     |
|                        Model conversion: -> Lab (float)                             |
|                                                                                     |
**************************************************************************************/

template <FloatingPointColor In>
  requires BasicColor<In>
[[nodiscard]] constexpr Lab<float> toLab(In in) noexcept
{
	if constexpr (LabColor<In>) {
		return in;
	} else if constexpr (GrayColor<In>) {
		return {in.gray, 0.0f, 0.0f};
	} else if constexpr (RgbColor<In>) {
		return toLab(toLrgb(in));
	} else if constexpr (LinearRgbColor<In>) {
		// Source:
		// https://bottosson.github.io/posts/oklab/#converting-from-linear-srgb-to-oklab

		float l = 0.4122214708f * in.red + 0.5363325363f * in.green + 0.0514459929f * in.blue;
		float m = 0.2119034982f * in.red + 0.6806995451f * in.green + 0.1073969566f * in.blue;
		float s = 0.0883024619f * in.red + 0.2817188376f * in.green + 0.6299787005f * in.blue;

		float l_ = std::cbrt(l);
		float m_ = std::cbrt(m);
		float s_ = std::cbrt(s);

		return {
		    0.2104542553f * l_ + 0.7936177850f * m_ - 0.0040720468f * s_,
		    1.9779984951f * l_ - 2.4285922050f * m_ + 0.4505937099f * s_,
		    0.0259040371f * l_ + 0.7827717662f * m_ - 0.8086757660f * s_,
		};
	} else if constexpr (LchColor<In>) {
		// Source: https://bottosson.github.io/posts/oklab/#the-oklab-color-space
		return {in.lightness, in.chroma * std::cos(in.hue), in.chroma * std::sin(in.hue)};
	} else {
		static_assert(false, "Unknown color model in detail::toLab");
	}
}

/**************************************************************************************
|                                                                                     |
|                        Model conversion: -> Lch (float)                             |
|                                                                                     |
**************************************************************************************/

template <FloatingPointColor In>
  requires BasicColor<In>
[[nodiscard]] constexpr Lch<float> toLch(In in) noexcept
{
	if constexpr (LchColor<In>) {
		return in;
	} else if constexpr (LabColor<In>) {
		// Source: https://bottosson.github.io/posts/oklab/#the-oklab-color-space
		return {in.lightness, std::sqrt(in.a * in.a + in.b * in.b), std::atan2(in.b, in.a)};
	} else {
		return toLch(toLab(in));
	}
}

/**************************************************************************************
|                                                                                     |
|                                   Main entry point                                  |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Converts `in` to `Out`.
 * @tparam Out Target color model.
 * @tparam In Source color model.
 * @param [in] in Color to convert.
 * @return `in` converted to `Out`.
 * @details
 * Same model: strips flags, converts the channel type directly, reattaches flags.
 * Different model: strips flags, converts to float precision, converts the model,
 * converts the channel type, reattaches flags.
 */
template <Color Out, Color In>
[[nodiscard]] constexpr Out convertColor(In in)
{
	if constexpr (std::same_as<In, Out>) {
		return in;
	} else {
		using T = value_type_t<Out>;
		auto x  = [](auto bare) {
      if constexpr (SameColorModel<In, Out>) {
        // Same model: convert the channel type directly - no float
        // intermediate needed and no model conversion to perform.
        return convertType<T>(bare);
      } else {
        // Different model: normalise to float, convert the model, then
        // convert to the target channel type.
        auto as_float = convertType<float>(bare);
        if constexpr (GrayColor<Out>) {
          return convertType<T>(toGray(as_float));
        } else if constexpr (RgbColor<Out>) {
          return convertType<T>(toRgb(as_float));
        } else if constexpr (LinearRgbColor<Out>) {
          return convertType<T>(toLrgb(as_float));
        } else if constexpr (LabColor<Out>) {
          return convertType<T>(toLab(as_float));
        } else if constexpr (LchColor<Out>) {
          return convertType<T>(toLch(as_float));
        } else {
          static_assert(false, "unhandled ColorModel - update convertColor");
        }
      }
		}(removeWeight(removeAlpha(in)));

		if constexpr (ColorWithAlpha<Out> && ColorWithWeight<Out>) {
			return addWeight(addAlpha(x, alpha<T>(in)), weight(in));
		} else if constexpr (ColorWithAlpha<Out>) {
			return addAlpha(x, alpha<T>(in));
		} else if constexpr (ColorWithWeight<Out>) {
			return addWeight(x, weight(in));
		} else {
			return x;
		}
	}
}

}  // namespace ufo::detail

#endif  // UFO_VISION_COLOR_DETAIL_CONVERT_HPP
