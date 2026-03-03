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

#ifndef UFO_VISION_COLOR_DETAIL_ARITHMETIC_HPP
#define UFO_VISION_COLOR_DETAIL_ARITHMETIC_HPP

// UFO
#include <ufo/vision/color/concepts.hpp>
#include <ufo/vision/color/convert.hpp>
#include <ufo/vision/color/lab.hpp>
#include <ufo/vision/color/lch.hpp>
#include <ufo/vision/color/lrgb.hpp>
#include <ufo/vision/color/rgb.hpp>
#include <ufo/vision/color/type_traits.hpp>

namespace ufo::detail
{
template <Color C, class BinaryOp>
[[nodiscard]] constexpr C reduce(C lhs, C rhs, BinaryOp op)
{
	if constexpr (ColorWithWeight<C> && !FloatingPointColor<C>) {
		return convert<C>(reduce(convert<float>(lhs), convert<float>(rhs), op));
	} else {
		if constexpr (GrayColor<C>) {
			lhs.gray = op(lhs.gray, rhs.gray);
		} else if constexpr (RgbFamilyColor<C>) {
			lhs.red   = op(lhs.red, rhs.red);
			lhs.green = op(lhs.green, rhs.green);
			lhs.blue  = op(lhs.blue, rhs.blue);
		} else if constexpr (LabColor<C>) {
			lhs.lightness = op(lhs.lightness, rhs.lightness);
			lhs.a         = op(lhs.a, rhs.a);
			lhs.b         = op(lhs.b, rhs.b);
		} else if constexpr (LchColor<C>) {
			lhs.lightness = op(lhs.lightness, rhs.lightness);
			lhs.chroma    = op(lhs.chroma, rhs.chroma);
			lhs.hue       = op(lhs.hue, rhs.hue);
		}

		if constexpr (ColorWithAlpha<C>) {
			lhs.alpha = op(lhs.alpha, rhs.alpha);
		}

		if constexpr (ColorWithWeight<C>) {
			lhs.weight = op(lhs.weight, rhs.weight);
		}
	}

	return lhs;
}

template <Color C, class BinaryOp>
[[nodiscard]] constexpr C reduce(C lhs, float rhs, BinaryOp op)
{
	if constexpr (ColorWithWeight<C> && !FloatingPointColor<C>) {
		return convert<C>(reduce(convert<float>(lhs), rhs, op));
	} else {
		if constexpr (GrayColor<C>) {
			lhs.gray = op(lhs.gray, rhs);
		} else if constexpr (RgbFamilyColor<C>) {
			lhs.red   = op(lhs.red, rhs);
			lhs.green = op(lhs.green, rhs);
			lhs.blue  = op(lhs.blue, rhs);
		} else if constexpr (LabColor<C>) {
			lhs.lightness = op(lhs.lightness, rhs);
			lhs.a         = op(lhs.a, rhs);
			lhs.b         = op(lhs.b, rhs);
		} else if constexpr (LchColor<C>) {
			lhs.lightness = op(lhs.lightness, rhs);
			lhs.chroma    = op(lhs.chroma, rhs);
			lhs.hue       = op(lhs.hue, rhs);
		}

		if constexpr (ColorWithAlpha<C>) {
			lhs.alpha = op(lhs.alpha, rhs);
		}

		if constexpr (ColorWithWeight<C>) {
			lhs.weight = op(lhs.weight, rhs);
		}
	}

	return lhs;
}

template <Color C, class BinaryOp>
[[nodiscard]] constexpr C reduce(float lhs, C rhs, BinaryOp op)
{
	if constexpr (ColorWithWeight<C> && !FloatingPointColor<C>) {
		return convert<C>(reduce(lhs, convert<float>(rhs), op));
	} else {
		if constexpr (GrayColor<C>) {
			rhs.gray = op(lhs, rhs.gray);
		} else if constexpr (RgbFamilyColor<C>) {
			rhs.red   = op(lhs, rhs.red);
			rhs.green = op(lhs, rhs.green);
			rhs.blue  = op(lhs, rhs.blue);
		} else if constexpr (LabColor<C>) {
			rhs.lightness = op(lhs, rhs.lightness);
			rhs.a         = op(lhs, rhs.a);
			rhs.b         = op(lhs, rhs.b);
		} else if constexpr (LchColor<C>) {
			rhs.lightness = op(lhs, rhs.lightness);
			rhs.chroma    = op(lhs, rhs.chroma);
			rhs.hue       = op(lhs, rhs.hue);
		}

		if constexpr (ColorWithAlpha<C>) {
			rhs.alpha = op(lhs, rhs.alpha);
		}

		if constexpr (ColorWithWeight<C>) {
			rhs.weight = op(lhs, rhs.weight);
		}
	}

	return rhs;
}
}  // namespace ufo::detail

#endif  // UFO_VISION_COLOR_DETAIL_ARITHMETIC_HPP