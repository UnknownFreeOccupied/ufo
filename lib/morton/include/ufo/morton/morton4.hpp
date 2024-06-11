/*!
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

#ifndef UFO_MORTON4_HPP
#define UFO_MORTON4_HPP

// UFO
#include <ufo/morton/morton.hpp>

// STL
#include <array>
#include <cstdint>

#if defined(UFO_BMI2)
#if defined(__i386__) || defined(__x86_64__) && (defined(__BMI2__) || defined(__AVX2__))
#include <immintrin.h>
#elif defined(__ARM_FEATURE_SIMD32) || defined(__ARM_NEON)
#include <arm_neon.h>
#endif
#endif

namespace ufo
{
template <>
struct Morton<4> {
	static constexpr std::uint_fast64_t const X_MASK = 0x1111111111111111;
	static constexpr std::uint_fast64_t const Y_MASK = 0x2222222222222222;
	static constexpr std::uint_fast64_t const Z_MASK = 0x4444444444444444;
	static constexpr std::uint_fast64_t const W_MASK = 0x8888888888888888;

	[[nodiscard]] static constexpr std::uint_fast64_t encode(std::uint_fast16_t x,
	                                                         std::uint_fast16_t y,
	                                                         std::uint_fast16_t z,
	                                                         std::uint_fast16_t w)
	{
#if defined(UFO_BMI2)
		return _pdep_u64(static_cast<std::uint_fast64_t>(x), X_MASK) |
		       _pdep_u64(static_cast<std::uint_fast64_t>(y), Y_MASK) |
		       _pdep_u64(static_cast<std::uint_fast64_t>(z), Z_MASK) |
		       _pdep_u64(static_cast<std::uint_fast64_t>(w), W_MASK);
#else
		return spread(x) | (spread(y) << 1) | (spread(z) << 2) | (spread(w) << 3);
#endif
	}

	[[nodiscard]] static constexpr std::array<std::uint_fast16_t, 4> decode(
	    std::uint_fast64_t x)
	{
#if defined(UFO_BMI2)
		return {static_cast<std::uint_fast32_t>(_pext_u64(x, X_MASK)),
		        static_cast<std::uint_fast32_t>(_pext_u64(x, Y_MASK)),
		        static_cast<std::uint_fast32_t>(_pext_u64(x, Z_MASK)),
		        static_cast<std::uint_fast32_t>(_pext_u64(x, W_MASK))};
#else
		return {compact(x), compact(x >> 1), compact(x >> 2), compact(x >> 3)};
#endif
	}

	[[nodiscard]] static constexpr std::uint_fast64_t spread(std::uint_fast16_t v)
	{
#if defined(UFO_BMI2)
		return _pdep_u64(static_cast<std::uint_fast64_t>(x), X_MASK);
#else
		std::uint_fast64_t x(v);
		x &= 0xFFFF;
		x = (x | (x << 32)) & 0xF800000007FF;
		x = (x | (x << 16)) & 0xF80007C0003F;
		x = (x | (x << 8)) & 0xC0380700C03807;
		x = (x | (x << 4)) & 0x843084308430843;
		x = (x | (x << 2)) & 0x909090909090909;
		x = (x | (x << 1)) & 0x1111111111111111;
		return x;
#endif
	}

	[[nodiscard]] static constexpr std::uint_fast16_t compact(std::uint_fast64_t x)
	{
#if defined(UFO_BMI2)
		return static_cast<std::uint_fast32_t>(_pext_u64(x, X_MASK));
#else
		x &= 0x1111111111111111;
		x = (x ^ (x >> 1)) & 0x909090909090909;
		x = (x ^ (x >> 2)) & 0x843084308430843;
		x = (x ^ (x >> 4)) & 0xC0380700C03807;
		x = (x ^ (x >> 8)) & 0xF80007C0003F;
		x = (x ^ (x >> 16)) & 0xF800000007FF;
		x = (x ^ (x >> 32)) & 0xFFFF;
		return static_cast<std::uint_fast16_t>(x);
#endif
	}
};
}  // namespace ufo

#endif  // UFO_MORTON4_HPP