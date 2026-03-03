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
/**
 * @defgroup morton Morton Module
 * @brief All Morton-related classes and functions.
 * @{
 */

#ifndef UFO_MORTON_HPP
#define UFO_MORTON_HPP

// UFO
#include <ufo/math/vec.hpp>

// STL
#include <cassert>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <tuple>
#include <utility>

#if defined(UFO_BMI2)
#include <immintrin.h>
#endif

namespace ufo
{
namespace detail
{
template <std::size_t Dim, std::unsigned_integral T, std::size_t Offset>
constexpr inline T MortonMask = [] {
	T           mask{};
	std::size_t bits = std::numeric_limits<T>::digits;
	for (std::size_t i = 0; i < bits / Dim; ++i) {
		mask |= T(1) << (i * Dim + Offset);
	}
	return mask;
}();

template <std::size_t Dim, std::unsigned_integral T, std::size_t Offset = 0>
  requires(Dim >= 1 && Offset < Dim)
[[nodiscard]] constexpr T spread(T x)
{
	if constexpr (1 == Dim) {
		return x;
	} else {
#if defined(UFO_BMI2)
		if !consteval {
			if constexpr (sizeof(T) == 4) {
				return _pdep_u32(static_cast<std::uint32_t>(x),
				                 MortonMask<Dim, std::uint32_t, Offset>);
			} else if constexpr (sizeof(T) == 8) {
				return _pdep_u64(static_cast<std::uint64_t>(x),
				                 MortonMask<Dim, std::uint64_t, Offset>);
			}
		}
#endif
		if constexpr (0 != Offset) {
			return spread<Dim, T, 0>(x) << Offset;
		} else if constexpr (2 == Dim) {
			T m(x);
			if constexpr (sizeof(T) == 8) {
				m &= 0x00000000FFFFFFFF;
				m = (m | (m << 16)) & 0x0000FFFF0000FFFF;
				m = (m | (m << 8)) & 0x00FF00FF00FF00FF;
				m = (m | (m << 4)) & 0x0F0F0F0F0F0F0F0F;
				m = (m | (m << 2)) & 0x3333333333333333;
				m = (m | (m << 1)) & 0x5555555555555555;
			} else {
				m &= 0x0000FFFF;
				m = (m | (m << 8)) & 0x00FF00FF;
				m = (m | (m << 4)) & 0x0F0F0F0F;
				m = (m | (m << 2)) & 0x33333333;
				m = (m | (m << 1)) & 0x55555555;
			}
			return m;
		} else if constexpr (3 == Dim) {
			T m(x);
			if constexpr (sizeof(T) == 8) {
				m &= 0x1FFFFF;
				m = (m | m << 32) & 0x1F00000000FFFF;
				m = (m | m << 16) & 0x1F0000FF0000FF;
				m = (m | m << 8) & 0x100F00F00F00F00F;
				m = (m | m << 4) & 0x10C30C30C30C30C3;
				m = (m | m << 2) & 0x1249249249249249;
			} else {
				m &= 0x3FF;
				m = (m | m << 16) & 0x30000FF;
				m = (m | m << 8) & 0x300F00F;
				m = (m | m << 4) & 0x30C30C3;
				m = (m | m << 2) & 0x9249249;
			}
			return m;
		} else {
			T           res{};
			std::size_t bits = std::numeric_limits<T>::digits / Dim;
			for (std::size_t i = 0; i < bits; ++i) {
				res |= (x & (T(1) << i)) << (i * (Dim - 1));
			}
			return res;
		}
	}
}

template <std::size_t Dim, std::unsigned_integral T, std::size_t Offset = 0>
  requires(Dim >= 1 && Offset < Dim)
[[nodiscard]] constexpr T compact(T m)
{
	if constexpr (1 == Dim) {
		return m;
	} else {
#if defined(UFO_BMI2)
		if !consteval {
			if constexpr (sizeof(T) == 4) {
				return _pext_u32(static_cast<std::uint32_t>(m),
				                 MortonMask<Dim, std::uint32_t, Offset>);
			} else if constexpr (sizeof(T) == 8) {
				return _pext_u64(static_cast<std::uint64_t>(m),
				                 MortonMask<Dim, std::uint64_t, Offset>);
			}
		}
#endif
		if constexpr (0 != Offset) {
			return compact<Dim, T, 0>(m >> Offset);
		} else if constexpr (2 == Dim) {
			T x(m & MortonMask<Dim, T, 0>);
			if constexpr (sizeof(T) == 8) {
				x = (x ^ (x >> 1)) & 0x3333333333333333;
				x = (x ^ (x >> 2)) & 0x0F0F0F0F0F0F0F0F;
				x = (x ^ (x >> 4)) & 0x00FF00FF00FF00FF;
				x = (x ^ (x >> 8)) & 0x0000FFFF0000FFFF;
				x = (x ^ (x >> 16)) & 0x00000000FFFFFFFF;
			} else {
				x = (x ^ (x >> 1)) & 0x33333333;
				x = (x ^ (x >> 2)) & 0x0F0F0F0F;
				x = (x ^ (x >> 4)) & 0x00FF00FF;
				x = (x ^ (x >> 8)) & 0x0000FFFF;
			}
			return x;
		} else if constexpr (3 == Dim) {
			T x(m & MortonMask<Dim, T, 0>);
			if constexpr (sizeof(T) == 8) {
				x = (x ^ (x >> 2)) & 0x10C30C30C30C30C3;
				x = (x ^ (x >> 4)) & 0x100F00F00F00F00F;
				x = (x ^ (x >> 8)) & 0x1F0000FF0000FF;
				x = (x ^ (x >> 16)) & 0x1F00000000FFFF;
				x = (x ^ (x >> 32)) & 0x1FFFFF;
			} else {
				x = (x ^ (x >> 2)) & 0x30C30C3;
				x = (x ^ (x >> 4)) & 0x300F00F;
				x = (x ^ (x >> 8)) & 0x30000FF;
				x = (x ^ (x >> 16)) & 0x3FF;
			}
			return x;
		} else {
			T           res{};
			std::size_t bits = std::numeric_limits<T>::digits / Dim;
			for (std::size_t i = 0; i < bits; ++i) {
				res |= (m & (T(1) << (i * Dim))) >> (i * (Dim - 1));
			}
			return res;
		}
	}
}

}  // namespace detail

template <std::size_t Dim>
struct Morton {
	static constexpr std::uint32_t X_M_32 = detail::MortonMask<Dim, std::uint32_t, 0>;
	static constexpr std::uint64_t X_M_64 = detail::MortonMask<Dim, std::uint64_t, 0>;

	static constexpr std::size_t LEVELS_32 = 32 / Dim;
	static constexpr std::size_t LEVELS_64 = 64 / Dim;

	template <std::convertible_to<std::uint32_t>... Args>
	  requires(sizeof...(Args) == Dim)
	[[nodiscard]] static constexpr std::uint32_t encode32(Args... args)
	{
		return encode_impl<std::uint32_t>(std::make_index_sequence<Dim>{},
		                                  static_cast<std::uint32_t>(args)...);
	}

	[[nodiscard]] static constexpr std::uint32_t encode32(Vec<Dim, std::uint32_t> const& v)
	{
		return std::apply([](auto... args) { return encode32(args...); }, v.fields);
	}

	template <std::convertible_to<std::uint32_t>... Args>
	  requires(sizeof...(Args) == Dim)
	[[nodiscard]] static constexpr std::uint64_t encode64(Args... args)
	{
		return encode_impl<std::uint64_t>(std::make_index_sequence<Dim>{},
		                                  static_cast<std::uint32_t>(args)...);
	}

	[[nodiscard]] static constexpr std::uint64_t encode64(Vec<Dim, std::uint32_t> const& v)
	{
		return std::apply([](auto... args) { return encode64(args...); }, v.fields);
	}

	[[nodiscard]] static constexpr Vec<Dim, std::uint32_t> decode32(std::uint32_t m)
	{
		return decode_impl<std::uint32_t>(m, std::make_index_sequence<Dim>{});
	}

	[[nodiscard]] static constexpr std::uint32_t decode32(std::uint32_t m, std::size_t pos)
	{
		assert(Dim > pos);
		return decode_pos_impl<std::uint32_t>(m, pos, std::make_index_sequence<Dim>{});
	}

	[[nodiscard]] static constexpr Vec<Dim, std::uint32_t> decode64(std::uint64_t m)
	{
		return decode_impl<std::uint64_t>(m, std::make_index_sequence<Dim>{});
	}

	[[nodiscard]] static constexpr std::uint32_t decode64(std::uint64_t m, std::size_t pos)
	{
		assert(Dim > pos);
		return decode_pos_impl<std::uint64_t>(m, pos, std::make_index_sequence<Dim>{});
	}

	[[nodiscard]] static constexpr std::uint32_t spread32(std::uint32_t x)
	{
		return detail::spread<Dim, std::uint32_t, 0>(x);
	}

	[[nodiscard]] static constexpr std::uint64_t spread64(std::uint32_t x)
	{
		return detail::spread<Dim, std::uint64_t, 0>(static_cast<std::uint64_t>(x));
	}

	[[nodiscard]] static constexpr std::uint32_t compact32(std::uint32_t m)
	{
		return detail::compact<Dim, std::uint32_t, 0>(m);
	}

	[[nodiscard]] static constexpr std::uint32_t compact64(std::uint64_t m)
	{
		return static_cast<std::uint32_t>(detail::compact<Dim, std::uint64_t, 0>(m));
	}

 private:
	template <std::unsigned_integral T, std::size_t... Is, std::convertible_to<T>... Args>
	[[nodiscard]] static constexpr T encode_impl(std::index_sequence<Is...>, Args... args)
	{
		return (detail::spread<Dim, T, Is>(static_cast<T>(args)) | ...);
	}

	template <std::unsigned_integral T, std::size_t... Is>
	[[nodiscard]] static constexpr Vec<Dim, std::uint32_t> decode_impl(
	    T m, std::index_sequence<Is...>)
	{
		return Vec<Dim, std::uint32_t>(
		    static_cast<std::uint32_t>(detail::compact<Dim, T, Is>(m))...);
	}

	template <std::unsigned_integral T, std::size_t... Is>
	[[nodiscard]] static constexpr std::uint32_t decode_pos_impl(T m, std::size_t pos,
	                                                             std::index_sequence<Is...>)
	{
		std::uint32_t res{};
		((Is == pos ? res = static_cast<std::uint32_t>(detail::compact<Dim, T, Is>(m)) : 0),
		 ...);
		return res;
	}
};

}  // namespace ufo

#endif  // UFO_MORTON_HPP

/**
 * @}
 */