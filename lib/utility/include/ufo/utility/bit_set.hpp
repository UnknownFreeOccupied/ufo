/**
 * @author Daniel Duberg (danielduberg@gmail.com)
 * @see https://github.com/UnknownFreeOccupied/ufo
 * @version 1.0
 * @date 2026-02-22
 *
 * @copyright Copyright (c) 2020-2026, Daniel Duberg
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2020-2026, Daniel Duberg
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

#ifndef UFO_UTILITY_BIT_SET_HPP
#define UFO_UTILITY_BIT_SET_HPP

// STL
#include <bit>
#include <cassert>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>

namespace ufo
{

template <std::size_t N>
class BitSet
{
	static_assert(0 < N && 64 >= N, "BitSet is defined for [1..64] bits");

 public:
	using T = std::conditional_t<
	    8 >= N, std::uint8_t,
	    std::conditional_t<16 >= N, std::uint16_t,
	                       std::conditional_t<32 >= N, std::uint32_t, std::uint64_t>>>;

	using value_type = T;

 private:
	static constexpr T const ALL_SET = []() {
		if constexpr (std::numeric_limits<T>::digits == N) {
			return static_cast<T>(~T(0));
		} else {
			return static_cast<T>(~(static_cast<T>(~T(0)) << N));
		}
	}();

 public:
	struct Reference {
		friend class BitSet<N>;

	 public:
		constexpr Reference& operator=(bool v) noexcept
		{
			T const x = set_;
			T const y = static_cast<T>(~bit_);
			T const z = static_cast<T>(v) << pos_;

			set_ = (x & y) | z;
			return *this;
		}

		constexpr Reference& operator=(Reference const& v) noexcept { return operator=(!!v); }

		constexpr operator bool() const noexcept { return !!(set_ & bit_); }

		constexpr bool operator~() const noexcept { return !(set_ & bit_); }

		constexpr Reference& flip() noexcept
		{
			set_ ^= bit_;
			return *this;
		}

	 private:
		constexpr Reference(T& set, std::size_t pos) noexcept
		    : set_(set), pos_(pos), bit_(static_cast<T>(T(1) << pos))
		{
		}

	 private:
		T& set_;
		T  pos_;
		T  bit_;
	};

	constexpr BitSet() noexcept = default;

	constexpr BitSet(T val) noexcept : set_(val & ALL_SET) {}

	template <class CharT, class Traits, class Alloc>
	explicit BitSet(std::basic_string<CharT, Traits, Alloc> const&              str,
	                typename std::basic_string<CharT, Traits, Alloc>::size_type pos = 0,
	                typename std::basic_string<CharT, Traits, Alloc>::size_type n =
	                    std::basic_string<CharT, Traits, Alloc>::npos,
	                CharT zero = CharT('0'), CharT one = CharT('1'))
	{
		// TODO: Implement
	}

	template <class CharT>
	explicit BitSet(
	    CharT const*                                 str,
	    typename std::basic_string<CharT>::size_type n = std::basic_string<CharT>::npos,
	    CharT zero = CharT('0'), CharT one = CharT('1'))
	{
		// TODO: Implement
	}

	constexpr bool operator==(BitSet rhs) const noexcept { return set_ == rhs.set_; }

	constexpr bool operator!=(BitSet rhs) const noexcept { return !(*this == rhs); }

	[[nodiscard]] constexpr bool operator[](std::size_t pos) const
	{
		assert(N > pos);
		return (set_ >> pos) & T(1);
	}

	[[nodiscard]] constexpr Reference operator[](std::size_t pos)
	{
		assert(N > pos);
		return Reference(set_, pos);
	}

	[[nodiscard]] bool test(std::size_t pos) const
	{
		if (size() <= pos) {
			std::out_of_range("position (which is " + std::to_string(pos) +
			                  ") >= size (which is " + std::to_string(size()) + ")");
		}
		return operator[](pos);
	}

	[[nodiscard]] constexpr bool all() const noexcept { return ALL_SET == set_; }

	[[nodiscard]] constexpr bool any() const noexcept { return !none(); }

	[[nodiscard]] constexpr bool none() const noexcept { return T(0) == set_; }

	[[nodiscard]] constexpr bool some() const noexcept { return !all() && !none(); }

	[[nodiscard]] constexpr std::size_t count() const noexcept
	{
		return std::popcount(set_);
	}

	[[nodiscard]] static constexpr std::size_t size() noexcept { return N; }

	constexpr BitSet& operator&=(BitSet const other) noexcept
	{
		set_ &= other.set_;
		return *this;
	}

	constexpr BitSet& operator|=(BitSet const& other) noexcept
	{
		set_ |= other.set_;
		return *this;
	}

	constexpr BitSet& operator^=(BitSet const& other) noexcept
	{
		set_ ^= other.set_;
		return *this;
	}

	constexpr BitSet operator~() const noexcept { return BitSet(static_cast<T>(~set_)); }

	constexpr void set() noexcept { set_ = ALL_SET; }

	constexpr void set(std::size_t pos, bool value)
	{
		assert(N > pos);

		T const x = set_;
		T const y = static_cast<T>(~(T(1) << pos));
		T const z = static_cast<T>(value) << pos;

		set_ = (x & y) | z;
	}

	constexpr void set(std::size_t pos)
	{
		assert(N > pos);
		set_ |= T(1) << pos;
	}

	constexpr void reset() noexcept { set_ = 0; }

	constexpr void reset(std::size_t pos)
	{
		assert(N > pos);
		set_ &= static_cast<T>(~(T(1) << pos));
	}

	constexpr void flip() noexcept { set_ = static_cast<T>(~set_); }

	constexpr void flip(std::size_t pos)
	{
		assert(N > pos);
		set_ ^= static_cast<T>(T(1) << pos);
	}

	[[nodiscard]] constexpr value_type data() const { return set_; }

 private:
	T set_{};
};

template <std::size_t N>
constexpr BitSet<N> operator&(BitSet<N> lhs, BitSet<N> rhs) noexcept
{
	return BitSet<N>(lhs.data() & rhs.data());
}

template <std::size_t N>
constexpr BitSet<N> operator|(BitSet<N> lhs, BitSet<N> rhs) noexcept
{
	return BitSet<N>(lhs.data() | rhs.data());
}

template <std::size_t N>
constexpr BitSet<N> operator^(BitSet<N> lhs, BitSet<N> rhs) noexcept
{
	return BitSet<N>(lhs.data() ^ rhs.data());
}

template <class CharT, class Traits, std::size_t N>
std::basic_ostream<CharT, Traits>& operator<<(std::basic_ostream<CharT, Traits>& os,
                                              BitSet<N>                          x)
{
	return os << +x.data();
}

template <class CharT, class Traits, std::size_t N>
std::basic_istream<CharT, Traits>& operator>>(std::basic_istream<CharT, Traits>& is,
                                              BitSet<N>&                         x)
{
	typename BitSet<N>::value_type data;
	is >> data;
	x = BitSet<N>(data);
	return is;
}
}  // namespace ufo

namespace std
{
template <std::size_t N>
struct hash<ufo::BitSet<N>> {
	std::size_t operator()(ufo::BitSet<N> x) const { return x.data(); }
};
}  // namespace std

#endif  // UFO_UTILITY_BIT_SET_HPP