/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
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
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_CONTAINER_TREE_PREDICATE_PREDICATE_INTERVAL_HPP
#define UFO_CONTAINER_TREE_PREDICATE_PREDICATE_INTERVAL_HPP

// UFO
#include <ufo/utility/type_traits.hpp>

// STL
#include <algorithm>
#include <cassert>
#include <cmath>
#include <concepts>
#include <limits>

namespace ufo::pred
{
template <class T, bool Negated>
struct PredicateInterval {
	using value_type = T::value_type;

	constexpr PredicateInterval() noexcept = default;

	constexpr PredicateInterval(value_type value) noexcept : min(value), max(value) {}

	constexpr PredicateInterval(value_type min, value_type max) noexcept
	    : min(min), max(max)
	{
	}

	value_type min = std::numeric_limits<value_type>::lowest();
	value_type max = std::numeric_limits<value_type>::max();
};

template <class T, bool Negated>
[[nodiscard]] constexpr PredicateInterval<T, !Negated> operator!(
    PredicateInterval<T, Negated> const& p) noexcept
{
	return PredicateInterval<T, !Negated>(p.min, p.max);
}

template <class T, bool Negated>
[[nodiscard]] constexpr PredicateInterval<T, Negated> operator==(
    [[maybe_unused]] PredicateInterval<T, Negated> const& p,
    typename T::value_type const&                         v) noexcept
{
	return PredicateInterval<T, Negated>(v, v);
}

template <class T, bool Negated>
[[nodiscard]] constexpr PredicateInterval<T, Negated> operator==(
    typename T::value_type const&                         v,
    [[maybe_unused]] PredicateInterval<T, Negated> const& p) noexcept
{
	return PredicateInterval<T, Negated>(v, v);
}

template <class T, bool Negated>
[[nodiscard]] constexpr PredicateInterval<T, true> operator!=(
    [[maybe_unused]] PredicateInterval<T, Negated> const& p,
    typename T::value_type const&                         v) noexcept
{
	return PredicateInterval<T, true>(v, v);
}

template <class T, bool Negated>
[[nodiscard]] constexpr PredicateInterval<T, true> operator!=(
    typename T::value_type const&                         v,
    [[maybe_unused]] PredicateInterval<T, Negated> const& p) noexcept
{
	return PredicateInterval<T, true>(v, v);
}

template <class T, bool Negated>
[[nodiscard]] constexpr PredicateInterval<T, Negated> operator<=(
    PredicateInterval<T, Negated> const& p, typename T::value_type const& v) noexcept
{
	return PredicateInterval<T, Negated>(std::min(p.min, v), v);
}

template <class T, bool Negated>
[[nodiscard]] constexpr PredicateInterval<T, Negated> operator<=(
    typename T::value_type const& v, PredicateInterval<T, Negated> const& p) noexcept
{
	return PredicateInterval<T, Negated>(v, std::max(p.max, v));
}

template <class T, bool Negated>
[[nodiscard]] constexpr PredicateInterval<T, Negated> operator<(
    PredicateInterval<T, Negated> const& p, typename T::value_type const& v) noexcept
{
	using value_type = typename T::value_type;
	if constexpr (std::floating_point<value_type>) {
		return p <= std::nextafter(v, std::numeric_limits<value_type>::lowest());
	} else {
		assert(v > v - value_type(1));
		return p <= v - value_type(1);
	}
}

template <class T, bool Negated>
[[nodiscard]] constexpr PredicateInterval<T, Negated> operator<(
    typename T::value_type const& v, PredicateInterval<T, Negated> const& p) noexcept
{
	using value_type = typename T::value_type;
	if constexpr (std::floating_point<value_type>) {
		return std::nextafter(v, std::numeric_limits<value_type>::max()) <= p;
	} else {
		assert(v < v + value_type(1));
		return v + value_type(1) <= p;
	}
}

template <class T, bool Negated>
[[nodiscard]] constexpr PredicateInterval<T, Negated> operator>=(
    PredicateInterval<T, Negated> const& p, typename T::value_type const& v) noexcept
{
	return v <= p;
}

template <class T, bool Negated>
[[nodiscard]] constexpr PredicateInterval<T, Negated> operator>=(
    typename T::value_type const& v, PredicateInterval<T, Negated> const& p) noexcept
{
	return p <= v;
}

template <class T, bool Negated>
[[nodiscard]] constexpr PredicateInterval<T, Negated> operator>(
    PredicateInterval<T, Negated> const& p, typename T::value_type const& v) noexcept
{
	return v < p;
}

template <class T, bool Negated>
[[nodiscard]] constexpr PredicateInterval<T, Negated> operator>(
    typename T::value_type const& v, PredicateInterval<T, Negated> const& p) noexcept
{
	return p < v;
}
}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_PREDICATE_INTERVAL_HPP