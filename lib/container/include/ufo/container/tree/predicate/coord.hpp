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

#ifndef UFO_CONTAINER_TREE_PREDICATE_COORD_HPP
#define UFO_CONTAINER_TREE_PREDICATE_COORD_HPP

// UFO
#include <ufo/container/tree/predicate/filter.hpp>
#include <ufo/container/tree/predicate/predicate_interval.hpp>

namespace ufo::pred
{
namespace detail
{
template <std::size_t Axis>
struct Coord {
	using value_type = float;
};
}  // namespace detail

template <std::size_t Axis, bool Negated = false>
using Coord = PredicateInterval<detail::Coord<Axis>, Negated>;

template <bool Negated = false>
using X = Coord<0, Negated>;

template <bool Negated = false>
using Y = Coord<1, Negated>;

template <bool Negated = false>
using Z = Coord<2, Negated>;

template <bool Negated = false>
using W = Coord<3, Negated>;

static constexpr inline X<false> const x;
static constexpr inline Y<false> const y;
static constexpr inline Z<false> const z;
static constexpr inline W<false> const w;

template <std::size_t Axis, bool Negated>
struct Filter<Coord<Axis, Negated>> {
	using Pred = Coord<Axis, Negated>;

	template <class Tree>
	static constexpr void init(Pred&, Tree const&) noexcept
	{
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnableValue(Pred const&, Value const&) noexcept
	{
		return true;
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               typename Tree::Node const& n) noexcept
	{
		auto c  = t.centerAxis(n, Axis);
		auto hl = t.halfLength(n)[Axis];

		if constexpr (Negated) {
			// Check if the interval is outside the node
			return p.min > c + hl || p.max < c - hl;
		} else {
			// Check if the node overlaps with the interval
			return p.min <= c + hl && p.max >= c - hl;
		}
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                typename Tree::Node const& n) noexcept
	{
		auto c  = t.centerAxis(n, Axis);
		auto hl = t.halfLength(n)[Axis];

		if constexpr (Negated) {
			// Check if the whole node is contained in the negated interval
			return !(p.min <= c - hl && p.max >= c + hl);
		} else {
			// Check if the node overlaps with the interval
			return p.min <= c + hl && p.max >= c - hl;
		}
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnableRay(Pred const& p, Tree const& t,
	                                                  typename Tree::Node const& n,
	                                                  typename Tree::Ray const&) noexcept
	{
		return returnable(p, t, n);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversableRay(Pred const& p, Tree const& t,
	                                                   typename Tree::Node const& n,
	                                                   typename Tree::Ray const&) noexcept
	{
		return traversable(p, t, n);
	}
};
}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_COORD_HPP