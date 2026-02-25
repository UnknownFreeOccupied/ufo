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

#ifndef UFO_CONTAINER_TREE_PREDICATE_XOR_HPP
#define UFO_CONTAINER_TREE_PREDICATE_XOR_HPP

// UFO
#include <ufo/container/tree/predicate/filter.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <type_traits>
#include <utility>

namespace ufo::pred
{
template <Filterable PredLeft, Filterable PredRight>
struct Xor {
	Xor(PredLeft const& left, PredRight const& right) noexcept : left(left), right(right) {}

	PredLeft  left;
	PredRight right;
};

template <Filterable PredLeft, Filterable PredRight>
[[nodiscard]] constexpr Xor<std::remove_cvref_t<PredLeft>, std::remove_cvref_t<PredRight>>
operator^(PredLeft&& left, PredRight&& right) noexcept
{
	return Xor(std::forward<PredLeft>(left), std::forward<PredRight>(right));
}

template <Filterable PredLeft, Filterable PredRight>
struct Filter<Xor<PredLeft, PredRight>> {
	using Pred = Xor<PredLeft, PredRight>;

	template <class Tree>
	static constexpr void init(Pred& p, Tree const& t) noexcept
	{
		Filter<PredLeft>::init(p.left, t);
		Filter<PredRight>::init(p.right, t);
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnableValue(Pred const&  p,
	                                                    Value const& v) noexcept
	{
		return Filter<PredLeft>::returnableValue(p.left, v) !=
		       Filter<PredRight>::returnableValue(p.right, v);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               typename Tree::Node const& n) noexcept
	{
		return Filter<PredLeft>::returnable(p.left, t, n) !=
		       Filter<PredRight>::returnable(p.right, t, n);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                typename Tree::Node const& n) noexcept
	{
		// Note: This is not the same as the returnable check! If both predicates
		// are true, we can still traverse both subtrees. If both are false, we
		// can't traverse either subtree. If one is true and one is false, we can
		// traverse the subtree corresponding to the true predicate.
		return Filter<PredLeft>::traversable(p.left, t, n) ||
		       Filter<PredRight>::traversable(p.right, t, n);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnableRay(Pred const& p, Tree const& t,
	                                                  typename Tree::Node const& n,
	                                                  typename Tree::Ray const&  r) noexcept
	{
		return Filter<PredLeft>::returnableRay(p.left, t, n, r) !=
		       Filter<PredRight>::returnableRay(p.right, t, n, r);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversableRay(Pred const& p, Tree const& t,
	                                                   typename Tree::Node const& n,
	                                                   typename Tree::Ray const& r) noexcept
	{
		// Note: This is not the same as the returnable check! If both predicates
		// are true, we can still traverse both subtrees. If both are false, we
		// can't traverse either subtree. If one is true and one is false, we can
		// traverse the subtree corresponding to the true predicate.
		return Filter<PredLeft>::traversableRay(p.left, t, n, r) ||
		       Filter<PredRight>::traversableRay(p.right, t, n, r);
	}
};

namespace detail
{
template <Filterable T, Filterable L, Filterable R>
struct contains_pred<T, Xor<L, R>>
    : std::disjunction<contains_pred<T, L>, contains_pred<T, R>> {
};

template <Filterable T, Filterable L, Filterable R>
struct contains_always_pred<T, Xor<L, R>> : std::false_type {
};
}  // namespace detail
}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_XOR_HPP