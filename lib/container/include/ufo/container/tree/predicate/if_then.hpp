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

#ifndef UFO_CONTAINER_TREE_PREDICATE_IF_THEN_HPP
#define UFO_CONTAINER_TREE_PREDICATE_IF_THEN_HPP

// UFO
#include <ufo/container/tree/predicate/filter.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <type_traits>

namespace ufo::pred
{
template <Filterable PredPre, Filterable PredPost>
struct IfThen {
	IfThen(PredPre const& pre, PredPost const& post) noexcept : pre(pre), post(post) {}

	PredPre  pre;
	PredPost post;
};

template <Filterable PredPre, Filterable PredPost>
[[nodiscard]] constexpr IfThen<std::remove_cvref_t<PredPre>,
                               std::remove_cvref_t<PredPost>>
operator>>(PredPre&& pre, PredPost&& post) noexcept
{
	return IfThen(std::forward<PredPre>(pre), std::forward<PredPost>(post));
}

template <Filterable PredPre, Filterable PredPost>
struct Filter<IfThen<PredPre, PredPost>> {
	using Pred = IfThen<PredPre, PredPost>;

	template <class Tree>
	static constexpr void init(Pred& p, Tree const& t) noexcept
	{
		Filter<PredPre>::init(p.pre, t);
		Filter<PredPost>::init(p.post, t);
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnableValue(Pred const&  p,
	                                                    Value const& v) noexcept
	{
		return !Filter<PredPre>::returnableValue(p.pre, v) ||
		       Filter<PredPost>::returnableValue(p.post, v);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               typename Tree::Node const& n) noexcept
	{
		return !Filter<PredPre>::returnable(p.pre, t, n) ||
		       Filter<PredPost>::returnable(p.post, t, n);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const&, Tree const&,
	                                                typename Tree::Node const&) noexcept
	{
		// NOTE: This is not the same as the returnable check! If the pre-predicate
		// is false, we can still traverse the subtree corresponding to the post-predicate.
		return true;
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnableRay(Pred const& p, Tree const& t,
	                                                  typename Tree::Node const& n,
	                                                  typename Tree::Ray const&  r) noexcept
	{
		return !Filter<PredPre>::returnableRay(p.pre, t, n, r) ||
		       Filter<PredPost>::returnableRay(p.post, t, n, r);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversableRay(Pred const&, Tree const&,
	                                                   typename Tree::Node const&,
	                                                   typename Tree::Ray const&) noexcept
	{
		// NOTE: This is not the same as the returnable check! If the pre-predicate
		// is false, we can still traverse the subtree corresponding to the post-predicate.
		return true;
	}
};

namespace detail
{
template <class T, class L, class R>
struct contains_pred<T, IfThen<L, R>>
    : std::disjunction<contains_pred<T, L>, contains_pred<T, R>> {
};

template <class T, class L, class R>
struct contains_always_pred<T, IfThen<L, R>> : std::false_type {
};
}  // namespace detail
}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_IF_THEN_HPP
