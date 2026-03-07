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

#ifndef UFO_CONTAINER_TREE_PREDICATE_OR_HPP
#define UFO_CONTAINER_TREE_PREDICATE_OR_HPP

// UFO
#include <ufo/container/tree/predicate/filter.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <tuple>
#include <type_traits>
#include <utility>

namespace ufo::pred
{
template <Filterable... Preds>
struct Or {
	Or(Preds&&... preds) noexcept : preds(std::forward<Preds>(preds)...) {}

	std::tuple<Preds...> preds;
};

namespace detail
{
template <class T>
[[nodiscard]] constexpr auto getOrTuple(T&& t) noexcept
{
	if constexpr (is_specialization_of_v<Or, std::remove_cvref_t<T>>) {
		return std::forward<T>(t).preds;
	} else {
		return std::forward_as_tuple(std::forward<T>(t));
	}
}
}  // namespace detail

template <Filterable PredLeft, Filterable PredRight>
[[nodiscard]] constexpr auto operator||(PredLeft&& p1, PredRight&& p2) noexcept
{
	return std::apply(
	    [](auto&&... preds) {
		    return Or<std::remove_cvref_t<decltype(preds)>...>(
		        std::forward<decltype(preds)>(preds)...);
	    },
	    std::tuple_cat(detail::getOrTuple(std::forward<PredLeft>(p1)),
	                   detail::getOrTuple(std::forward<PredRight>(p2))));
}

template <Filterable... Preds>
struct Filter<Or<Preds...>> {
	using Pred = Or<Preds...>;

	template <class Tree>
	static constexpr void init(Pred& p, Tree const& t) noexcept
	{
		std::apply([&t](auto&&... preds) { (Filter<Preds>::init(preds, t), ...); }, p.preds);
	}

	template <class Value>
	[[nodiscard]] static constexpr bool returnableValue(Pred const&  p,
	                                                    Value const& v) noexcept
	{
		return std::apply(
		    [&v](auto&&... preds) {
			    return (Filter<Preds>::returnableValue(preds, v) || ...);
		    },
		    p.preds);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnable(Pred const& p, Tree const& t,
	                                               typename Tree::Node const& n) noexcept
	{
		return std::apply(
		    [&t, &n](auto&&... preds) {
			    return (Filter<Preds>::returnable(preds, t, n) || ...);
		    },
		    p.preds);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversable(Pred const& p, Tree const& t,
	                                                typename Tree::Node const& n) noexcept
	{
		return std::apply(
		    [&t, &n](auto&&... preds) {
			    return (Filter<Preds>::traversable(preds, t, n) || ...);
		    },
		    p.preds);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool returnableRay(Pred const& p, Tree const& t,
	                                                  typename Tree::Node const& n,
	                                                  typename Tree::Ray const&  r) noexcept
	{
		return std::apply(
		    [&t, &n, &r](auto&&... preds) {
			    return (Filter<Preds>::returnableRay(preds, t, n, r) || ...);
		    },
		    p.preds);
	}

	template <class Tree>
	[[nodiscard]] static constexpr bool traversableRay(Pred const& p, Tree const& t,
	                                                   typename Tree::Node const& n,
	                                                   typename Tree::Ray const& r) noexcept
	{
		return std::apply(
		    [&t, &n, &r](auto&&... preds) {
			    return (Filter<Preds>::traversableRay(preds, t, n, r) || ...);
		    },
		    p.preds);
	}
};

namespace detail
{
template <Filterable T, Filterable... Ts>
struct contains_pred<T, Or<Ts...>> : std::disjunction<contains_pred<T, Ts>...> {
};

template <Filterable T, Filterable... Ts>
struct contains_always_pred<T, Or<Ts...>>
    : std::conjunction<contains_always_pred<T, Ts>...> {
};
}  // namespace detail

}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_OR_HPP