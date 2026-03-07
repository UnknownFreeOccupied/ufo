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

#ifndef UFO_CONTAINER_TREE_PREDICATE_SPATIAL_HPP
#define UFO_CONTAINER_TREE_PREDICATE_SPATIAL_HPP

// UFO
#include <ufo/container/tree/predicate/and.hpp>
#include <ufo/container/tree/predicate/filter.hpp>
#include <ufo/container/tree/predicate/if_and_only_if.hpp>
#include <ufo/container/tree/predicate/if_then.hpp>
#include <ufo/container/tree/predicate/or.hpp>
#include <ufo/container/tree/predicate/xor.hpp>
#include <ufo/utility/type_traits.hpp>

// STL
#include <type_traits>

namespace ufo::pred
{
//
// Is spatial predicate
//

namespace detail
{
template <Filterable>
struct is_spatial_pred : std::false_type {
};
}  // namespace detail

template <Filterable T>
struct is_spatial_pred : detail::is_spatial_pred<std::decay_t<T>> {
};

// Helper variable template
template <Filterable T>
constexpr inline bool is_spatial_pred_v = is_spatial_pred<T>::value;

//
// Contains spatial predicate
//

namespace detail
{
template <Filterable Pred>
struct contains_spatial_pred : is_spatial_pred<Pred> {
};

template <Filterable... Preds>
struct contains_spatial_pred<And<Preds...>>
    : std::disjunction<contains_spatial_pred<Preds>...> {
};

template <Filterable L, Filterable R>
struct contains_spatial_pred<Iff<L, R>>
    : std::disjunction<contains_spatial_pred<L>, contains_spatial_pred<R>> {
};

template <Filterable L, Filterable R>
struct contains_spatial_pred<IfThen<L, R>>
    : std::disjunction<contains_spatial_pred<L>, contains_spatial_pred<R>> {
};

template <Filterable... Preds>
struct contains_spatial_pred<Or<Preds...>>
    : std::disjunction<contains_spatial_pred<Preds>...> {
};

template <Filterable L, Filterable R>
struct contains_spatial_pred<Xor<L, R>>
    : std::disjunction<contains_spatial_pred<L>, contains_spatial_pred<R>> {
};

}  // namespace detail

template <Filterable T>
using contains_spatial_pred = detail::contains_spatial_pred<std::decay_t<T>>;

template <Filterable T>
constexpr inline bool contains_spatial_pred_v = contains_spatial_pred<T>::value;

//
// Contains always spatial predicate
//

namespace detail
{
template <Filterable Pred>
struct contains_always_spatial_pred : is_spatial_pred<Pred> {
};

template <Filterable... Preds>
struct contains_always_spatial_pred<And<Preds...>>
    : std::disjunction<contains_always_spatial_pred<Preds>...> {
};

template <Filterable L, Filterable R>
struct contains_always_spatial_pred<Iff<L, R>> : std::false_type {
};

template <Filterable L, Filterable R>
struct contains_always_spatial_pred<IfThen<L, R>> : std::false_type {
};

template <Filterable... Preds>
struct contains_always_spatial_pred<Or<Preds...>>
    : std::conjunction<contains_always_spatial_pred<Preds>...> {
};

template <Filterable... Preds>
struct contains_always_spatial_pred<Xor<Preds...>> : std::false_type {
};
}  // namespace detail

template <Filterable T>
using contains_always_spatial_pred = detail::contains_always_spatial_pred<T>;

template <class T>
constexpr inline bool contains_always_spatial_pred_v =
    contains_always_spatial_pred<T>::value;

}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_SPATIAL_HPP