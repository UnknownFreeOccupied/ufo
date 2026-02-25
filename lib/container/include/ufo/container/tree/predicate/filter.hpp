/**
 * UFOTree: An Efficient Probabilistic 3D Treeping Framework That Embraces the Unknown
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

#ifndef UFO_CONTAINER_TREE_PREDICATE_FILTER_HPP
#define UFO_CONTAINER_TREE_PREDICATE_FILTER_HPP

// STL
#include <type_traits>

namespace ufo::pred
{
template <class Predicate>
struct Filter {
	static_assert(false, "Predicate not implemented correctly.");
};

//
// Contains predicate
//

namespace detail
{
template <class, class>
struct contains_pred : std::false_type {
};

template <class T>
struct contains_pred<T, T> : std::true_type {
};

template <class, class>
struct contains_always_pred : std::false_type {
};

template <class T>
struct contains_always_pred<T, T> : std::true_type {
};
}  // namespace detail

template <class Pred, class Preds>
constexpr inline bool contains_pred_v = detail::contains_pred<Pred, Preds>::value;

template <class Pred, class Preds>
constexpr inline bool contains_always_pred_v =
    detail::contains_always_pred<Pred, Preds>::value;

//
// Predicate concept
//

template <class T>
concept Filterable = requires { typename Filter<T>; };

}  // namespace ufo::pred

#endif  // UFO_CONTAINER_TREE_PREDICATE_FILTER_HPP