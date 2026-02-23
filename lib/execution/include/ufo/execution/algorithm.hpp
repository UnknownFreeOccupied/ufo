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

#ifndef UFO_EXECUTION_ALGORITHM_HPP
#define UFO_EXECUTION_ALGORITHM_HPP

// UFO
#include <ufo/execution/execution.hpp>

// STL
#include <algorithm>
#include <concepts>
#include <iterator>
#include <ranges>

namespace ufo
{
/**************************************************************************************
|                                                                                     |
|                                      For each                                       |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Applies @p f to each integer index in `[first, last)` sequentially.
 *
 * A convenience overload that avoids manually constructing an iterator range when
 * iterating over a contiguous index space (e.g., element indices of a flat array).
 *
 * @tparam Index  An integral type used as the loop index.
 * @tparam UnaryFunc  A callable with signature compatible with `void(Index)`.
 *
 * @param first  First index in the half-open range (inclusive).
 * @param last   One past the last index (exclusive).
 * @param f      Function object to invoke for each index.
 *
 * @return The (possibly moved) function object @p f after all invocations.
 */
template <std::integral Index, class UnaryFunc>
UnaryFunc for_each(Index first, Index last, UnaryFunc f)
{
	for (; last != first; ++first) {
		f(first);
	}
	return f;
}

/**
 * @brief Applies @p f to each integer index in `[first, last)` using the given
 * execution policy.
 *
 * Dispatches to the appropriate parallel backend at compile time:
 * - **STL**: wraps the range in an `IndexIterator` and forwards to
 *   `std::for_each` with the translated STL execution policy.
 * - **GCD** (`UFO_PAR_GCD`): uses `dispatch_apply` on the global concurrent queue.
 * - **TBB** (`UFO_PAR_TBB`): uses `oneapi::tbb::parallel_for`.
 * - **OMP**: uses an `#pragma omp parallel for` loop.
 *
 * @tparam ExecutionPolicy  A UFO execution policy type (see `execution.hpp`).
 * @tparam Index            An integral type used as the loop index.
 * @tparam UnaryFunc        A callable with signature compatible with `void(Index)`.
 *
 * @param policy  The execution policy controlling parallelism.
 * @param first   First index in the half-open range (inclusive).
 * @param last    One past the last index (exclusive).
 * @param f       Function object to invoke for each index.
 */
template <class T, std::integral Index, class UnaryFunc>
  requires execution::ExecutionPolicy<T>
void for_each(T&& policy, Index first, Index last, UnaryFunc f)
{
	if constexpr (execution::STLBackend<T>) {
		auto r = std::views::iota(first, last);
		std::for_each(execution::toSTL(std::forward<T>(policy)), std::ranges::begin(r),
		              std::ranges::end(r), f);
	}
#if defined(UFO_PAR_GCD)
	else if constexpr (execution::GCDBackend<T>) {
		dispatch_apply(static_cast<std::size_t>(last - first),
		               dispatch_get_global_queue(0, 0), ^(std::size_t i) {
			               f(first + static_cast<Index>(i));
		               });
	}
#endif
#if defined(UFO_PAR_TBB)
	else if constexpr (execution::TBBBackend<T>) {
		oneapi::tbb::parallel_for(first, last, f);
	}
#endif
	else if constexpr (execution::OMPBackend<T>) {
		auto s_first = static_cast<std::make_signed_t<Index>>(first);
		auto s_last  = static_cast<std::make_signed_t<Index>>(last);
#pragma omp parallel for
		for (auto i = s_first; i < s_last; ++i) {
			f(static_cast<Index>(i));
		}
	} else {
		std::unreachable();
	}
}

/**
 * @brief Applies @p f to each element in `[first, last)` sequentially.
 *
 * A thin wrapper around `std::for_each` for non-integral iterator types.
 * The `requires(!std::integral<InputIt>)` constraint prevents this overload from
 * competing with the index-based overload when an integral type is passed.
 *
 * @tparam InputIt    An input iterator type (must not be an integral type).
 * @tparam UnaryFunc  A callable with signature compatible with `void(decltype(*first))`.
 *
 * @param first  Iterator to the first element.
 * @param last   Iterator one past the last element.
 * @param f      Function object to invoke for each dereferenced element.
 *
 * @return The (possibly moved) function object @p f after all invocations.
 */
template <class InputIt, class UnaryFunc>
  requires(!std::integral<InputIt>)
UnaryFunc for_each(InputIt first, InputIt last, UnaryFunc f)
{
	return std::for_each(first, last, f);
}

/**
 * @brief Applies @p f to each element in `[first, last)` using the given execution
 * policy.
 *
 * Dispatches to the appropriate parallel backend at compile time:
 * - **STL**: forwards directly to `std::for_each` with the translated STL policy.
 * - **GCD** (`UFO_PAR_GCD`): uses `dispatch_apply`; iterators must support
 *   random access (`first[i]`).
 * - **TBB** (`UFO_PAR_TBB`): uses `oneapi::tbb::parallel_for` over an index range
 *   mapped back to iterator offsets.
 * - **OMP**: uses an `#pragma omp parallel for` loop.
 *
 * The `requires(!std::integral<RandomIt>)` constraint prevents this overload from
 * competing with the index-based overload when an integral type is passed.
 *
 * @tparam ExecutionPolicy  A UFO execution policy type (see `execution.hpp`).
 * @tparam RandomIt         A random-access iterator type (must not be integral).
 * @tparam UnaryFunc        A callable with signature compatible with
 *                          `void(decltype(*first))`.
 *
 * @param policy  The execution policy controlling parallelism.
 * @param first   Iterator to the first element.
 * @param last    Iterator one past the last element.
 * @param f       Function object to invoke for each dereferenced element.
 */
template <execution::ExecutionPolicy T, class RandomIt, class UnaryFunc>
  requires(!std::integral<RandomIt>)
void for_each(T&& policy, RandomIt first, RandomIt last, UnaryFunc f)
{
	if constexpr (execution::STLBackend<T>) {
		std::for_each(execution::toSTL(std::forward<T>(policy)), first, last, f);
	}
#if defined(UFO_PAR_GCD)
	else if constexpr (execution::GCDBackend<T>) {
		std::size_t s = static_cast<std::size_t>(std::distance(first, last));
		dispatch_apply(s, dispatch_get_global_queue(0, 0), ^(std::size_t i) {
			f(first[i]);
		});
	}
#endif
#if defined(UFO_PAR_TBB)
	else if constexpr (execution::TBBBackend<T>) {
		// TODO: Benchmark against parallel_for_each
		std::size_t s = static_cast<std::size_t>(std::distance(first, last));
		oneapi::tbb::parallel_for(std::size_t{}, s,
		                          [first, f](std::size_t i) { f(first[i]); });
	}
#endif
	else if constexpr (execution::OMPBackend<T>) {
		auto s = static_cast<std::make_signed_t<std::size_t>>(
		    static_cast<std::size_t>(std::distance(first, last)));
#pragma omp parallel for
		for (auto i = static_cast<decltype(s)>(0); i < s; ++i) {
			f(first[i]);
		}
	} else {
		std::unreachable();
	}
}

/**************************************************************************************
|                                                                                     |
|                                      Transform                                      |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Applies @p unary_op to each element of `[first1, last1)` and writes the
 * results to `[d_first, ...)` sequentially.
 *
 * A thin wrapper around `std::transform`.
 *
 * @tparam InputIt   An input iterator type.
 * @tparam OutputIt  An output iterator type.
 * @tparam UnaryOp   A callable with signature compatible with
 *                   `OutputIt::value_type(InputIt::value_type)`.
 *
 * @param first1    Iterator to the first source element.
 * @param last1     Iterator one past the last source element.
 * @param d_first   Iterator to the first destination element.
 * @param unary_op  Unary transformation to apply to each source element.
 *
 * @return Iterator one past the last written destination element.
 */
template <class InputIt, class OutputIt, class UnaryOp>
OutputIt transform(InputIt first1, InputIt last1, OutputIt d_first, UnaryOp unary_op)
{
	return std::transform(first1, last1, d_first, unary_op);
}

/**
 * @brief Applies @p unary_op to each element of `[first1, last1)` and writes the
 * results to `[d_first, ...)` using the given execution policy.
 *
 * - **STL**: forwards to `std::transform` with the translated STL policy.
 * - **All other backends**: delegates to the parallel `for_each` over an index
 *   range, requiring random-access iterators.
 *
 * @tparam ExecutionPolicy  A UFO execution policy type (see `execution.hpp`).
 * @tparam RandomIt1        A random-access iterator type for the source range.
 * @tparam RandomIt2        A random-access iterator type for the destination range.
 * @tparam UnaryOp          A callable with signature compatible with
 *                          `RandomIt2::value_type(RandomIt1::value_type)`.
 *
 * @param policy    The execution policy controlling parallelism.
 * @param first1    Iterator to the first source element.
 * @param last1     Iterator one past the last source element.
 * @param d_first   Iterator to the first destination element.
 * @param unary_op  Unary transformation to apply to each source element.
 *
 * @return Iterator one past the last written destination element.
 */
template <execution::ExecutionPolicy T, class RandomIt1, class RandomIt2, class UnaryOp>
RandomIt2 transform(T&& policy, RandomIt1 first1, RandomIt1 last1, RandomIt2 d_first,
                    UnaryOp unary_op)
{
	if constexpr (execution::STLBackend<T>) {
		return std::transform(execution::toSTL(std::forward<T>(policy)), first1, last1,
		                      d_first, unary_op);
	} else {
		std::size_t s = static_cast<std::size_t>(std::distance(first1, last1));
		for_each(
		    std::forward<T>(policy), std::size_t{}, s,
		    [first1, d_first, unary_op](std::size_t i) { d_first[i] = unary_op(first1[i]); });

		return d_first + s;
	}
}

/**
 * @brief Applies @p binary_op to corresponding pairs of elements from
 * `[first1, last1)` and `[first2, ...)`, writing results to `[d_first, ...)`
 * sequentially.
 *
 * A thin wrapper around `std::transform`.
 *
 * @tparam InputIt1  An input iterator type for the first source range.
 * @tparam InputIt2  An input iterator type for the second source range.
 * @tparam OutputIt  An output iterator type for the destination range.
 * @tparam BinaryOp  A callable with signature compatible with
 *                   `OutputIt::value_type(InputIt1::value_type, InputIt2::value_type)`.
 *
 * @param first1     Iterator to the first element of the first source range.
 * @param last1      Iterator one past the last element of the first source range.
 * @param first2     Iterator to the first element of the second source range.
 * @param d_first    Iterator to the first destination element.
 * @param binary_op  Binary transformation applied to each pair of source elements.
 *
 * @return Iterator one past the last written destination element.
 */
template <class InputIt1, class InputIt2, class OutputIt, class BinaryOp>
OutputIt transform(InputIt1 first1, InputIt1 last1, InputIt2 first2, OutputIt d_first,
                   BinaryOp binary_op)
{
	return std::transform(first1, last1, first2, d_first, binary_op);
}

/**
 * @brief Applies @p binary_op to corresponding pairs of elements from
 * `[first1, last1)` and `[first2, ...)`, writing results to `[d_first, ...)`
 * using the given execution policy.
 *
 * - **STL**: forwards to `std::transform` with the translated STL policy.
 * - **All other backends**: delegates to the parallel `for_each` over an index
 *   range, requiring random-access iterators for all three ranges.
 *
 * @tparam ExecutionPolicy  A UFO execution policy type (see `execution.hpp`).
 * @tparam RandomIt1        A random-access iterator type for the first source range.
 * @tparam RandomIt2        A random-access iterator type for the second source range.
 * @tparam RandomIt3        A random-access iterator type for the destination range.
 * @tparam BinaryOp         A callable with signature compatible with
 *                          `RandomIt3::value_type(RandomIt1::value_type,
 *                          RandomIt2::value_type)`.
 *
 * @param policy     The execution policy controlling parallelism.
 * @param first1     Iterator to the first element of the first source range.
 * @param last1      Iterator one past the last element of the first source range.
 * @param first2     Iterator to the first element of the second source range.
 * @param d_first    Iterator to the first destination element.
 * @param binary_op  Binary transformation applied to each pair of source elements.
 *
 * @return Iterator one past the last written destination element.
 */
template <execution::ExecutionPolicy T, class RandomIt1, class RandomIt2, class RandomIt3,
          class BinaryOp>
RandomIt3 transform(T&& policy, RandomIt1 first1, RandomIt1 last1, RandomIt2 first2,
                    RandomIt3 d_first, BinaryOp binary_op)
{
	if constexpr (execution::STLBackend<T>) {
		return std::transform(execution::toSTL(std::forward<T>(policy)), first1, last1,
		                      first2, d_first, binary_op);
	} else {
		std::size_t s = static_cast<std::size_t>(std::distance(first1, last1));
		for_each(std::forward<T>(policy), std::size_t{}, s,
		         [first1, first2, d_first, binary_op](std::size_t i) {
			         d_first[i] = binary_op(first1[i], first2[i]);
		         });

		return d_first + s;
	}
}
}  // namespace ufo

#endif  // UFO_EXECUTION_ALGORITHM_HPP
