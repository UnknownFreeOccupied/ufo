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
#include <atomic>
#include <concepts>
#include <functional>
#include <iterator>
#include <mutex>
#include <ranges>
#include <utility>

namespace ufo
{
using execution::ExecutionPolicy;
/**************************************************************************************
|                                                                                     |
|                                      For each                                       |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Applies `f` to each integer index in `[first, last)` sequentially.
 * @tparam Index      An integral type used as the loop index.
 * @tparam UnaryFunc  A callable with signature compatible with `void(Index)`.
 * @param [in] first  First index in the half-open range (inclusive).
 * @param [in] last   One past the last index (exclusive).
 * @param [in] f      Function object to invoke for each index.
 * @return The (possibly moved) function object `f` after all invocations.
 * @details
 * A convenience overload that avoids manually constructing an iterator range when
 * iterating over a contiguous index space (e.g., element indices of a flat array).
 */
template <std::integral Index, class UnaryFunc, class Proj = std::identity>
constexpr UnaryFunc for_each(Index first, Index last, UnaryFunc f, Proj proj = {})
{
	for (; last != first; ++first) {
		std::invoke(f, std::invoke(proj, first));
	}
	return f;
}

/**
 * @brief Applies `f` to each integer index in `[first, last)` using the given
 * execution policy.
 * @tparam ExecutionPolicy  A UFO execution policy type (see `execution.hpp`).
 * @tparam Index            An integral type used as the loop index.
 * @tparam UnaryFunc        A callable with signature compatible with `void(Index)`.
 * @param [in] policy  The execution policy controlling parallelism.
 * @param [in] first   First index in the half-open range (inclusive).
 * @param [in] last    One past the last index (exclusive).
 * @param [in] f       Function object to invoke for each index.
 * @details
 * Dispatches to the appropriate parallel backend at compile time:
 * - **STL**: wraps the range in an `IndexIterator` and forwards to
 *   `std::for_each` with the translated STL execution policy.
 * - **GCD** (`UFO_PAR_GCD`): uses `dispatch_apply` on the global concurrent queue.
 * - **TBB** (`UFO_PAR_TBB`): uses `oneapi::tbb::parallel_for`.
 * - **OMP**: uses an `#pragma omp parallel for` loop.
 */
template <ufo::execution::ExecutionPolicy T, std::integral Index, class UnaryFunc,
          class Proj = std::identity>
void for_each(T&& policy, Index first, Index last, UnaryFunc f, Proj proj = {})
{
	if constexpr (execution::STLBackend<T>) {
		auto r = std::views::iota(first, last) | std::views::transform(std::move(proj));
		std::for_each(execution::toSTL(std::forward<T>(policy)), std::ranges::begin(r),
		              std::ranges::end(r), f);
	}
#if defined(UFO_PAR_GCD)
	else if constexpr (execution::GCDBackend<T>) {
		dispatch_apply(static_cast<std::size_t>(last - first),
		               dispatch_get_global_queue(0, 0), ^(std::size_t i) {
			               std::invoke(f, std::invoke(proj, first + static_cast<Index>(i)));
		               });
	}
#endif
#if defined(UFO_PAR_TBB)
	else if constexpr (execution::TBBBackend<T>) {
		oneapi::tbb::parallel_for(
		    first, last, [f, proj](Index i) { std::invoke(f, std::invoke(proj, i)); });
	}
#endif
	else if constexpr (execution::OMPBackend<T>) {
		auto s_first = static_cast<std::make_signed_t<Index>>(first);
		auto s_last  = static_cast<std::make_signed_t<Index>>(last);
#pragma omp parallel for
		for (auto i = s_first; i < s_last; ++i) {
			std::invoke(f, std::invoke(proj, static_cast<Index>(i)));
		}
	} else {
		std::unreachable();
	}
}

/**
 * @brief Applies `f` to each element in `[first, last)` sequentially.
 * @tparam InputIt    An input iterator type (must not be an integral type).
 * @tparam UnaryFunc  A callable with signature compatible with `void(decltype(*first))`.
 * @param [in] first  Iterator to the first element.
 * @param [in] last   Iterator one past the last element.
 * @param [in] f      Function object to invoke for each dereferenced element.
 * @return The (possibly moved) function object `f` after all invocations.
 * @details
 * A thin wrapper around `std::for_each` for non-integral iterator types.
 */
template <std::input_iterator InputIt, class UnaryFunc, class Proj = std::identity>
constexpr UnaryFunc for_each(InputIt first, InputIt last, UnaryFunc f, Proj proj = {})
{
	for (; first != last; ++first) {
		std::invoke(f, std::invoke(proj, *first));
	}
	return f;
}

/**
 * @brief Applies `f` to each element in `[first, last)` using the given execution
 * policy.
 * @tparam ExecutionPolicy  A UFO execution policy type (see `execution.hpp`).
 * @tparam RandomIt         A random-access iterator type (must not be integral).
 * @tparam UnaryFunc        A callable with signature compatible with
 *                          `void(decltype(*first))`.
 * @param [in] policy  The execution policy controlling parallelism.
 * @param [in] first   Iterator to the first element.
 * @param [in] last    Iterator one past the last element.
 * @param [in] f       Function object to invoke for each dereferenced element.
 * @details
 * Dispatches to the appropriate parallel backend at compile time:
 * - **STL**: forwards directly to `std::for_each` with the translated STL policy.
 * - **GCD** (`UFO_PAR_GCD`): uses `dispatch_apply`; iterators must support
 *   random access (`first[i]`).
 * - **TBB** (`UFO_PAR_TBB`): uses `oneapi::tbb::parallel_for` over an index range
 *   mapped back to iterator offsets.
 * - **OMP**: uses an `#pragma omp parallel for` loop.
 */
template <ufo::execution::ExecutionPolicy T, std::random_access_iterator RandomIt,
          class UnaryFunc, class Proj = std::identity>
void for_each(T&& policy, RandomIt first, RandomIt last, UnaryFunc f, Proj proj = {})
{
	if constexpr (execution::STLBackend<T>) {
		for_each(
		    std::forward<T>(policy), std::size_t{},
		    static_cast<std::size_t>(std::distance(first, last)),
		    [first, f, proj](std::size_t i) { std::invoke(f, std::invoke(proj, first[i])); });
	}
#if defined(UFO_PAR_GCD)
	else if constexpr (execution::GCDBackend<T>) {
		std::size_t s = static_cast<std::size_t>(std::distance(first, last));
		dispatch_apply(s, dispatch_get_global_queue(0, 0), ^(std::size_t i) {
			std::invoke(f, std::invoke(proj, first[i]));
		});
	}
#endif
#if defined(UFO_PAR_TBB)
	else if constexpr (execution::TBBBackend<T>) {
		// TODO: Benchmark against parallel_for_each
		std::size_t s = static_cast<std::size_t>(std::distance(first, last));
		oneapi::tbb::parallel_for(std::size_t{}, s, [first, f, proj](std::size_t i) {
			std::invoke(f, std::invoke(proj, first[i]));
		});
	}
#endif
	else if constexpr (execution::OMPBackend<T>) {
		auto s = static_cast<std::make_signed_t<std::size_t>>(std::distance(first, last));
#pragma omp parallel for
		for (auto i = static_cast<decltype(s)>(0); i < s; ++i) {
			std::invoke(f, std::invoke(proj, first[i]));
		}
	} else {
		std::unreachable();
	}
}

/**
 * @brief Applies `f` to each element in the range `r` sequentially.
 * @tparam Range      A range type.
 * @tparam UnaryFunc  A callable with signature compatible with
 *                    `void(std::ranges::range_reference_t<Range>)`.
 * @param [in] r  The range of elements to iterate over.
 * @param [in] f  Function object to invoke for each element.
 * @return The (possibly moved) function object `f` after all invocations.
 */
template <std::ranges::input_range Range, class UnaryFunc, class Proj = std::identity>
  requires(!ufo::execution::ExecutionPolicy<std::remove_cvref_t<Range>>)
constexpr std::ranges::for_each_result<std::ranges::iterator_t<Range>, UnaryFunc>
for_each(Range&& r, UnaryFunc f, Proj proj = {})
{
	return std::ranges::for_each(std::forward<Range>(r), std::move(f), std::move(proj));
}

/**
 * @brief Applies `f` to each element in the range `r` using the given execution policy.
 * @tparam ExecutionPolicy  A UFO execution policy type.
 * @tparam Range            A random-access range type.
 * @tparam UnaryFunc        A callable with signature compatible with
 *                          `void(std::ranges::range_reference_t<Range>)`.
 * @param [in] policy  The execution policy controlling parallelism.
 * @param [in] r       The range of elements to iterate over.
 * @param [in] f       Function object to invoke for each element.
 */
template <ufo::execution::ExecutionPolicy T, std::ranges::random_access_range Range,
          class UnaryFunc, class Proj = std::identity>
void for_each(T&& policy, Range&& r, UnaryFunc f, Proj proj = {})
{
	ufo::for_each(std::forward<T>(policy), std::ranges::begin(r), std::ranges::end(r),
	              std::move(f), std::move(proj));
}

/**************************************************************************************
|                                                                                     |
|                                      Transform                                      |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Applies `unary_op` to each element of `[first1, last1)` and writes the
 * results to `[d_first, ...)` sequentially.
 * @tparam InputIt   An input iterator type.
 * @tparam OutputIt  An output iterator type.
 * @tparam UnaryOp   A callable with signature compatible with
 *                   `OutputIt::value_type(InputIt::value_type)`.
 * @param [in] first1    Iterator to the first source element.
 * @param [in] last1     Iterator one past the last source element.
 * @param [out] d_first  Iterator to the first destination element.
 * @param [in] unary_op  Unary transformation to apply to each source element.
 * @return Iterator one past the last written destination element.
 */
template <std::input_iterator InputIt, class OutputIt, class UnaryOp,
          class Proj = std::identity>
constexpr OutputIt transform(InputIt first1, InputIt last1, OutputIt d_first,
                             UnaryOp unary_op, Proj proj = {})
{
	for (; first1 != last1; ++first1, ++d_first) {
		*d_first = std::invoke(unary_op, std::invoke(proj, *first1));
	}
	return d_first;
}

/**
 * @brief Applies `unary_op` to each element of `[first1, last1)` and writes the
 * results to `[d_first, ...)` using the given execution policy.
 * @tparam ExecutionPolicy  A UFO execution policy type (see `execution.hpp`).
 * @tparam RandomIt1        A random-access iterator type for the source range.
 * @tparam RandomIt2        A random-access iterator type for the destination range.
 * @tparam UnaryOp          A callable with signature compatible with
 *                          `RandomIt2::value_type(RandomIt1::value_type)`.
 * @param [in] policy    The execution policy controlling parallelism.
 * @param [in] first1    Iterator to the first source element.
 * @param [in] last1     Iterator one past the last source element.
 * @param [out] d_first   Iterator to the first destination element.
 * @param [in] unary_op  Unary transformation to apply to each source element.
 * @return Iterator one past the last written destination element.
 */
template <ufo::execution::ExecutionPolicy T, std::random_access_iterator RandomIt1,
          std::random_access_iterator RandomIt2, class UnaryOp,
          class Proj = std::identity>
  requires(!std::input_or_output_iterator<UnaryOp>)

RandomIt2 transform(T&& policy, RandomIt1 first1, RandomIt1 last1, RandomIt2 d_first,
                    UnaryOp unary_op, Proj proj = {})
{
	if constexpr (execution::STLBackend<T>) {
		std::transform(execution::toSTL(std::forward<T>(policy)), first1, last1, d_first,
		               [&unary_op, &proj](auto&& x) {
			               return std::invoke(unary_op,
			                                  std::invoke(proj, std::forward<decltype(x)>(x)));
		               });
		return d_first + std::distance(first1, last1);
	} else {
		std::size_t s = static_cast<std::size_t>(std::distance(first1, last1));
		for_each(std::forward<T>(policy), std::size_t{}, s,
		         [first1, d_first, unary_op, proj](std::size_t i) {
			         d_first[i] = std::invoke(unary_op, std::invoke(proj, first1[i]));
		         });

		return d_first + s;
	}
}

/**
 * @brief Applies `unary_op` to each element in range `r1` and writes the results to
 * `d_first` sequentially.
 * @tparam Range1   A source range type.
 * @tparam OutputIt  An output iterator type.
 * @tparam UnaryOp   A callable with signature compatible with
 *                   `OutputIt::value_type(std::ranges::range_reference_t<Range1>)`.
 * @param [in] r1        The source range.
 * @param [out] d_first   Iterator to the first destination element.
 * @param [in] unary_op  Unary transformation to apply to each source element.
 * @return Iterator one past the last written destination element.
 */
template <std::ranges::input_range Range1, std::weakly_incrementable OutputIt,
          class UnaryOp, class Proj = std::identity>
  requires(
      !ufo::execution::ExecutionPolicy<std::remove_cvref_t<Range1>> &&
      std::indirectly_writable<
          OutputIt, std::indirect_result_t<
                        UnaryOp, std::projected<std::ranges::iterator_t<Range1>, Proj>>>)
constexpr std::ranges::unary_transform_result<std::ranges::iterator_t<Range1>, OutputIt>
transform(Range1&& r1, OutputIt d_first, UnaryOp unary_op, Proj proj = {})
{
	return std::ranges::transform(std::forward<Range1>(r1), d_first, unary_op,
	                              std::move(proj));
}

/**
 * @brief Applies `unary_op` to each element in range `r1` and writes the results to
 * `d_first` using the given execution policy.
 * @tparam ExecutionPolicy  A UFO execution policy type.
 * @tparam Range1           A random-access source range type.
 * @tparam RandomIt2        A random-access destination iterator type.
 * @tparam UnaryOp          A callable with signature compatible with
 *                          `RandomIt2::value_type(std::ranges::range_reference_t<Range1>)`.
 * @param [in] policy    The execution policy controlling parallelism.
 * @param [in] r1        The source range.
 * @param [out] d_first   Iterator to the first destination element.
 * @param [in] unary_op  Unary transformation to apply to each source element.
 * @return Iterator one past the last written destination element.
 */
template <ufo::execution::ExecutionPolicy T, std::ranges::random_access_range Range1,
          std::random_access_iterator RandomIt2, class UnaryOp,
          class Proj = std::identity>
  requires(
      std::indirectly_writable<
          RandomIt2, std::indirect_result_t<
                         UnaryOp, std::projected<std::ranges::iterator_t<Range1>, Proj>>>)
RandomIt2 transform(T&& policy, Range1&& r1, RandomIt2 d_first, UnaryOp unary_op,
                    Proj proj = {})
{
	return ufo::transform(std::forward<T>(policy), std::ranges::begin(r1),
	                      std::ranges::end(r1), d_first, std::move(unary_op),
	                      std::move(proj));
}

/**
 * @brief Applies `binary_op` to corresponding pairs of elements from
 * `[first1, last1)` and `[first2, ...)`, writing results to `[d_first, ...)`
 * sequentially.
 * @tparam InputIt1  An input iterator type for the first source range.
 * @tparam InputIt2  An input iterator type for the second source range.
 * @tparam OutputIt  An output iterator type for the destination range.
 * @tparam BinaryOp  A callable with signature compatible with
 *                   `OutputIt::value_type(InputIt1::value_type, InputIt2::value_type)`.
 * @param [in] first1     Iterator to the first element of the first source range.
 * @param [in] last1      Iterator one past the last element of the first source range.
 * @param [in] first2     Iterator to the first element of the second source range.
 * @param [out] d_first    Iterator to the first destination element.
 * @param [in] binary_op  Binary transformation applied to each pair of source elements.
 * @return Iterator one past the last written destination element.
 */
template <std::input_iterator InputIt1, std::input_iterator InputIt2, class OutputIt,
          class BinaryOp, class Proj1 = std::identity, class Proj2 = std::identity>
constexpr OutputIt transform(InputIt1 first1, InputIt1 last1, InputIt2 first2,
                             OutputIt d_first, BinaryOp binary_op, Proj1 proj1 = {},
                             Proj2 proj2 = {})
{
	for (; first1 != last1; ++first1, ++first2, ++d_first) {
		*d_first =
		    std::invoke(binary_op, std::invoke(proj1, *first1), std::invoke(proj2, *first2));
	}
	return d_first;
}

/**
 * @brief Applies `binary_op` to corresponding pairs of elements from
 * `[first1, last1)` and `[first2, ...)`, writing results to `[d_first, ...)`
 * using the given execution policy.
 * @tparam ExecutionPolicy  A UFO execution policy type (see `execution.hpp`).
 * @tparam RandomIt1        A random-access iterator type for the first source range.
 * @tparam RandomIt2        A random-access iterator type for the second source range.
 * @tparam RandomIt3        A random-access iterator type for the destination range.
 * @tparam BinaryOp         A callable with signature compatible with
 *                          `RandomIt3::value_type(RandomIt1::value_type,
 *                          RandomIt2::value_type)`.
 * @param [in] policy     The execution policy controlling parallelism.
 * @param [in] first1     Iterator to the first element of the first source range.
 * @param [in] last1      Iterator one past the last element of the first source range.
 * @param [in] first2     Iterator to the first element of the second source range.
 * @param [out] d_first    Iterator to the first destination element.
 * @param [in] binary_op  Binary transformation applied to each pair of source elements.
 * @return Iterator one past the last written destination element.
 */
template <ufo::execution::ExecutionPolicy T, std::random_access_iterator RandomIt1,
          std::random_access_iterator RandomIt2, std::random_access_iterator RandomIt3,
          class BinaryOp, class Proj1 = std::identity, class Proj2 = std::identity>
RandomIt3 transform(T&& policy, RandomIt1 first1, RandomIt1 last1, RandomIt2 first2,
                    RandomIt3 d_first, BinaryOp binary_op, Proj1 proj1 = {},
                    Proj2 proj2 = {})
{
	if constexpr (execution::STLBackend<T>) {
		std::transform(execution::toSTL(std::forward<T>(policy)), first1, last1, first2,
		               d_first, [&binary_op, &proj1, &proj2](auto&& x, auto&& y) {
			               return std::invoke(binary_op,
			                                  std::invoke(proj1, std::forward<decltype(x)>(x)),
			                                  std::invoke(proj2, std::forward<decltype(y)>(y)));
		               });
		return d_first + std::distance(first1, last1);
	} else {
		std::size_t s = static_cast<std::size_t>(std::distance(first1, last1));
		for_each(std::forward<T>(policy), std::size_t{}, s,
		         [first1, first2, d_first, binary_op, proj1, proj2](std::size_t i) {
			         d_first[i] = std::invoke(binary_op, std::invoke(proj1, first1[i]),
			                                  std::invoke(proj2, first2[i]));
		         });

		return d_first + s;
	}
}

/**
 * @brief Applies `binary_op` to corresponding pairs of elements from range `r1` and
 * `first2`, writing results to `d_first` sequentially.
 * @tparam Range1    A source range type.
 * @tparam InputIt2  An input iterator type for the second source range.
 * @tparam OutputIt  An output iterator type for the destination range.
 * @tparam BinaryOp  A callable with signature compatible with
 *                   `OutputIt::value_type(std::ranges::range_reference_t<Range1>,
 *                   InputIt2::value_type)`.
 * @param [in] r1         The first source range.
 * @param [in] first2     Iterator to the first element of the second source range.
 * @param [out] d_first    Iterator to the first destination element.
 * @param [in] binary_op  Binary transformation applied to each pair of source elements.
 * @return Iterator one past the last written destination element.
 */
template <std::ranges::input_range Range1, std::input_iterator InputIt2,
          std::weakly_incrementable OutputIt, class BinaryOp, class Proj1 = std::identity,
          class Proj2 = std::identity>
  requires(
      !ufo::execution::ExecutionPolicy<std::remove_cvref_t<Range1>> &&
      std::indirectly_writable<
          OutputIt, std::indirect_result_t<
                        BinaryOp, std::projected<std::ranges::iterator_t<Range1>, Proj1>,
                        std::projected<InputIt2, Proj2>>>)
constexpr std::ranges::binary_transform_result<std::ranges::iterator_t<Range1>, InputIt2,
                                               OutputIt>
transform(Range1&& r1, InputIt2 first2, OutputIt d_first, BinaryOp binary_op,
          Proj1 proj1 = {}, Proj2 proj2 = {})
{
	// std::ranges::transform for binary expects a range for the second input too.
	// But UFOMap provides an iterator. We can wrap it in an unbounded view if we know it's
	// safe, or just implement it manually. Actually, std::ranges::transform (binary)
	// doesn't have an iterator-based overload for the second range. So we implement it
	// manually to maintain the same interface.
	auto first1 = std::ranges::begin(r1);
	auto last1  = std::ranges::end(r1);
	for (; first1 != last1; ++first1, ++first2, ++d_first) {
		*d_first =
		    std::invoke(binary_op, std::invoke(proj1, *first1), std::invoke(proj2, *first2));
	}
	return {std::move(first1), std::move(first2), std::move(d_first)};
}

/**
 * @brief Applies `binary_op` to corresponding pairs of elements from range `r1` and
 * `first2`, writing results to `d_first` using the given execution policy.
 * @tparam ExecutionPolicy  A UFO execution policy type.
 * @tparam Range1           A random-access source range type.
 * @tparam RandomIt2        A random-access second source iterator type.
 * @tparam RandomIt3        A random-access destination iterator type.
 * @tparam BinaryOp         A callable with signature compatible with
 *                          `RandomIt3::value_type(std::ranges::range_reference_t<Range1>,
 *                          RandomIt2::value_type)`.
 * @param [in] policy     The execution policy controlling parallelism.
 * @param [in] r1         The first source range.
 * @param [in] first2     Iterator to the first element of the second source range.
 * @param [out] d_first    Iterator to the first destination element.
 * @param [in] binary_op  Binary transformation applied to each pair of source elements.
 * @return Iterator one past the last written destination element.
 */
template <ufo::execution::ExecutionPolicy T, std::ranges::random_access_range Range1,
          std::random_access_iterator RandomIt2, std::random_access_iterator RandomIt3,
          class BinaryOp, class Proj1 = std::identity, class Proj2 = std::identity>
  requires(
      std::indirectly_writable<
          RandomIt3, std::indirect_result_t<
                         BinaryOp, std::projected<std::ranges::iterator_t<Range1>, Proj1>,
                         std::projected<RandomIt2, Proj2>>>)
RandomIt3 transform(T&& policy, Range1&& r1, RandomIt2 first2, RandomIt3 d_first,
                    BinaryOp binary_op, Proj1 proj1 = {}, Proj2 proj2 = {})
{
	return ufo::transform(std::forward<T>(policy), std::ranges::begin(r1),
	                      std::ranges::end(r1), first2, d_first, std::move(binary_op),
	                      std::move(proj1), std::move(proj2));
}
/**************************************************************************************
|                                                                                     |
|                                       Reduce                                        |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Reduces the range `[first, last)` using `init` and the given execution
 * policy.
 * @tparam ExecutionPolicy  A UFO execution policy type.
 * @tparam InputIt          An input iterator type.
 * @tparam T                The type of the initial value and result.
 * @param [in] policy  The execution policy controlling parallelism.
 * @param [in] first   Iterator to the first element.
 * @param [in] last    Iterator one past the last element.
 * @param [in] init    Initial value for the reduction.
 * @return The reduced value.
 */
template <ufo::execution::ExecutionPolicy P, std::input_iterator InputIt, class T>
[[nodiscard]] T reduce(P&& policy, InputIt first, InputIt last, T init)
{
	if constexpr (execution::STLBackend<P>) {
		return std::reduce(execution::toSTL(std::forward<P>(policy)), first, last, init);
	} else {
		// Generic implementation using for_each
		// For parallel backends, we use a simple approach here for now.
		// A more efficient implementation would use backend-specific features.
		std::mutex m;
		for_each(std::forward<P>(policy), first, last, [&](auto const& x) {
			std::lock_guard lock(m);
			init = init + x;
		});
		return init;
	}
}

/**
 * @brief Reduces the range `r` using `init` and the given execution policy.
 * @tparam ExecutionPolicy  A UFO execution policy type.
 * @tparam Range            A range type.
 * @tparam T                The type of the initial value and result.
 * @param [in] policy  The execution policy controlling parallelism.
 * @param [in] r       The range to reduce.
 * @param [in] init    Initial value for the reduction.
 * @return The reduced value.
 */
template <ufo::execution::ExecutionPolicy P, std::ranges::input_range Range, class T>
[[nodiscard]] T reduce(P&& policy, Range&& r, T init)
{
	return reduce(std::forward<P>(policy), std::ranges::begin(r), std::ranges::end(r),
	              std::move(init));
}

/**************************************************************************************
|                                                                                     |
|                                     Predicates                                      |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Checks if any element in `[first, last)` satisfies `p` using the given
 * execution policy.
 * @tparam ExecutionPolicy  A UFO execution policy type.
 * @tparam InputIt          An input iterator type.
 * @tparam UnaryPred        A unary predicate type.
 * @param [in] policy  The execution policy controlling parallelism.
 * @param [in] first   Iterator to the first element.
 * @param [in] last    Iterator one past the last element.
 * @param [in] p       Predicate to satisfy.
 * @return `true` if any element satisfies `p`, `false` otherwise.
 */
template <ufo::execution::ExecutionPolicy P, std::input_iterator InputIt, class UnaryPred>
[[nodiscard]] bool any_of(P&& policy, InputIt first, InputIt last, UnaryPred p)
{
	if constexpr (execution::STLBackend<P>) {
		return std::any_of(execution::toSTL(std::forward<P>(policy)), first, last, p);
	} else {
		std::atomic<bool> found = false;
		for_each(std::forward<P>(policy), first, last, [&](auto const& x) {
			if (p(x)) {
				found = true;
			}
		});
		return found;
	}
}

/**
 * @brief Checks if any element in range `r` satisfies `p` using the given
 * execution policy.
 * @tparam ExecutionPolicy  A UFO execution policy type.
 * @tparam Range            A range type.
 * @tparam UnaryPred        A unary predicate type.
 * @param [in] policy  The execution policy controlling parallelism.
 * @param [in] r       The range of elements.
 * @param [in] p       Predicate to satisfy.
 * @return `true` if any element satisfies `p`, `false` otherwise.
 */
template <ufo::execution::ExecutionPolicy P, std::ranges::input_range Range,
          class UnaryPred>
[[nodiscard]] bool any_of(P&& policy, Range&& r, UnaryPred p)
{
	return any_of(std::forward<P>(policy), std::ranges::begin(r), std::ranges::end(r),
	              std::move(p));
}

/**
 * @brief Checks if all elements in `[first, last)` satisfy `p` using the given
 * execution policy.
 * @tparam ExecutionPolicy  A UFO execution policy type.
 * @tparam InputIt          An input iterator type.
 * @tparam UnaryPred        A unary predicate type.
 * @param [in] policy  The execution policy controlling parallelism.
 * @param [in] first   Iterator to the first element.
 * @param [in] last    Iterator one past the last element.
 * @param [in] p       Predicate to satisfy.
 * @return `true` if all elements satisfy `p`, `false` otherwise.
 */
template <ufo::execution::ExecutionPolicy P, std::input_iterator InputIt, class UnaryPred>
[[nodiscard]] bool all_of(P&& policy, InputIt first, InputIt last, UnaryPred p)
{
	return !any_of(std::forward<P>(policy), first, last,
	               [&p](auto const& x) { return !p(x); });
}

/**
 * @brief Checks if all elements in range `r` satisfy `p` using the given
 * execution policy.
 * @tparam ExecutionPolicy  A UFO execution policy type.
 * @tparam Range            A range type.
 * @tparam UnaryPred        A unary predicate type.
 * @param [in] policy  The execution policy controlling parallelism.
 * @param [in] r       The range of elements.
 * @param [in] p       Predicate to satisfy.
 * @return `true` if all elements satisfy `p`, `false` otherwise.
 */
template <ufo::execution::ExecutionPolicy P, std::ranges::input_range Range,
          class UnaryPred>
[[nodiscard]] bool all_of(P&& policy, Range&& r, UnaryPred p)
{
	return all_of(std::forward<P>(policy), std::ranges::begin(r), std::ranges::end(r),
	              std::move(p));
}

/**************************************************************************************
|                                                                                     |
|                                      Searching                                      |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Finds the first element in `[first, last)` satisfying `p` using the given
 * execution policy.
 * @tparam ExecutionPolicy  A UFO execution policy type.
 * @tparam RandomIt         A random-access iterator type.
 * @tparam UnaryPred        A unary predicate type.
 * @param [in] policy  The execution policy controlling parallelism.
 * @param [in] first   Iterator to the first element.
 * @param [in] last    Iterator one past the last element.
 * @param [in] p       Predicate to satisfy.
 * @return Iterator to the first element that satisfies `p`, or `last` if no such element
 * is found.
 */
template <ufo::execution::ExecutionPolicy P, std::random_access_iterator RandomIt,
          class UnaryPred>
[[nodiscard]] RandomIt find_if(P&& policy, RandomIt first, RandomIt last, UnaryPred p)
{
	if constexpr (execution::STLBackend<P>) {
		return std::find_if(execution::toSTL(std::forward<P>(policy)), first, last, p);
	} else {
		std::atomic<std::size_t> min_idx =
		    static_cast<std::size_t>(std::distance(first, last));
		for_each(std::forward<P>(policy), std::size_t{}, min_idx.load(), [&](std::size_t i) {
			if (p(first[i])) {
				std::size_t expected = min_idx.load();
				while (i < expected && !min_idx.compare_exchange_weak(expected, i)) {
					// Keep trying
				}
			}
		});
		return first + min_idx.load();
	}
}

/**
 * @brief Finds the first element in range `r` satisfying `p` using the given
 * execution policy.
 * @tparam ExecutionPolicy  A UFO execution policy type.
 * @tparam Range            A random-access range type.
 * @tparam UnaryPred        A unary predicate type.
 * @param [in] policy  The execution policy controlling parallelism.
 * @param [in] r       The range to search.
 * @param [in] p       Predicate to satisfy.
 * @return Iterator to the first element that satisfies `p`, or `std::ranges::end(r)` if
 * no such element is found.
 */
template <ufo::execution::ExecutionPolicy P, std::ranges::random_access_range Range,
          class UnaryPred>
[[nodiscard]] auto find_if(P&& policy, Range&& r, UnaryPred p)
{
	return find_if(std::forward<P>(policy), std::ranges::begin(r), std::ranges::end(r),
	               std::move(p));
}

/**************************************************************************************
|                                                                                     |
|                                      Counting                                       |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Counts elements in `[first, last)` satisfying `p` using the given
 * execution policy.
 * @tparam ExecutionPolicy  A UFO execution policy type.
 * @tparam InputIt          An input iterator type.
 * @tparam UnaryPred        A unary predicate type.
 * @param [in] policy  The execution policy controlling parallelism.
 * @param [in] first   Iterator to the first element.
 * @param [in] last    Iterator one past the last element.
 * @param [in] p       Predicate to satisfy.
 * @return Number of elements satisfying `p`.
 */
template <ufo::execution::ExecutionPolicy P, std::input_iterator InputIt, class UnaryPred>
[[nodiscard]] std::size_t count_if(P&& policy, InputIt first, InputIt last, UnaryPred p)
{
	if constexpr (execution::STLBackend<P>) {
		return std::count_if(execution::toSTL(std::forward<P>(policy)), first, last, p);
	} else {
		std::atomic<std::size_t> count = 0;
		for_each(std::forward<P>(policy), first, last, [&](auto const& x) {
			if (p(x)) {
				++count;
			}
		});
		return count.load();
	}
}

/**
 * @brief Counts elements in range `r` satisfying `p` using the given
 * execution policy.
 * @tparam ExecutionPolicy  A UFO execution policy type.
 * @tparam Range            A range type.
 * @tparam UnaryPred        A unary predicate type.
 * @param [in] policy  The execution policy controlling parallelism.
 * @param [in] r       The range of elements.
 * @param [in] p       Predicate to satisfy.
 * @return Number of elements satisfying `p`.
 */
template <ufo::execution::ExecutionPolicy P, std::ranges::input_range Range,
          class UnaryPred>
[[nodiscard]] std::size_t count_if(P&& policy, Range&& r, UnaryPred p)
{
	return count_if(std::forward<P>(policy), std::ranges::begin(r), std::ranges::end(r),
	                std::move(p));
}

}  // namespace ufo

#endif  // UFO_EXECUTION_ALGORITHM_HPP
