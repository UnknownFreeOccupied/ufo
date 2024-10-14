/*!
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

#ifndef UFO_UTILITY_EXECUTION_HPP
#define UFO_UTILITY_EXECUTION_HPP

// STL
#include <type_traits>
#if defined(UFO_TBB)
#include <execution>
#define UFO_TBB_SEQ       std::execution::seq,
#define UFO_TBB_PAR       std::execution::par,
#define UFO_TBB_PAR_UNSEQ std::execution::par_unseq,
#else
#define UFO_TBB_SEQ
#define UFO_TBB_PAR
#define UFO_TBB_PAR_UNSEQ
#endif

// OMP
#if defined(UFO_OMP)
#include <omp.h>
#endif

namespace ufo::execution
{
namespace detail
{
enum class ExecutionPolicy { NONE, SEQ, TBB, OMP, OFFLOAD };
}

// sequenced execution policy
struct sequenced_policy {
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::SEQ;
};

struct tbb_policy {
#if defined(UFO_TBB)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::TBB;
#else
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::NONE;
#endif
};

struct omp_policy {
#if defined(UFO_OMP)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::OMP;
#else
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::NONE;
#endif
};

// parallel execution policy
struct parallel_policy
    : tbb_policy
    , omp_policy {
#if defined(UFO_TBB)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::TBB;
#elif defined(UFO_OMP)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::OMP;
#else
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::NONE;
#endif
};

// offload execution policy
struct offload_policy {
#if defined(UFO_WEBGPU)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::OFFLOAD;
#else
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::NONE;
#endif
};

template <class T>
struct is_execution_policy
    : std::conditional_t<
          detail::ExecutionPolicy::NONE != std::decay_t<T>::policy,
          std::disjunction<std::is_same<execution::sequenced_policy, std::decay_t<T>>,
                           std::is_same<execution::tbb_policy, std::decay_t<T>>,
                           std::is_same<execution::omp_policy, std::decay_t<T>>,
                           std::is_same<execution::parallel_policy, std::decay_t<T>>,
                           std::is_same<execution::offload_policy, std::decay_t<T>>>,
          std::false_type>

{
};

template <class T>
constexpr inline bool is_execution_policy_v = is_execution_policy<T>::value;

template <class T>
constexpr inline bool is_seq_v = detail::ExecutionPolicy::SEQ == std::decay_t<T>::policy;

template <class T>
constexpr inline bool is_tbb_v = detail::ExecutionPolicy::TBB == std::decay_t<T>::policy;

template <class T>
constexpr inline bool is_omp_v = detail::ExecutionPolicy::OMP == std::decay_t<T>::policy;

template <class T>
constexpr inline bool is_offload_v =
    detail::ExecutionPolicy::OFFLOAD == std::decay_t<T>::policy;

constexpr inline sequenced_policy seq{};
constexpr inline tbb_policy       tbb{};
constexpr inline omp_policy       omp{};
constexpr inline parallel_policy  par{};
constexpr inline offload_policy   offload{};
}  // namespace ufo::execution

#endif  // UFO_UTILITY_EXECUTION_HPP