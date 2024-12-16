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

#ifndef UFO_EXECUTION_EXECUTION_HPP
#define UFO_EXECUTION_EXECUTION_HPP

// STL
#include <type_traits>

// TODO: Check if UFO_PAR_STL, UFO_PAR_TBB, UFO_PAR_GCD, UFO_PAR_OMP Something for Window?

#if defined(UFO_PAR_STL)
#include <execution>
#define UFO_PAR_STL_SEQ       std::execution::seq,
#define UFO_PAR_STL_PAR       std::execution::par,
#define UFO_PAR_STL_PAR_UNSEQ std::execution::par_unseq,
#if __cplusplus >= 202002L
#define UFO_PAR_STL_UNSEQ std::execution::unseq,
#else
#define UFO_PAR_STL_UNSEQ std::execution::seq,
#endif
#else
#define UFO_PAR_STL_SEQ
#define UFO_PAR_STL_PAR
#define UFO_PAR_STL_PAR_UNSEQ
#define UFO_PAR_STL_UNSEQ
#endif

// GCD (Grand Central Dispatch)
#if defined(UFO_PAR_GCD)
#include <dispatch/dispatch.h>
#endif

// oneTBB (Threading Building Blocks)
#if defined(UFO_PAR_TBB)
// TODO: Add CMake check?
// #include <oneapi/tbb.h>
#endif

// OMP (OpenMP)
#if defined(UFO_PAR_OMP)
#include <omp.h>
#endif

namespace ufo::execution
{
namespace detail
{
enum class ExecutionPolicy {
	NONE,
	SEQ,
	UNSEQ,
	PAR,
	PAR_UNSEQ,
	GCD,
	GCD_UNSEQ,
	TBB,
	TBB_UNSEQ,
	OMP,
	OMP_UNSEQ
};
}

struct sequenced_policy {
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::SEQ;
};

struct unsequenced_policy {
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::UNSEQ;
};

struct gcd_policy {
#if defined(UFO_PAR_GCD)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::GCD;
#else
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::NONE;
#endif
};

struct gcd_unsequenced_policy {
#if defined(UFO_PAR_GCD)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::GCD_UNSEQ;
#else
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::NONE;
#endif
};

struct tbb_policy {
#if defined(UFO_PAR_TBB)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::TBB;
#else
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::NONE;
#endif
};

struct tbb_unsequenced_policy {
#if defined(UFO_PAR_TBB)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::TBB_UNSEQ;
#else
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::NONE;
#endif
};

struct omp_policy {
#if defined(UFO_PAR_OMP)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::OMP;
#else
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::NONE;
#endif
};

struct omp_unsequenced_policy {
#if defined(UFO_PAR_OMP)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::OMP_UNSEQ;
#else
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::NONE;
#endif
};

struct parallel_policy
    : gcd_policy
    , tbb_policy
    , omp_policy {
#if defined(UFO_PAR_STL)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::PAR;
#elif defined(UFO_PAR_GCD)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::GCD;
#elif defined(UFO_PAR_TBB)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::TBB;
#elif defined(UFO_PAR_OMP)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::OMP;
#else
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::NONE;
#endif
};

struct parallel_unsequenced_policy
    : gcd_unsequenced_policy
    , tbb_unsequenced_policy
    , omp_unsequenced_policy {
#if defined(UFO_PAR_STL)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::PAR_UNSEQ;
#elif defined(UFO_PAR_GCD)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::GCD_UNSEQ;
#elif defined(UFO_PAR_TBB)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::TBB_UNSEQ;
#elif defined(UFO_PAR_OMP)
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::OMP_UNSEQ;
#else
	static constexpr detail::ExecutionPolicy policy = detail::ExecutionPolicy::NONE;
#endif
};

template <class, class = void>
struct is_execution_policy : std::false_type {
};

template <class T>
struct is_execution_policy<
    T, std::enable_if_t<detail::ExecutionPolicy::NONE != std::decay_t<T>::policy>>
    : std::disjunction<
          std::is_same<execution::sequenced_policy, std::decay_t<T>>,
          std::is_same<execution::parallel_policy, std::decay_t<T>>,
          std::is_same<execution::parallel_unsequenced_policy, std::decay_t<T>>,
          std::is_same<execution::unsequenced_policy, std::decay_t<T>>,
          std::is_same<execution::gcd_policy, std::decay_t<T>>,
          std::is_same<execution::gcd_unsequenced_policy, std::decay_t<T>>,
          std::is_same<execution::tbb_policy, std::decay_t<T>>,
          std::is_same<execution::tbb_unsequenced_policy, std::decay_t<T>>,
          std::is_same<execution::omp_policy, std::decay_t<T>>,
          std::is_same<execution::omp_unsequenced_policy, std::decay_t<T>>> {
};

template <class T>
constexpr inline bool is_execution_policy_v = is_execution_policy<T>::value;

template <class T>
constexpr inline bool is_seq_v = detail::ExecutionPolicy::SEQ == std::decay_t<T>::policy;

template <class T>
constexpr inline bool is_par_v = detail::ExecutionPolicy::PAR == std::decay_t<T>::policy;

template <class T>
constexpr inline bool is_par_unseq_v =
    detail::ExecutionPolicy::PAR_UNSEQ == std::decay_t<T>::policy;

template <class T>
constexpr inline bool is_unseq_v =
    detail::ExecutionPolicy::UNSEQ == std::decay_t<T>::policy;

template <class T>
constexpr inline bool is_gcd_v = detail::ExecutionPolicy::GCD == std::decay_t<T>::policy;

template <class T>
constexpr inline bool is_gcd_unseq_v =
    detail::ExecutionPolicy::GCD_UNSEQ == std::decay_t<T>::policy;

template <class T>
constexpr inline bool is_tbb_v = detail::ExecutionPolicy::TBB == std::decay_t<T>::policy;

template <class T>
constexpr inline bool is_tbb_unseq_v =
    detail::ExecutionPolicy::TBB_UNSEQ == std::decay_t<T>::policy;

template <class T>
constexpr inline bool is_omp_v = detail::ExecutionPolicy::OMP == std::decay_t<T>::policy;

template <class T>
constexpr inline bool is_omp_unseq_v =
    detail::ExecutionPolicy::OMP_UNSEQ == std::decay_t<T>::policy;

constexpr inline sequenced_policy            seq{};
constexpr inline parallel_policy             par{};
constexpr inline parallel_unsequenced_policy par_unseq{};
constexpr inline unsequenced_policy          unseq{};
constexpr inline gcd_policy                  gcd{};
constexpr inline gcd_unsequenced_policy      gcd_unseq{};
constexpr inline tbb_policy                  tbb{};
constexpr inline tbb_unsequenced_policy      tbb_unseq{};
constexpr inline omp_policy                  omp{};
constexpr inline omp_unsequenced_policy      omp_unseq{};
}  // namespace ufo::execution

#endif  // UFO_EXECUTION_EXECUTION_HPP