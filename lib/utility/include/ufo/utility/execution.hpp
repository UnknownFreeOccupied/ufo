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
#endif

// OMP
#if defined(UFO_OMP)
#include <omp.h>
#endif

namespace ufo
{
#if defined(UFO_TBB)
namespace execution = std::execution;

template <class T>
using is_execution_policy = std::is_execution_policy<T>;
#else
namespace execution
{
// sequenced execution policy
class sequenced_policy
{
};

// parallel execution policy
class parallel_policy
{
};

// parallel and unsequenced execution policy
class parallel_unsequenced_policy
{
};

// execution policy objects
inline constexpr sequenced_policy            seq{};
inline constexpr parallel_policy             par{};
inline constexpr parallel_unsequenced_policy par_unseq{};
}  // namespace execution

template <class T>
struct is_execution_policy
    : std::disjunction<std::is_same<execution::sequenced_policy, T>,
                       std::is_same<execution::parallel_policy, T>,
                       std::is_same<execution::parallel_unsequenced_policy, T>> {
};
#endif

template <class T>
inline constexpr bool is_execution_policy_v = is_execution_policy<T>::value;
}  // namespace ufo

#endif  // UFO_UTILITY_EXECUTION_HPP