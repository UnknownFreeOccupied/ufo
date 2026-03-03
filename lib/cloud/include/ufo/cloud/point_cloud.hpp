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

#ifndef UFO_CLOUD_POINT_CLOUD_HPP
#define UFO_CLOUD_POINT_CLOUD_HPP

// UFO
#include <ufo/cloud/cloud.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/math/transform.hpp>
#include <ufo/math/vec.hpp>

// STL
#include <algorithm>
#include <cstddef>
#include <limits>

namespace ufo
{
/**
 * @brief A structure-of-arrays point cloud whose first channel is `Dim`-dimensional
 * positions of scalar type `T`.
 *
 * `PointCloud` is an alias for `Cloud<Vec<Dim, T>, Rest...>`, which stores per-point
 * data in separate contiguous arrays (one per channel) for cache-friendly access. The
 * position channel is always channel 0; any additional per-point attributes (colors,
 * normals, intensities, ...) are appended as `Rest...`.
 *
 * @tparam Dim  Spatial dimensionality (typically 2, 3, or 4).
 * @tparam T    Scalar type for position coordinates (e.g., `float`, `double`).
 * @tparam Rest Additional per-point attribute types stored in separate SoA channels.
 */
template <std::size_t Dim, class T, class... Rest>
using PointCloud = Cloud<Vec<Dim, T>, Rest...>;

//! @brief 2D single-precision point cloud (positions only).
using PointCloud2f = PointCloud<2, float>;
//! @brief 3D single-precision point cloud (positions only).
using PointCloud3f = PointCloud<3, float>;
//! @brief 4D single-precision point cloud (positions only).
using PointCloud4f = PointCloud<4, float>;
//! @brief 2D double-precision point cloud (positions only).
using PointCloud2d = PointCloud<2, double>;
//! @brief 3D double-precision point cloud (positions only).
using PointCloud3d = PointCloud<3, double>;
//! @brief 4D double-precision point cloud (positions only).
using PointCloud4d = PointCloud<4, double>;

//
// Transform
//

/**
 * @brief Applies a rigid transform to every point position and returns the result.
 *
 * A copy of `pc` is made and transformed; the original is left unchanged.
 * Runs sequentially (equivalent to calling the overload with `execution::seq`).
 *
 * @tparam Dim  Spatial dimensionality.
 * @tparam T    Scalar type.
 * @tparam Rest Additional per-point attribute types (passed through unchanged).
 * @param t  The rigid transform (rotation + translation) to apply.
 * @param pc The source point cloud (taken by value to allow move-from).
 * @return   A new point cloud with transformed positions and identical attributes.
 */
template <std::size_t Dim, class T, class... Rest>
[[nodiscard]] PointCloud<Dim, T, Rest...> transform(Transform<Dim, T> const&    t,
                                                    PointCloud<Dim, T, Rest...> pc)
{
	transformInPlace(t, pc);
	return pc;
}

/**
 * @brief Applies a rigid transform to every point position and returns the result,
 * using the given execution policy.
 *
 * A copy of `pc` is made, then `transformInPlace` is called on it. The original
 * cloud is left unchanged.
 *
 * @tparam ExecutionPolicy An execution policy type (e.g., `execution::seq_t`,
 *                         `execution::par_t`).
 * @tparam Dim             Spatial dimensionality.
 * @tparam T               Scalar type.
 * @tparam Rest            Additional per-point attribute types.
 * @param policy Execution policy controlling parallelism.
 * @param t      The rigid transform to apply.
 * @param pc     The source point cloud (taken by value to allow move-from).
 * @return       A new point cloud with transformed positions.
 */
template <class ExecutionPolicy, std::size_t Dim, class T, class... Rest>
  requires execution::is_execution_policy_v<ExecutionPolicy>
[[nodiscard]] PointCloud<Dim, T, Rest...> transform(ExecutionPolicy&&           policy,
                                                    Transform<Dim, T> const&    t,
                                                    PointCloud<Dim, T, Rest...> pc)
{
	transformInPlace(std::forward<ExecutionPolicy>(policy), t, pc);
	return pc;
}

/**
 * @brief Applies a rigid transform to every point position in-place.
 *
 * Only the position channel (channel 0) is modified; all other attribute channels
 * are left unchanged. Runs sequentially.
 *
 * @tparam Dim  Spatial dimensionality.
 * @tparam T    Scalar type.
 * @tparam Rest Additional per-point attribute types.
 * @param t  The rigid transform to apply.
 * @param pc The point cloud to transform in-place.
 */
template <std::size_t Dim, class T, class... Rest>
void transformInPlace(Transform<Dim, T> const& t, PointCloud<Dim, T, Rest...>& pc)
{
	auto v = view<0>(pc);
	transform(v.begin(), v.end(), v.begin(), t);
}

/**
 * @brief Applies a rigid transform to every point position in-place, using the given
 * execution policy.
 *
 * Only the position channel (channel 0) is modified; all other attribute channels
 * are left unchanged.
 *
 * @tparam ExecutionPolicy An execution policy type.
 * @tparam Dim             Spatial dimensionality.
 * @tparam T               Scalar type.
 * @tparam Rest            Additional per-point attribute types.
 * @param policy Execution policy controlling parallelism.
 * @param t      The rigid transform to apply.
 * @param pc     The point cloud to transform in-place.
 */
template <class ExecutionPolicy, std::size_t Dim, class T, class... Rest>
  requires execution::is_execution_policy_v<ExecutionPolicy>
void transformInPlace(ExecutionPolicy&& policy, Transform<Dim, T> const& t,
                      PointCloud<Dim, T, Rest...>& pc)
{
	auto v = view<0>(pc);
	transform(std::forward<ExecutionPolicy>(policy), v.begin(), v.end(), v.begin(), t);
}

//
// Filter
//

/**
 * @brief Returns a copy of `pc` with all points outside the given distance range
 * removed.
 *
 * Points are kept if and only if:
 * @code
 *   min_distance <= distance(origin, point) <= max_distance
 * @endcode
 * Distance comparisons are performed on squared distances to avoid `sqrt`. If
 * `filter_nan` is `true`, any point with a NaN coordinate is also removed.
 *
 * @tparam Dim  Spatial dimensionality.
 * @tparam T    Scalar type.
 * @tparam Rest Additional per-point attribute types (filtered together with positions).
 * @param pc           The source point cloud (taken by value to allow move-from).
 * @param origin       The reference point from which distances are measured.
 * @param min_distance Minimum allowed distance (inclusive). Use `T(0)` for no lower
 *                     bound.
 * @param max_distance Maximum allowed distance (inclusive). Use
 *                     `std::numeric_limits<T>::max()` for no upper bound.
 * @param filter_nan   If `true`, points with any NaN coordinate are removed.
 *                     Defaults to `true`.
 * @return A new point cloud containing only points within the distance range.
 */
template <std::size_t Dim, class T, class... Rest>
[[nodiscard]] PointCloud<Dim, T, Rest...> filterDistance(PointCloud<Dim, T, Rest...> pc,
                                                         Vec<Dim, T> const& origin,
                                                         T const&           min_distance,
                                                         T const&           max_distance,
                                                         bool filter_nan = true)
{
	filterDistanceInPlace(pc, origin, min_distance, max_distance, filter_nan);
	return pc;
}

/**
 * @brief Removes in-place all points from `pc` that fall outside the given distance
 * range.
 *
 * Points are kept if and only if:
 * @code
 *   min_distance <= distance(origin, point) <= max_distance
 * @endcode
 * Comparisons are done on squared distances to avoid `sqrt`. If `filter_nan` is
 * `true`, any point with a NaN coordinate is also erased.
 *
 * If the parameters describe a no-op filter (min_distance <= 0,
 * max_distance >= `std::numeric_limits<T>::max()`, and filter_nan is `false`), the
 * function returns immediately without touching the cloud.
 *
 * All per-point attribute channels (positions and `Rest...`) are erased together,
 * preserving the SoA layout invariant.
 *
 * @tparam Dim  Spatial dimensionality.
 * @tparam T    Scalar type.
 * @tparam Rest Additional per-point attribute types.
 * @param pc           The point cloud to filter in-place.
 * @param origin       The reference point from which distances are measured.
 * @param min_distance Minimum allowed distance (inclusive).
 * @param max_distance Maximum allowed distance (inclusive).
 * @param filter_nan   If `true`, points with any NaN coordinate are removed.
 *                     Defaults to `true`.
 */
template <std::size_t Dim, class T, class... Rest>
void filterDistanceInPlace(PointCloud<Dim, T, Rest...>& pc, Vec<Dim, T> const& origin,
                           T const& min_distance, T const& max_distance,
                           bool filter_nan = true)
{
	if (T(0) >= min_distance && std::numeric_limits<T>::max() <= max_distance &&
	    !filter_nan) {
		return;
	}

	auto const min_sq = min_distance * min_distance;
	auto const max_sq = max_distance * max_distance;

	auto to_erase = std::ranges::remove_if(
	    pc, [&origin, min_sq, max_sq, filter_nan](Vec<Dim, T> const& x) {
		    auto dist_sq = distanceSquared(origin, x);
		    return min_sq > dist_sq || max_sq < dist_sq || (filter_nan && isnan(x));
	    });
	pc.erase(to_erase.begin(), to_erase.end());
}
}  // namespace ufo

#endif  // UFO_CLOUD_POINT_CLOUD_HPP
