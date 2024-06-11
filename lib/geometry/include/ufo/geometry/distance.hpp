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

#ifndef UFO_GEOMETRY_DISTANCE_HPP
#define UFO_GEOMETRY_DISTANCE_HPP

// UFO
#include <ufo/geometry/shape/aabb.hpp>
#include <ufo/geometry/shape/bs.hpp>
#include <ufo/geometry/shape/frustum.hpp>
#include <ufo/geometry/shape/line_segment.hpp>
#include <ufo/geometry/shape/obb.hpp>
#include <ufo/geometry/shape/plane.hpp>
#include <ufo/geometry/shape/ray.hpp>
#include <ufo/geometry/shape/triangle.hpp>
#include <ufo/math/vec.hpp>

// STL
#include <cmath>

namespace ufo
{
/**************************************************************************************
|                                                                                     |
|                                        AABB                                         |
|                                                                                     |
**************************************************************************************/

/*!
 * @brief Computes the minimum squared distance between a and b.
 *
 * @note The squared distance is generally faster to compute than the distance. Therefore,
 * if the relative distance is what is important then it is recommended to use this
 * function.
 *
 * @param a,b Geometry objects.
 * @return The minimum squared distance between a and b.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(AABB<Dim, T> const& a, AABB<Dim, T> const& b)
{
	T delta  = std::fdim(min(a)[0], max(b)[0]) + std::fdim(min(b)[0], max(a)[0]);
	T result = delta * delta;
	for (std::size_t i = 1; Dim > i; ++i) {
		T delta = std::fdim(min(a)[i], max(b)[i]) + std::fdim(min(b)[i], max(a)[i]);
		result += delta * delta;
	}
	return result;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(AABB<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return std::sqrt(distanceSquared(a, b));
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(AABB<Dim, T> const& a, BS<Dim, T> const& b)
{
	auto dist = distance(a, b);
	return dist * dist;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(AABB<Dim, T> const& a, BS<Dim, T> const& b)
{
	return std::fdim(distance(a, b.center), b.radius);
}

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(AABB<3, T> const& a, Frustum<T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(AABB<3, T> const& a, Frustum<T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(AABB<Dim, T> const&        a,
//                                           LineSegment<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(AABB<Dim, T> const& a, LineSegment<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(AABB<Dim, T> const& a, OBB<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(AABB<Dim, T> const& a, OBB<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(AABB<3, T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(AABB<3, T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(AABB<Dim, T> const& a, Ray<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(AABB<Dim, T> const& a, Ray<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(AABB<Dim, T> const&     a,
//                                           Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(AABB<Dim, T> const& a, Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(AABB<Dim, T> const& a, Vec<Dim, T> const& b)
{
	return distanceSquared(clamp(b, min(a)(), max(a)()), b);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(AABB<Dim, T> const& a, Vec<Dim, T> const& b)
{
	return std::sqrt(distanceSquared(a, b));
}

/**************************************************************************************
|                                                                                     |
|                                         BS                                          |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(BS<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(BS<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(BS<Dim, T> const& a, BS<Dim, T> const& b)
{
	auto dist = distance(a, b);
	return dist * dist;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(BS<Dim, T> const& a, BS<Dim, T> const& b)
{
	return std::fdim(distance(a.center, b.center), a.radius + b.radius);
}

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(BS<3, T> const& a, Frustum<T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(BS<3, T> const& a, Frustum<T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(BS<Dim, T> const&          a,
//                                           LineSegment<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(BS<Dim, T> const& a, LineSegment<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(BS<Dim, T> const& a, OBB<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(BS<Dim, T> const& a, OBB<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(BS<3, T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(BS<3, T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(BS<Dim, T> const& a, Ray<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(BS<Dim, T> const& a, Ray<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(BS<Dim, T> const& a, Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(BS<Dim, T> const& a, Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(BS<Dim, T> const& a, Vec<Dim, T> const& b)
{
	auto dist = distance(a, b);
	return dist * dist;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(BS<Dim, T> const& a, Vec<Dim, T> const& b)
{
	return std::fdim(distance(a.center, b), a.radius);
}

/**************************************************************************************
|                                                                                     |
|                                       Frustum                                       |
|                                                                                     |
**************************************************************************************/

template <class T>
[[nodiscard]] constexpr T distanceSquared(Frustum<T> const& a, AABB<3, T> const& b)
{
	return distanceSquared(b, a);
}

template <class T>
[[nodiscard]] constexpr T distance(Frustum<T> const& a, AABB<3, T> const& b)
{
	return distance(b, a);
}

template <class T>
[[nodiscard]] constexpr T distanceSquared(Frustum<T> const& a, BS<3, T> const& b)
{
	return distanceSquared(b, a);
}

template <class T>
[[nodiscard]] constexpr T distance(Frustum<T> const& a, BS<3, T> const& b)
{
	return distance(b, a);
}

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(Frustum<T> const& a, Frustum<T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(Frustum<T> const& a, Frustum<T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(Frustum<T> const& a, LineSegment<3, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(Frustum<T> const& a, LineSegment<3, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(Frustum<T> const& a, OBB<3, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(Frustum<T> const& a, OBB<3, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(Frustum<T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(Frustum<T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(Frustum<T> const& a, Ray<3, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(Frustum<T> const& a, Ray<3, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(Frustum<T> const& a, Triangle<3, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(Frustum<T> const& a, Triangle<3, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(Frustum<T> const& a, Vec<3, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(Frustum<T> const& a, Vec<3, T> const& b)
// {
// 	// TODO: Implement
// }

/**************************************************************************************
|                                                                                     |
|                                    Line segment                                     |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(LineSegment<Dim, T> const& a,
                                          AABB<Dim, T> const&        b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(LineSegment<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(LineSegment<Dim, T> const& a,
                                          BS<Dim, T> const&          b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(LineSegment<Dim, T> const& a, BS<Dim, T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(LineSegment<Dim, T> const& a,
                                          Frustum<T> const&          b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(LineSegment<Dim, T> const& a, Frustum<T> const& b)
{
	return distance(b, a);
}

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(LineSegment<Dim, T> const& a,
//                                           LineSegment<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(LineSegment<Dim, T> const& a,
//                                    LineSegment<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(LineSegment<Dim, T> const& a,
//                                           OBB<Dim, T> const&         b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(LineSegment<Dim, T> const& a, OBB<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(LineSegment<Dim, T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(LineSegment<Dim, T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(LineSegment<Dim, T> const& a,
//                                           Ray<Dim, T> const&         b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(LineSegment<Dim, T> const& a, Ray<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(LineSegment<Dim, T> const& a,
//                                           Triangle<Dim, T> const&    b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(LineSegment<Dim, T> const& a,
//                                    Triangle<Dim, T> const&    b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(LineSegment<Dim, T> const& a,
//                                           Vec<Dim, T> const&         b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(LineSegment<Dim, T> const& a, Vec<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

/**************************************************************************************
|                                                                                     |
|                                         OBB                                         |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(OBB<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(OBB<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(OBB<Dim, T> const& a, BS<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(OBB<Dim, T> const& a, BS<Dim, T> const& b)
{
	return distance(b, a);
}

template <class T>
[[nodiscard]] constexpr T distanceSquared(OBB<3, T> const& a, Frustum<T> const& b)
{
	return distanceSquared(b, a);
}

template <class T>
[[nodiscard]] constexpr T distance(OBB<3, T> const& a, Frustum<T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(OBB<Dim, T> const&         a,
                                          LineSegment<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(OBB<Dim, T> const& a, LineSegment<Dim, T> const& b)
{
	return distance(b, a);
}

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(OBB<Dim, T> const& a, OBB<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(OBB<Dim, T> const& a, OBB<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(OBB<3, T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(OBB<3, T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(OBB<Dim, T> const& a, Ray<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(OBB<Dim, T> const& a, Ray<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(OBB<Dim, T> const& a, Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(OBB<Dim, T> const& a, Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(OBB<Dim, T> const& a, Vec<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(OBB<Dim, T> const& a, Vec<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

/**************************************************************************************
|                                                                                     |
|                                        Plane                                        |
|                                                                                     |
**************************************************************************************/

template <class T>
[[nodiscard]] constexpr T distanceSquared(Plane<T> const& a, AABB<3, T> const& b)
{
	return distanceSquared(b, a);
}

template <class T>
[[nodiscard]] constexpr T distance(Plane<T> const& a, AABB<3, T> const& b)
{
	return distance(b, a);
}

template <class T>
[[nodiscard]] constexpr T distanceSquared(Plane<T> const& a, BS<3, T> const& b)
{
	return distanceSquared(b, a);
}

template <class T>
[[nodiscard]] constexpr T distance(Plane<T> const& a, BS<3, T> const& b)
{
	return distance(b, a);
}

template <class T>
[[nodiscard]] constexpr T distanceSquared(Plane<T> const& a, Frustum<T> const& b)
{
	return distanceSquared(b, a);
}

template <class T>
[[nodiscard]] constexpr T distance(Plane<T> const& a, Frustum<T> const& b)
{
	return distance(b, a);
}

template <class T>
[[nodiscard]] constexpr T distanceSquared(Plane<T> const& a, LineSegment<3, T> const& b)
{
	return distanceSquared(b, a);
}

template <class T>
[[nodiscard]] constexpr T distance(Plane<T> const& a, LineSegment<3, T> const& b)
{
	return distance(b, a);
}

template <class T>
[[nodiscard]] constexpr T distanceSquared(Plane<T> const& a, OBB<3, T> const& b)
{
	return distanceSquared(b, a);
}

template <class T>
[[nodiscard]] constexpr T distance(Plane<T> const& a, OBB<3, T> const& b)
{
	return distance(b, a);
}

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(Plane<T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(Plane<T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(Plane<T> const& a, Ray<3, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(Plane<T> const& a, Ray<3, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(Plane<T> const& a, Triangle<3, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(Plane<T> const& a, Triangle<3, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distanceSquared(Plane<T> const& a, Vec<3, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr T distance(Plane<T> const& a, Vec<3, T> const& b)
// {
// 	// TODO: Implement
// }

/**************************************************************************************
|                                                                                     |
|                                         Ray                                         |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Ray<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Ray<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Ray<Dim, T> const& a, BS<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Ray<Dim, T> const& a, BS<Dim, T> const& b)
{
	return distance(b, a);
}

template <class T>
[[nodiscard]] constexpr T distanceSquared(Ray<3, T> const& a, Frustum<T> const& b)
{
	return distanceSquared(b, a);
}

template <class T>
[[nodiscard]] constexpr T distance(Ray<3, T> const& a, Frustum<T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Ray<Dim, T> const&         a,
                                          LineSegment<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Ray<Dim, T> const& a, LineSegment<Dim, T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Ray<Dim, T> const& a, OBB<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Ray<Dim, T> const& a, OBB<Dim, T> const& b)
{
	return distance(b, a);
}

template <class T>
[[nodiscard]] constexpr T distanceSquared(Ray<3, T> const& a, Plane<T> const& b)
{
	return distanceSquared(b, a);
}

template <class T>
[[nodiscard]] constexpr T distance(Ray<3, T> const& a, Plane<T> const& b)
{
	return distance(b, a);
}

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(Ray<Dim, T> const& a, Ray<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(Ray<Dim, T> const& a, Ray<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(Ray<Dim, T> const& a, Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(Ray<Dim, T> const& a, Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(Ray<Dim, T> const& a, Vec<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(Ray<Dim, T> const& a, Vec<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

/**************************************************************************************
|                                                                                     |
|                                      Triangle                                       |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Triangle<Dim, T> const& a,
                                          AABB<Dim, T> const&     b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Triangle<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Triangle<Dim, T> const& a, BS<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Triangle<Dim, T> const& a, BS<Dim, T> const& b)
{
	return distance(b, a);
}

template <class T>
[[nodiscard]] constexpr T distanceSquared(Triangle<3, T> const& a, Frustum<T> const& b)
{
	return distanceSquared(b, a);
}

template <class T>
[[nodiscard]] constexpr T distance(Triangle<3, T> const& a, Frustum<T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Triangle<Dim, T> const&    a,
                                          LineSegment<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Triangle<Dim, T> const&    a,
                                   LineSegment<Dim, T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Triangle<Dim, T> const& a, OBB<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Triangle<Dim, T> const& a, OBB<Dim, T> const& b)
{
	return distance(b, a);
}

template <class T>
[[nodiscard]] constexpr T distanceSquared(Triangle<3, T> const& a, Plane<T> const& b)
{
	return distanceSquared(b, a);
}

template <class T>
[[nodiscard]] constexpr T distance(Triangle<3, T> const& a, Plane<T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Triangle<Dim, T> const& a, Ray<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Triangle<Dim, T> const& a, Ray<Dim, T> const& b)
{
	return distance(b, a);
}

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(Triangle<Dim, T> const& a,
//                                           Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(Triangle<Dim, T> const& a, Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distanceSquared(Triangle<Dim, T> const& a, Vec<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr T distance(Triangle<Dim, T> const& a, Vec<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

/**************************************************************************************
|                                                                                     |
|                                         Vec                                         |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Vec<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Vec<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Vec<Dim, T> const& a, BS<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Vec<Dim, T> const& a, BS<Dim, T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Vec<Dim, T> const& a, Frustum<T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Vec<Dim, T> const& a, Frustum<T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Vec<Dim, T> const&         a,
                                          LineSegment<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Vec<Dim, T> const& a, LineSegment<Dim, T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Vec<Dim, T> const& a, OBB<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Vec<Dim, T> const& a, OBB<Dim, T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Vec<Dim, T> const& a, Plane<T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Vec<Dim, T> const& a, Plane<T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Vec<Dim, T> const& a, Ray<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Vec<Dim, T> const& a, Ray<Dim, T> const& b)
{
	return distance(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Vec<Dim, T> const& a, Triangle<Dim, T> const& b)
{
	return distanceSquared(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distance(Vec<Dim, T> const& a, Triangle<Dim, T> const& b)
{
	return distance(b, a);
}
}  // namespace ufo

#endif  // UFO_GEOMETRY_DISTANCE_HPP