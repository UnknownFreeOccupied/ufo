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

#ifndef UFO_GEOMETRY_CONTAINS_HPP
#define UFO_GEOMETRY_CONTAINS_HPP

// UFO
#include <ufo/geometry/shape/aabb.hpp>
#include <ufo/geometry/shape/bs.hpp>
#include <ufo/geometry/shape/capsule.hpp>
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

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(AABB<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return all(min(a) <= min(b) && max(b) <= max(a));
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(AABB<Dim, T> const& a, BS<Dim, T> const& b)
{
	return contains(a, AABB<Dim, T>(min(b), max(b)));
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(AABB<Dim, T> const& a, Capsule<Dim, T> const& b)
{
	return contains(a, AABB<Dim, T>(min(b), max(b)));
}

template <class T>
[[nodiscard]] constexpr bool contains(AABB<3, T> const& a, Frustum<T> const& b)
{
	return contains(a, AABB<3, T>(min(b), max(b)));
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(AABB<Dim, T> const& a, LineSegment<Dim, T> const& b)
{
	return contains(a, b.start) && contains(a, b.end);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(AABB<Dim, T> const& a, OBB<Dim, T> const& b)
{
	return contains(a, min(b)) && contains(a, max(b));
}

template <class T>
[[nodiscard]] constexpr bool contains(AABB<3, T> const& a, Plane<T> const& b)
{
	return false;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(AABB<Dim, T> const& a, Ray<Dim, T> const& b)
{
	return false;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(AABB<Dim, T> const& a, Triangle<Dim, T> const& b)
{
	return contains(a, AABB<Dim, T>(min(b), max(b)));
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(AABB<Dim, T> const& a, Vec<Dim, T> const& b)
{
	return all(min(a) <= b && b <= max(a));
}

/**************************************************************************************
|                                                                                     |
|                                         BS                                          |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(BS<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return contains(a, min(b)) && contains(a, max(b));
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(BS<Dim, T> const& a, BS<Dim, T> const& b)
{
	return a.radius >= distance(a.center, b.center) + b.radius;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(BS<Dim, T> const& a, Capsule<Dim, T> const& b)
{
	return distance(a.center, b.start) + b.radius <= a.radius &&
	       distance(a.center, b.end) + b.radius <= a.radius;
}

template <class T>
[[nodiscard]] constexpr bool contains(BS<3, T> const& a, Frustum<T> const& b)
{
	return contains(a, min(b)) && contains(a, max(b));
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(BS<Dim, T> const& a, LineSegment<Dim, T> const& b)
{
	return contains(a, b.start) && contains(a, b.end);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(BS<Dim, T> const& a, OBB<Dim, T> const& b)
{
	return contains(a, min(b)) && contains(a, max(b));
}

template <class T>
[[nodiscard]] constexpr bool contains(BS<3, T> const& a, Plane<T> const& b)
{
	return false;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(BS<Dim, T> const& a, Ray<Dim, T> const& b)
{
	return false;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(BS<Dim, T> const& a, Triangle<Dim, T> const& b)
{
	return contains(a, b[0]) && contains(a, b[1]) && contains(a, b[2]);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(BS<Dim, T> const& a, Vec<Dim, T> const& b)
{
	return distanceSquared(a, b) <= a.radius * a.radius;
}

/**************************************************************************************
|                                                                                     |
|                                       Capsule                                       |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Capsule<Dim, T> const& a, AABB<Dim, T> const& b)
{
	for (auto c : corners(b)) {
		if (!contains(a, c)) {
			return false;
		}
	}
	return true;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Capsule<Dim, T> const& a, BS<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Capsule<Dim, T> const& a, Capsule<Dim, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Capsule<3, T> const& a, Frustum<T> const& b)
{
	return contains(a, AABB<3, T>(min(b), max(b)));
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Capsule<Dim, T> const&     a,
                                      LineSegment<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Capsule<Dim, T> const& a, OBB<Dim, T> const& b)
{
	return contains(a, AABB<Dim, T>(min(b), max(b)));
}

template <class T>
[[nodiscard]] constexpr bool contains(Capsule<3, T> const& a, Plane<T> const& b)
{
	return false;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Capsule<Dim, T> const& a, Ray<Dim, T> const& b)
{
	return false;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Capsule<Dim, T> const& a, Triangle<Dim, T> const& b)
{
	return contains(a, b[0]) && contains(a, b[1]) && contains(a, b[2]);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Capsule<Dim, T> const& a, Vec<Dim, T> const& b)
{
	return distanceSquared(LineSegment<Dim, T>(a.start, a.end), b) <= a.radius * a.radius;
}

/**************************************************************************************
|                                                                                     |
|                                       Frustum                                       |
|                                                                                     |
**************************************************************************************/

template <class T>
[[nodiscard]] constexpr bool contains(Frustum<T> const& a, AABB<3, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Frustum<T> const& a, BS<3, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Frustum<T> const& a, Capsule<3, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Frustum<T> const& a, Frustum<T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Frustum<T> const& a, LineSegment<3, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Frustum<T> const& a, OBB<3, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Frustum<T> const& a, Plane<T> const& b)
{
	return false;
}

template <class T>
[[nodiscard]] constexpr bool contains(Frustum<T> const& a, Ray<3, T> const& b)
{
	return false;
}

template <class T>
[[nodiscard]] constexpr bool contains(Frustum<T> const& a, Triangle<3, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Frustum<T> const& a, Vec<3, T> const& b)
{
	// TODO: Implement
}

/**************************************************************************************
|                                                                                     |
|                                    Line segment                                     |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(LineSegment<Dim, T> const& a, AABB<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(LineSegment<Dim, T> const& a, BS<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(LineSegment<Dim, T> const& a,
                                      Capsule<Dim, T> const&     b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(LineSegment<3, T> const& a, Frustum<T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(LineSegment<Dim, T> const& a,
                                      LineSegment<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(LineSegment<Dim, T> const& a, OBB<Dim, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(LineSegment<3, T> const& a, Plane<T> const& b)
{
	return false;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(LineSegment<Dim, T> const& a, Ray<Dim, T> const& b)
{
	return false;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(LineSegment<Dim, T> const& a,
                                      Triangle<Dim, T> const&    b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(LineSegment<Dim, T> const& a, Vec<Dim, T> const& b)
{
	// TODO: Implement
}

/**************************************************************************************
|                                                                                     |
|                                         OBB                                         |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(OBB<Dim, T> const& a, AABB<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(OBB<Dim, T> const& a, BS<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(OBB<Dim, T> const& a, Capsule<Dim, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(OBB<3, T> const& a, Frustum<T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(OBB<Dim, T> const& a, LineSegment<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(OBB<Dim, T> const& a, OBB<Dim, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(OBB<3, T> const& a, Plane<T> const& b)
{
	return false;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(OBB<Dim, T> const& a, Ray<Dim, T> const& b)
{
	return false;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(OBB<Dim, T> const& a, Triangle<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(OBB<Dim, T> const& a, Vec<Dim, T> const& b)
{
	// TODO: Implement
}

/**************************************************************************************
|                                                                                     |
|                                        Plane                                        |
|                                                                                     |
**************************************************************************************/

template <class T>
[[nodiscard]] constexpr bool contains(Plane<T> const& a, AABB<3, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Plane<T> const& a, BS<3, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Plane<T> const& a, Capsule<3, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Plane<T> const& a, Frustum<T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Plane<T> const& a, LineSegment<3, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Plane<T> const& a, OBB<3, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Plane<T> const& a, Plane<T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Plane<T> const& a, Ray<3, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Plane<T> const& a, Triangle<3, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Plane<T> const& a, Vec<3, T> const& b)
{
	// TODO: Implement
}

/**************************************************************************************
|                                                                                     |
|                                         Ray                                         |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Ray<Dim, T> const& a, AABB<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Ray<Dim, T> const& a, BS<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Ray<Dim, T> const& a, Capsule<Dim, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Ray<3, T> const& a, Frustum<T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Ray<Dim, T> const& a, LineSegment<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Ray<Dim, T> const& a, OBB<Dim, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Ray<3, T> const& a, Plane<T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Ray<Dim, T> const& a, Ray<Dim, T> const& b)
{
	return a == b;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Ray<Dim, T> const& a, Triangle<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Ray<Dim, T> const& a, Vec<Dim, T> const& b)
{
	if (a.origin == b) {
		return true;
	}
	auto direction = normalize(b - a.origin);
	return T(1) == dot(direction, a.direction);
}

/**************************************************************************************
|                                                                                     |
|                                      Triangle                                       |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Triangle<Dim, T> const& a, AABB<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Triangle<Dim, T> const& a, BS<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Triangle<Dim, T> const& a, Capsule<Dim, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Triangle<3, T> const& a, Frustum<T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Triangle<Dim, T> const&    a,
                                      LineSegment<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Triangle<Dim, T> const& a, OBB<Dim, T> const& b)
{
	// TODO: Implement
}

template <class T>
[[nodiscard]] constexpr bool contains(Triangle<3, T> const& a, Plane<T> const& b)
{
	return false;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Triangle<Dim, T> const& a, Ray<Dim, T> const& b)
{
	return false;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Triangle<Dim, T> const& a,
                                      Triangle<Dim, T> const& b)
{
	// TODO: Implement
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Triangle<Dim, T> const& a, Vec<Dim, T> const& b)
{
	// TODO: Implement
}

/**************************************************************************************
|                                                                                     |
|                                         Vec                                         |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Vec<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return a == min(b) && a == max(b);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Vec<Dim, T> const& a, BS<Dim, T> const& b)
{
	return T(0) == b.radius && a == b.center;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Vec<Dim, T> const& a, Capsule<Dim, T> const& b)
{
	return T(0) == b.radius && a == b.start && a == b.end;
}

template <class T>
[[nodiscard]] constexpr bool contains(Vec<3, T> const& a, Frustum<T> const& b)
{
	return a == min(b) && a == max(b);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Vec<Dim, T> const& a, LineSegment<Dim, T> const& b)
{
	return a == b.start && a == b.end;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Vec<Dim, T> const& a, OBB<Dim, T> const& b)
{
	return a == b.center && Vec<Dim, T>(0) == b.half_length;
}

template <class T>
[[nodiscard]] constexpr bool contains(Vec<3, T> const& a, Plane<T> const& b)
{
	return false;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Vec<Dim, T> const& a, Ray<Dim, T> const& b)
{
	return false;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Vec<Dim, T> const& a, Triangle<Dim, T> const& b)
{
	return a == b[0] && a == b[1] && a == b[2];
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool contains(Vec<Dim, T> const& a, Vec<Dim, T> const& b)
{
	return all(a == b);
}
}  // namespace ufo

#endif  // UFO_GEOMETRY_CONTAINS_HPP