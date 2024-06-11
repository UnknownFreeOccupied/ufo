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

#ifndef UFO_GEOMETRY_INTERSECTS_HPP
#define UFO_GEOMETRY_INTERSECTS_HPP

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
[[nodiscard]] constexpr bool intersects(AABB<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return all(min(a) <= max(b) && min(b) <= max(a));
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(AABB<Dim, T> const& a, BS<Dim, T> const& b)
{
	auto closest_point    = closestPoint(a, b.center);
	auto distance_squared = distanceSquared(closest_point, b.center);
	auto radius_squared   = b.radius * b.radius;
	return distance_squared <= radius_squared;
}

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(AABB<Dim, T> const& a, Capsule<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

template <class T>
[[nodiscard]] constexpr bool intersects(AABB<3, T> const& a, Frustum<T> const& b)
{
	return 0 <= detail::classify(a, b.bottom) && 0 <= detail::classify(a, b.far) &&
	       0 <= detail::classify(a, b.left) && 0 <= detail::classify(a, b.near) &&
	       0 <= detail::classify(a, b.right) && 0 <= detail::classify(a, b.top);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(AABB<Dim, T> const&        a,
                                        LineSegment<Dim, T> const& b)
{
	Ray<Dim, T> ray;
	ray.origin    = b.start;
	ray.direction = b.end - b.start;
	T length      = norm(ray.direction);
	ray.direction /= length;
	return intersectsLine(a, ray, T(0), length);
}

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(AABB<Dim, T> const& a, OBB<Dim, T> const& b)
// {
// 	// TODO: Implement

// 	// std::array<float, 9> obb_rot_matrix = obb.rotation.rotMatrix();

// 	// std::array<Point, 15> test = {
// 	//     Point(1, 0, 0),  // AABB axis 1
// 	//     Point(0, 1, 0),  // AABB axis 2
// 	//     Point(0, 0, 1),  // AABB axis 3
// 	//     Point(obb_rot_matrix[0], obb_rot_matrix[1], obb_rot_matrix[2]),
// 	//     Point(obb_rot_matrix[3], obb_rot_matrix[4], obb_rot_matrix[5]),
// 	//     Point(obb_rot_matrix[6], obb_rot_matrix[7], obb_rot_matrix[8])};

// 	// for (std::size_t i{}; 3 > i; ++i) {  // Fill out rest of axis
// 	// 	test[6 + i * 3 + 0] = Point::cross(test[i], test[3]);
// 	// 	test[6 + i * 3 + 1] = Point::cross(test[i], test[4]);
// 	// 	test[6 + i * 3 + 2] = Point::cross(test[i], test[5]);
// 	// }

// 	// return std::all_of(std::cbegin(test), std::cend(test), [&aabb, &obb](auto const& t) {
// 	// 	return overlapOnAxis(aabb, obb, t);
// 	// });
// }

template <class T>
[[nodiscard]] constexpr bool intersects(AABB<3, T> const& a, Plane<T> const& b)
{
	auto hl = a.halfLength();

	T p_len{};
	for (std::size_t i{}; 3 > i; ++i) {
		p_len[i] += hl[i] * std::abs(b.normal[i]);
	}
	auto distance = dot(b.normal, a.center()) - b.distance;
	return std::abs(distance) <= p_len;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(AABB<Dim, T> const& a, Ray<Dim, T> const& b)
{
	return intersectsLine(a, b, T(0), std::numeric_limits<T>::max());
}

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(AABB<Dim, T> const& a, Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement

// 	// for (auto& point : triangle.points) {
// 	// 	point -= aabc.center;
// 	// }

// 	// std::array<Point, 3> f = {triangle.points[1] - triangle.points[0],
// 	//                           triangle.points[2] - triangle.points[1],
// 	//                           triangle.points[0] - triangle.points[2]};

// 	// std::array<Point, 3> u = {Point(1, 0, 0), Point(0, 1, 0), Point(0, 0, 1)};

// 	// std::array<Point, 9> axis = {
// 	//     Point::cross(u[0], f[0]), Point::cross(u[0], f[1]), Point::cross(u[0], f[2]),
// 	//     Point::cross(u[1], f[0]), Point::cross(u[1], f[1]), Point::cross(u[1], f[2]),
// 	//     Point::cross(u[2], f[0]), Point::cross(u[2], f[1]), Point::cross(u[2], f[2])};

// 	// std::array<Point, 9 + 3 + 1> all_axis = {axis[0],
// 	//                                          axis[1],
// 	//                                          axis[2],
// 	//                                          axis[3],
// 	//                                          axis[4],
// 	//                                          axis[5],
// 	//                                          axis[6],
// 	//                                          axis[7],
// 	//                                          axis[8],
// 	//                                          u[0],
// 	//                                          u[1],
// 	//                                          u[2],
// 	//                                          Point::cross(f[0], f[1])};

// 	// for (auto a : all_axis) {
// 	// 	std::array<float, 3> p = {Point::dot(triangle.points[0], a),
// 	// 	                          Point::dot(triangle.points[1], a),
// 	// 	                          Point::dot(triangle.points[2], a)};

// 	// 	float r = aabc.half_size * std::abs(Point::dot(u[0], a)) +
// 	// 	          aabc.half_size * std::abs(Point::dot(u[1], a)) +
// 	// 	          aabc.half_size * std::abs(Point::dot(u[2], a));

// 	// 	if (std::max(-std::max({p[0], p[1], p[2]}), std::min({p[0], p[1], p[2]})) > r) {
// 	// 		return false;
// 	// 	}
// 	// }

// 	// return true;
// }

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(AABB<Dim, T> const& a, Vec<Dim, T> const& b)
{
	return all(min(a) <= b && b <= max(a));
}

/**************************************************************************************
|                                                                                     |
|                                         BS                                          |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(BS<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(BS<Dim, T> const& a, BS<Dim, T> const& b)
{
	auto radius_sum       = a.radius + b.radius;
	auto distance_squared = normSquared(a.center - b.center);
	return distance_squared <= (radius_sum * radius_sum);
}

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(BS<Dim, T> const& a, Capsule<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

template <class T>
[[nodiscard]] constexpr bool intersects(BS<3, T> const& a, Frustum<T> const& b)
{
	for (std::size_t i{}; 6 > i; ++i) {
		if (-a.radius > dot(a.center, b[i].normal) + b[i].distance) {
			return false;
		}
	}
	return true;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(BS<Dim, T> const& a, LineSegment<Dim, T> const& b)
{
	auto closest_point    = closestPoint(b, a.center);
	auto distance_squared = distanceSquared(closest_point, a.center);
	return distance_squared <= (a.radius * a.radius);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(BS<Dim, T> const& a, OBB<Dim, T> const& b)
{
	auto closest_point    = closestPoint(b, a.center);
	auto distance_squared = distanceSquared(closest_point, a.center);
	return distance_squared <= (a.radius * a.radius);
}

template <class T>
[[nodiscard]] constexpr bool intersects(BS<3, T> const& a, Plane<T> const& b)
{
	auto closest_point    = closestPoint(b, a.center);
	auto distance_squared = distanceSquared(closest_point, a.center);
	return distance_squared < (a.radius * a.radius);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(BS<Dim, T> const& a, Ray<Dim, T> const& b)
{
	auto e    = a.center - b.origin;
	auto r_sq = a.radius * a.radius;
	auto e_sq = normSquared(e);
	auto d    = dot(e, b.direction);
	return 0 <= r_sq - (e_sq - d * d);
}

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(BS<Dim, T> const& a, Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(BS<Dim, T> const& a, Vec<Dim, T> const& b)
{
	return normSquared(b - a.center) <= (a.radius * a.radius);
}

/**************************************************************************************
|                                                                                     |
|                                       Capsule                                       |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Capsule<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Capsule<Dim, T> const& a, BS<Dim, T> const& b)
{
	return intersects(b, a);
}

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(Capsule<Dim, T> const& a,
//                                         Capsule<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr bool intersects(Capsule<3, T> const& a, Frustum<T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(Capsule<Dim, T> const&     a,
//                                         LineSegment<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(Capsule<Dim, T> const& a, OBB<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr bool intersects(Capsule<3, T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(Capsule<Dim, T> const& a, Ray<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(Capsule<Dim, T> const&  a,
//                                         Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(Capsule<Dim, T> const& a, Vec<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

/**************************************************************************************
|                                                                                     |
|                                       Frustum                                       |
|                                                                                     |
**************************************************************************************/

template <class T>
[[nodiscard]] constexpr bool intersects(Frustum<T> const& a, AABB<3, T> const& b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(Frustum<T> const& a, BS<3, T> const& b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(Frustum<T> const& a, Capsule<3, T> const& b)
{
	return intersects(b, a);
}

// template <class T>
// [[nodiscard]] constexpr bool intersects(Frustum<T> const& a, Frustum<T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr bool intersects(Frustum<T> const& a, LineSegment<3, T> const& b)
// {
// 	// TODO: Implement
// }

// template <class T>
// [[nodiscard]] constexpr bool intersects(Frustum<T> const& a, OBB<3, T> const& b)
// {
// 	// TODO: Implement

// 	// No idea if correct
// 	// 	return 0 <= classify(obb, frustum.bottom) && 0 <= classify(obb, frustum.far) &&
// 	// 	       0 <= classify(obb, frustum.left) && 0 <= classify(obb, frustum.near) &&
// 	// 	       0 <= classify(obb, frustum.right) && 0 <= classify(obb, frustum.top);
// }

// template <class T>
// [[nodiscard]] constexpr bool intersects(Frustum<T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement
// }

template <class T>
[[nodiscard]] constexpr bool intersects(Frustum<T> const& a, Ray<3, T> const& b)
{
	return false;
}

// template <class T>
// [[nodiscard]] constexpr bool intersects(Frustum<T> const& a, Triangle<3, T> const& b)
// {
// 	// TODO: Implement
// }

template <class T>
[[nodiscard]] constexpr bool intersects(Frustum<T> const& a, Vec<3, T> const& b)
{
	for (std::size_t i{}; 6 > i; ++i) {
		if (0 > dot(b, a[i].normal) + a[i].distance) {
			return false;
		}
	}
	return true;
}

/**************************************************************************************
|                                                                                     |
|                                    Line segment                                     |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(LineSegment<Dim, T> const& a,
                                        AABB<Dim, T> const&        b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(LineSegment<Dim, T> const& a, BS<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(LineSegment<Dim, T> const& a,
                                        Capsule<Dim, T> const&     b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(LineSegment<3, T> const& a, Frustum<T> const& b)
{
	return intersects(b, a);
}

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(LineSegment<Dim, T> const& a,
//                                         LineSegment<Dim, T> const& b)
// {
// 	// TODO: Implement

// 	// // Plucker coordinates
// 	// Point d_1 = line_segment_1.end - line_segment_1.start;
// 	// Point d_2 = line_segment_2.end - line_segment_2.start;
// 	// Point m_1 = Point::cross(line_segment_1.start, line_segment_1.end);
// 	// Point m_2 = Point::cross(line_segment_1.start, line_segment_1.end);
// 	// // FIXME: Almost equal?
// 	// return 0 == Point::dot(d_1, m_2) + Point::dot(d_2, m_1);
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(LineSegment<Dim, T> const& a,
//                                         OBB<Dim, T> const&         b)
// {
// 	// TODO: Implement

// 	// Ray ray;
// 	// ray.origin                = line_segment.start;
// 	// ray.direction             = line_segment.end - line_segment.start;
// 	// float line_length_squared = ray.direction.squaredNorm();
// 	// if (line_length_squared < 0.0000001f) {
// 	// 	return intersects(line_segment.start, obb);
// 	// }
// 	// ray.direction /= line_length_squared;  // Normalize

// 	// // Begin ray casting

// 	// Point p = obb.center - ray.origin;

// 	// Point X(obb.rotation[0], 0, 0);
// 	// Point Y(0, obb.rotation[1], 0);
// 	// Point Z(0, 0, obb.rotation[2]);

// 	// Point f(Point::dot(X, ray.direction), Point::dot(Y, ray.direction),
// 	//         Point::dot(Z, ray.direction));

// 	// Point e(Point::dot(X, p), Point::dot(Y, p), Point::dot(Z, p));

// 	// float t[6] = {0, 0, 0, 0, 0, 0};
// 	// for (std::size_t i{}; 3 > i; ++i) {
// 	// 	if (0.0f == f[i])  // FIXME: Should be approximate equal
// 	// 	{
// 	// 		if (-e[i] - obb.half_size[i] > 0 || -e[i] + obb.half_size[i] < 0) {
// 	// 			return false;
// 	// 		}
// 	// 		f[i] = 0.00001f;  // Avoid div by 0!
// 	// 	}
// 	// 	t[i * 2 + 0] = (e[i] + obb.half_size[i]) / f[i];  // tmin[x, y, z]
// 	// 	t[i * 2 + 1] = (e[i] - obb.half_size[i]) / f[i];  // tmax[x, y, z]
// 	// }

// 	// float tmin = std::max(std::max(std::min(t[0], t[1]), std::min(t[2], t[3])),
// 	//                       std::min(t[4], t[5]));
// 	// float tmax = std::min(std::min(std::max(t[0], t[1]), std::max(t[2], t[3])),
// 	//                       std::max(t[4], t[5]));

// 	// // if tmax < 0, ray is intersecting AABB
// 	// // but entire AABB is behing it's origin
// 	// if (tmax < 0) {
// 	// 	return false;
// 	// }

// 	// // if tmin > tmax, ray doesn't intersect AABB
// 	// if (tmin > tmax) {
// 	// 	return false;
// 	// }

// 	// // If tmin is < 0, tmax is closer
// 	// float t_result = tmin;

// 	// if (tmin < 0) {
// 	// 	t_result = tmax;
// 	// }

// 	// // End ray casting
// 	// return t_result >= 0 && t_result * t_result <= line_length_squared;
// }

// template <class T>
// [[nodiscard]] constexpr bool intersects(LineSegment<3, T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement

// 	// Point ab   = line_segment.end - line_segment.start;
// 	// float n_A  = Point::dot(plane.normal, line_segment.start);
// 	// float n_AB = Point::dot(plane.normal, ab);
// 	// if (0.0f == n_AB)  // FIXME: Almost equal?
// 	// {
// 	// 	return false;
// 	// }
// 	// float t = (plane.distance - n_A) / n_AB;
// 	// return t >= 0.0f && t <= 1.0f;
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(LineSegment<Dim, T> const& a,
//                                         Ray<Dim, T> const&         b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(LineSegment<Dim, T> const& a,
//                                         Triangle<Dim, T> const&    b)
// {
// 	// TODO: Implement
// }

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(LineSegment<Dim, T> const& a,
                                        Vec<Dim, T> const&         b)
{
	auto closest_point    = closestPoint(a, b);
	auto distance_squared = distanceSquared(closest_point, b);
	return 0 == distance_squared;
}

/**************************************************************************************
|                                                                                     |
|                                         OBB                                         |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(OBB<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(OBB<Dim, T> const& a, BS<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(OBB<Dim, T> const& a, Capsule<Dim, T> const& b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(OBB<3, T> const& a, Frustum<T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(OBB<Dim, T> const&         a,
                                        LineSegment<Dim, T> const& b)
{
	return intersects(b, a);
}

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(OBB<Dim, T> const& a, OBB<Dim, T> const& b)
// {
// 	// TODO: Implement

// 	// std::array<float, 9> obb_1_rot_matrix = obb_1.rotation.rotMatrix();
// 	// std::array<float, 9> obb_2_rot_matrix = obb_2.rotation.rotMatrix();

// 	// std::array<Point, 15> test = {
// 	//     Point(obb_1_rot_matrix[0], obb_1_rot_matrix[1], obb_1_rot_matrix[2]),
// 	//     Point(obb_1_rot_matrix[3], obb_1_rot_matrix[4], obb_1_rot_matrix[5]),
// 	//     Point(obb_1_rot_matrix[6], obb_1_rot_matrix[7], obb_1_rot_matrix[8]),
// 	//     Point(obb_2_rot_matrix[0], obb_2_rot_matrix[1], obb_2_rot_matrix[2]),
// 	//     Point(obb_2_rot_matrix[3], obb_2_rot_matrix[4], obb_2_rot_matrix[5]),
// 	//     Point(obb_2_rot_matrix[6], obb_2_rot_matrix[7], obb_2_rot_matrix[8])};

// 	// for (std::size_t i{}; 3 > i; ++i) {  // Fill out rest of axis
// 	// 	test[6 + i * 3 + 0] = Point::cross(test[i], test[0]);
// 	// 	test[6 + i * 3 + 1] = Point::cross(test[i], test[1]);
// 	// 	test[6 + i * 3 + 2] = Point::cross(test[i], test[2]);
// 	// }

// 	// return std::all_of(std::cbegin(test), std::cend(test), [&obb_1, &obb_2](auto const&
// 	// t) { 	return overlapOnAxis(obb_1, obb_2, t);
// 	// });
// }

// template <class T>
// [[nodiscard]] constexpr bool intersects(OBB<3, T> const& a, Plane<T> const& b)
// {
// 	// TODO: Implement

// 	// Point rot[] = {
// 	//     Point(obb.rotation[0], 0, 0),
// 	//     Point(0, obb.rotation[1], 0),
// 	//     Point(0, 0, obb.rotation[2]),
// 	// };
// 	// Point normal = plane.normal;

// 	// // Project the half extents of the AABB onto the plane normal
// 	// float p_len = obb.half_size.x * std::fabs(Point::dot(normal, rot[0])) +
// 	//               obb.half_size.y * std::fabs(Point::dot(normal, rot[1])) +
// 	//               obb.half_size.z * std::fabs(Point::dot(normal, rot[2]));
// 	// // Find the distance from the center of the OBB to the plane
// 	// float distance = Point::dot(plane.normal, obb.center) - plane.distance;
// 	// // Intersection occurs if the distance falls within the projected side
// 	// return std::abs(distance) <= p_len;
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(OBB<Dim, T> const& a, Ray<Dim, T> const& b)
// {
// 	// TODO: Implement

// 	// Point p = obb.center - ray.origin;

// 	// Point X(obb.rotation[0], 0, 0);
// 	// Point Y(0, obb.rotation[1], 0);
// 	// Point Z(0, 0, obb.rotation[2]);

// 	// Point f(Point::dot(X, ray.direction), Point::dot(Y, ray.direction),
// 	//         Point::dot(Z, ray.direction));

// 	// Point e(Point::dot(X, p), Point::dot(Y, p), Point::dot(Z, p));

// 	// float t[6] = {0, 0, 0, 0, 0, 0};
// 	// for (std::size_t i{}; 3 > i; ++i) {
// 	// 	if (0.0f == f[i])  // FIXME: Should be approximate equal?
// 	// 	{
// 	// 		if (-e[i] - obb.half_size[i] > 0 || -e[i] + obb.half_size[i] < 0) {
// 	// 			return false;
// 	// 		}
// 	// 		f[i] = 0.00001f;  // Avoid div by 0!
// 	// 	}

// 	// 	t[i * 2 + 0] = (e[i] + obb.half_size[i]) / f[i];  // tmin[x, y, z]
// 	// 	t[i * 2 + 1] = (e[i] - obb.half_size[i]) / f[i];  // tmax[x, y, z]
// 	// }

// 	// float tmin = std::max(std::max(std::min(t[0], t[1]), std::min(t[2], t[3])),
// 	//                       std::min(t[4], t[5]));
// 	// float tmax = std::min(std::min(std::max(t[0], t[1]), std::max(t[2], t[3])),
// 	//                       std::max(t[4], t[5]));

// 	// // if tmax < 0, ray is intersecting AABB
// 	// // but entire AABB is behing it's origin
// 	// if (tmax < 0) {
// 	// 	return false;
// 	// }
// 	// // if tmin > tmax, ray doesn't intersect AABB
// 	// if (tmin > tmax) {
// 	// 	return false;
// 	// }
// 	// return true;
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(OBB<Dim, T> const& a, Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(OBB<Dim, T> const& a, Vec<Dim, T> const& b)
// {
// 	// TODO: Implement

// 	// // FIXME: Implement look earlier. THIS IS WRONG!
// 	// Point                dir            = point - obb.center;
// 	// std::array<float, 9> obb_rot_matrix = obb.rotation.rotMatrix();
// 	// for (std::size_t i{}; 3 > i; ++i) {
// 	// 	Point axis(obb_rot_matrix[i * 3], obb_rot_matrix[i * 3 + 1],
// 	// 	           obb_rot_matrix[i * 3 + 2]);
// 	// 	float distance = Point::dot(dir, axis);
// 	// 	if (distance > obb.half_size[i]) {
// 	// 		return false;
// 	// 	}
// 	// 	if (distance < -obb.half_size[i])  // FIXME: Should this be else if?
// 	// 	{
// 	// 		return false;
// 	// 	}
// 	// }
// 	// return true;
// }

/**************************************************************************************
|                                                                                     |
|                                        Plane                                        |
|                                                                                     |
**************************************************************************************/

template <class T>
[[nodiscard]] constexpr bool intersects(Plane<T> const& a, AABB<3, T> const& b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(Plane<T> const& a, BS<3, T> const& b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(Plane<T> const& a, Capsule<3, T> const& b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(Plane<T> const& a, Frustum<T> const& b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(Plane<T> const& a, LineSegment<3, T> const& b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(Plane<T> const& a, OBB<3, T> const& b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(Plane<T> const& a, Plane<T> const& b)
{
	auto d = cross(plane_1.normal, plane_2.normal);
	return 0 != dot(d, d);
}

// template <class T>
// [[nodiscard]] constexpr bool intersects(Plane<T> const& a, Ray<3, T> const& b)
// {
// 	// TODO: Implement

// 	// float nd = Point::dot(ray.direction, plane.normal);
// 	// float pn = Point::dot(ray.origin, plane.normal);
// 	// if (nd >= 0) {
// 	// 	return false;
// 	// }
// 	// float t = (plane.distance - pn) / nd;
// 	// return t >= 0;
// }

// template <class T>
// [[nodiscard]] constexpr bool intersects(Plane<T> const& a, Triangle<3, T> const& b)
// {
// 	// TODO: Implement
// }

template <class T>
[[nodiscard]] constexpr bool intersects(Plane<T> const& a, Vec<3, T> const& b)
{
	// TODO: Check if correct
	return T(0) == dot(b, a.normal) - a.distance;
}

/**************************************************************************************
|                                                                                     |
|                                         Ray                                         |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Ray<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Ray<Dim, T> const& a, BS<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Ray<Dim, T> const& a, Capsule<Dim, T> const& b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(Ray<3, T> const& a, Frustum<T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Ray<Dim, T> const&         a,
                                        LineSegment<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Ray<Dim, T> const& a, OBB<Dim, T> const& b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(Ray<3, T> const& a, Plane<T> const& b)
{
	return intersects(b, a);
}

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(Ray<Dim, T> const& a, Ray<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(Ray<Dim, T> const& a, Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Ray<Dim, T> const& a, Vec<Dim, T> const& b)
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
[[nodiscard]] constexpr bool intersects(Triangle<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Triangle<Dim, T> const& a, BS<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Triangle<Dim, T> const& a,
                                        Capsule<Dim, T> const&  b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(Triangle<3, T> const& a, Frustum<T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Triangle<Dim, T> const&    a,
                                        LineSegment<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Triangle<Dim, T> const& a, OBB<Dim, T> const& b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(Triangle<3, T> const& a, Plane<T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Triangle<Dim, T> const& a, Ray<Dim, T> const& b)
{
	return intersects(b, a);
}

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(Triangle<Dim, T> const& a,
//                                         Triangle<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

// template <std::size_t Dim, class T>
// [[nodiscard]] constexpr bool intersects(Triangle<Dim, T> const& a, Vec<Dim, T> const& b)
// {
// 	// TODO: Implement
// }

/**************************************************************************************
|                                                                                     |
|                                         Vec                                         |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Vec<Dim, T> const& a, AABB<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Vec<Dim, T> const& a, BS<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Vec<Dim, T> const& a, Capsule<Dim, T> const& b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(Vec<3, T> const& a, Frustum<T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Vec<Dim, T> const&         a,
                                        LineSegment<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Vec<Dim, T> const& a, OBB<Dim, T> const& b)
{
	return intersects(b, a);
}

template <class T>
[[nodiscard]] constexpr bool intersects(Vec<3, T> const& a, Plane<T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Vec<Dim, T> const& a, Ray<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Vec<Dim, T> const& a, Triangle<Dim, T> const& b)
{
	return intersects(b, a);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool intersects(Vec<Dim, T> const& a, Vec<Dim, T> const& b)
{
	return all(a == b);
}
}  // namespace ufo

#endif  // UFO_GEOMETRY_INTERSECTS_HPP