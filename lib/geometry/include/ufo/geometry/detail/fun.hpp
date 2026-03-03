#ifndef UFO_GEOMETRY_DETAIL_FUN_HPP
#define UFO_GEOMETRY_DETAIL_FUN_HPP

// UFO
#include <ufo/geometry/aabb.hpp>
#include <ufo/geometry/capsule.hpp>
#include <ufo/geometry/cylinder.hpp>
#include <ufo/geometry/ellipsoid.hpp>
#include <ufo/geometry/frustum.hpp>
#include <ufo/geometry/line.hpp>
#include <ufo/geometry/line_segment.hpp>
#include <ufo/geometry/obb.hpp>
#include <ufo/geometry/plane.hpp>
#include <ufo/geometry/ray.hpp>
#include <ufo/geometry/sphere.hpp>
#include <ufo/geometry/triangle.hpp>
#include <ufo/math/vec.hpp>

// STL
#include <array>
#include <cmath>
#include <cstddef>
#include <limits>

namespace ufo::detail
{
/**************************************************************************************
|                                                                                     |
|                                       Corners                                       |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr std::array<Vec<Dim, T>, (1 << Dim)> corners(AABB<Dim, T> const& a)
{
	std::array<Vec<Dim, T>, (1 << Dim)> c;
	for (std::size_t i = 0; i < (1 << Dim); ++i) {
		for (std::size_t j = 0; j < Dim; ++j) {
			c[i][j] = (i & (1 << j)) ? a.max[j] : a.min[j];
		}
	}
	return c;
}

template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr std::array<Vec<Dim, T>, (1 << Dim)> corners(OBB<Dim, T> const& a)
{
	std::array<Vec<Dim, T>, (1 << Dim)> c;
	std::array<Vec<Dim, T>, Dim>        axes;
	if constexpr (2 == Dim) {
		axes[0] = Vec<2, T>(a.rotation[0][0], a.rotation[0][1]) * a.half_length[0];
		axes[1] = Vec<2, T>(a.rotation[1][0], a.rotation[1][1]) * a.half_length[1];
	} else if constexpr (3 == Dim) {
		axes[0] = Vec<3, T>(a.rotation[0][0], a.rotation[0][1], a.rotation[0][2]) *
		          a.half_length[0];
		axes[1] = Vec<3, T>(a.rotation[1][0], a.rotation[1][1], a.rotation[1][2]) *
		          a.half_length[1];
		axes[2] = Vec<3, T>(a.rotation[2][0], a.rotation[2][1], a.rotation[2][2]) *
		          a.half_length[2];
	}

	for (std::size_t i = 0; i < (1 << Dim); ++i) {
		c[i] = a.center;
		for (std::size_t j = 0; j < Dim; ++j) {
			c[i] += (i & (1 << j)) ? axes[j] : -axes[j];
		}
	}
	return c;
}

template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr auto corners(Frustum<Dim, T> const& a)
{
	if constexpr (2 == Dim) {
		std::array<Vec<2, T>, 4> c;
		auto intersect = [](Plane<2, T> const& p1, Plane<2, T> const& p2) -> Vec<2, T> {
			T det = p1.normal.x() * p2.normal.y() - p1.normal.y() * p2.normal.x();
			return Vec<2, T>((p1.distance * p2.normal.y() - p2.distance * p1.normal.y()) / det,
			                 (p1.normal.x() * p2.distance - p2.normal.x() * p1.distance) / det);
		};
		// indices in test: 0: far_right, 1: far_left, 2: near_left, 3: near_right
		// planes: 0: left, 1: right, 2: bottom/near, 3: top/far
		c[0] = intersect(a[1], a[3]);  // right, top
		c[1] = intersect(a[0], a[3]);  // left, top
		c[2] = intersect(a[0], a[2]);  // left, bottom
		c[3] = intersect(a[1], a[2]);  // right, bottom
		return c;
	} else if constexpr (3 == Dim) {
		std::array<Vec<3, T>, 8> c;
		auto                     intersect = [](Plane<3, T> const& p1, Plane<3, T> const& p2,
                        Plane<3, T> const& p3) -> Vec<3, T> {
      auto m1 = cross(p2.normal, p3.normal);
      auto m2 = cross(p3.normal, p1.normal);
      auto m3 = cross(p1.normal, p2.normal);
      T    d  = dot(p1.normal, m1);
      return (m1 * p1.distance + m2 * p2.distance + m3 * p3.distance) / d;
		};
		// indices: 0: left, 1: right, 2: bottom, 3: top, 4: near, 5: far
		c[0] = intersect(a[1], a[3], a[5]);  // right, top, far
		c[1] = intersect(a[0], a[3], a[5]);  // left, top, far
		c[2] = intersect(a[0], a[2], a[5]);  // left, bottom, far
		c[3] = intersect(a[1], a[2], a[5]);  // right, bottom, far
		c[4] = intersect(a[1], a[3], a[4]);  // right, top, near
		c[5] = intersect(a[0], a[3], a[4]);  // left, top, near
		c[6] = intersect(a[0], a[2], a[4]);  // left, bottom, near
		c[7] = intersect(a[1], a[2], a[4]);  // right, bottom, near
		return c;
	} else {
		return std::array<Vec<Dim, T>, 0>{};
	}
}

/**************************************************************************************
|                                                                                     |
|                                         Min                                         |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> min(AABB<Dim, T> const& a)
{
	return a.min;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> min(Capsule<Dim, T> const& a)
{
	return min(a.start, a.end) - a.radius;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> min(Cylinder<Dim, T> const& a)
{
	Vec<Dim, T> axis   = a.end - a.start;
	T           length = ufo::length(axis);
	if (length < std::numeric_limits<T>::epsilon()) {
		return a.start - a.radius;
	}
	axis /= length;
	Vec<Dim, T> min_ext;
	for (std::size_t i = 0; i < Dim; ++i) {
		T s        = std::sqrt(std::max(T(0), T(1) - axis[i] * axis[i]));
		min_ext[i] = std::min(a.start[i], a.end[i]) - a.radius * s;
	}
	return min_ext;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> min(Ellipsoid<Dim, T> const& a)
{
	return a.center - a.radii;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> min(Frustum<Dim, T> const& a)
{
	auto        c = corners(a);
	Vec<Dim, T> m = c[0];
	for (std::size_t i = 1; i < c.size(); ++i) {
		m = min(m, c[i]);
	}
	return m;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> min(Line<Dim, T> const&)
{
	Vec<Dim, T> v;
	for (auto& x : v) {
		x = -std::numeric_limits<T>::infinity();
	}
	return v;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> min(LineSegment<Dim, T> const& a)
{
	return min(a.start, a.end);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> min(OBB<Dim, T> const& a)
{
	auto        c = corners(a);
	Vec<Dim, T> m = c[0];
	for (std::size_t i = 1; i < c.size(); ++i) {
		m = min(m, c[i]);
	}
	return m;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> min(Plane<Dim, T> const&)
{
	Vec<Dim, T> v;
	for (auto& x : v) {
		x = -std::numeric_limits<T>::infinity();
	}
	return v;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> min(Ray<Dim, T> const& a)
{
	Vec<Dim, T> min_pt;
	for (std::size_t i = 0; i < Dim; ++i) {
		min_pt[i] = a.direction[i] > T(0)   ? a.origin[i]
		            : a.direction[i] < T(0) ? -std::numeric_limits<T>::infinity()
		                                    : a.origin[i];
	}
	return min_pt;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> min(Sphere<Dim, T> const& a)
{
	return a.center - a.radius;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> min(Triangle<Dim, T> const& a)
{
	return min(a[0], min(a[1], a[2]));
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> min(Vec<Dim, T> const& a)
{
	return a;
}

/**************************************************************************************
|                                                                                     |
|                                         Max                                         |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> max(AABB<Dim, T> const& a)
{
	return a.max;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> max(Capsule<Dim, T> const& a)
{
	return max(a.start, a.end) + a.radius;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> max(Cylinder<Dim, T> const& a)
{
	Vec<Dim, T> axis   = a.end - a.start;
	T           length = ufo::length(axis);
	if (length < std::numeric_limits<T>::epsilon()) {
		return a.start + a.radius;
	}
	axis /= length;
	Vec<Dim, T> max_ext;
	for (std::size_t i = 0; i < Dim; ++i) {
		T s        = std::sqrt(std::max(T(0), T(1) - axis[i] * axis[i]));
		max_ext[i] = std::max(a.start[i], a.end[i]) + a.radius * s;
	}
	return max_ext;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> max(Ellipsoid<Dim, T> const& a)
{
	return a.center + a.radii;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> max(Frustum<Dim, T> const& a)
{
	auto        c = corners(a);
	Vec<Dim, T> m = c[0];
	for (std::size_t i = 1; i < c.size(); ++i) {
		m = max(m, c[i]);
	}
	return m;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> max(Line<Dim, T> const&)
{
	Vec<Dim, T> v;
	for (auto& x : v) {
		x = std::numeric_limits<T>::infinity();
	}
	return v;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> max(LineSegment<Dim, T> const& a)
{
	return max(a.start, a.end);
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> max(OBB<Dim, T> const& a)
{
	auto        c = corners(a);
	Vec<Dim, T> m = c[0];
	for (std::size_t i = 1; i < c.size(); ++i) {
		m = max(m, c[i]);
	}
	return m;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> max(Plane<Dim, T> const&)
{
	Vec<Dim, T> v;
	for (auto& x : v) {
		x = std::numeric_limits<T>::infinity();
	}
	return v;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> max(Ray<Dim, T> const& a)
{
	Vec<Dim, T> max_pt;
	for (std::size_t i = 0; i < Dim; ++i) {
		max_pt[i] = a.direction[i] < T(0)   ? a.origin[i]
		            : a.direction[i] > T(0) ? std::numeric_limits<T>::infinity()
		                                    : a.origin[i];
	}
	return max_pt;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> max(Sphere<Dim, T> const& a)
{
	return a.center + a.radius;
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> max(Triangle<Dim, T> const& a)
{
	return max(a[0], max(a[1], a[2]));
}

template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> max(Vec<Dim, T> const& a)
{
	return a;
}
}  // namespace ufo::detail

#endif  // UFO_GEOMETRY_DETAIL_FUN_HPP
