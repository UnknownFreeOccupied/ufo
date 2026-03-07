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

#ifndef UFO_GEOMETRY_TRIANGLE_HPP
#define UFO_GEOMETRY_TRIANGLE_HPP

// UFO
#include <ufo/geometry/aabb.hpp>
#include <ufo/numeric/vec.hpp>

// STL
#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <format>
#include <limits>
#include <ostream>

namespace ufo
{
/**
 * @struct Triangle
 * @brief Triangle in Dim-dimensional space.
 * @tparam Dim The dimensionality of the space (default: 3).
 * @tparam T   The numeric type (default: float), must be a floating-point type.
 * @details
 * Represents a triangle defined by three points.
 */
template <std::size_t Dim = 3, std::floating_point T = float>
struct Triangle {
	using value_type = T;

	/**
	 * @brief The vertices of the triangle.
	 */
	std::array<Vec<Dim, T>, 3> points;

	/**
	 * @brief Default constructor.
	 */
	constexpr Triangle() noexcept = default;

	/**
	 * @brief Copy constructor.
	 */
	constexpr Triangle(Triangle const&) noexcept = default;

	/**
	 * @brief Constructs a triangle from three points.
	 * @param [in] point_1 The first point.
	 * @param [in] point_2 The second point.
	 * @param [in] point_3 The third point.
	 */
	constexpr Triangle(Vec<Dim, T> point_1, Vec<Dim, T> point_2,
	                   Vec<Dim, T> point_3) noexcept
	    : points{point_1, point_2, point_3}
	{
	}

	/**
	 * @brief Converting constructor from a triangle with a different scalar type.
	 * @tparam U     The scalar type of the other triangle.
	 * @param [in] other The other triangle.
	 */
	template <std::convertible_to<T> U>
	constexpr explicit Triangle(Triangle<Dim, U> const& other) noexcept
	    : points{Vec<Dim, T>(other[0]), Vec<Dim, T>(other[1]), Vec<Dim, T>(other[2])}
	{
	}

	/**
	 * @brief Returns the dimensionality of the triangle.
	 * @return The dimensionality.
	 */
	[[nodiscard]] static constexpr std::size_t dimension() noexcept { return Dim; }

	/**
	 * @brief Returns the vertex at position pos.
	 * @param [in] pos The position of the vertex [0..2].
	 * @return A (const) reference to the vertex.
	 */
	[[nodiscard]] constexpr auto& operator[](this auto& self, std::size_t pos) noexcept
	{
		return self.points[pos];
	}

	/**
	 * @brief Returns the centroid (arithmetic mean) of the triangle.
	 * @return The centroid of the triangle.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> center() const noexcept
	{
		return (points[0] + points[1] + points[2]) / T(3);
	}

	/**
	 * @brief Returns the area of the triangle.
	 * @return The area of the triangle.
	 * @details
	 * Uses the general formula for a triangle in N-dimensional space.
	 */
	[[nodiscard]] constexpr T area() const noexcept
	{
		Vec<Dim, T> v1  = points[1] - points[0];
		Vec<Dim, T> v2  = points[2] - points[0];
		T           d1  = dot(v1, v1);
		T           d2  = dot(v2, v2);
		T           d12 = dot(v1, v2);
		return T(0.5) * std::sqrt(std::max(T(0), d1 * d2 - d12 * d12));
	}

	/**
	 * @brief Returns the AABB of the triangle.
	 * @return The AABB.
	 */
	[[nodiscard]] constexpr AABB<Dim, T> aabb() const noexcept
	{
		return AABB<Dim, T>(ufo::min(points[0], ufo::min(points[1], points[2])),
		                    ufo::max(points[0], ufo::max(points[1], points[2])));
	}

	/**
	 * @brief Returns the surface unit normal of the triangle.
	 * @return The surface unit normal.
	 * @details
	 * This method is only available for 3D triangles.
	 */
	[[nodiscard]] constexpr Vec<3, T> normal() const noexcept
	  requires(3 == Dim)
	{
		return normalize(cross(points[1] - points[0], points[2] - points[0]));
	}

	/**
	 * @brief Returns true if the triangle is degenerate.
	 * @param [in] eps The epsilon value for the check.
	 * @return True if the triangle is degenerate.
	 * @details
	 * A triangle is degenerate if its area is effectively zero.
	 */
	[[nodiscard]] constexpr bool isDegenerate(
	    T eps = std::numeric_limits<T>::epsilon()) const noexcept
	{
		return area() <= eps;
	}

	/**
	 * @brief Returns the volume of the triangle.
	 * @return The volume (always 0).
	 */
	[[nodiscard]] constexpr T volume() const noexcept { return T(0); }

	/**
	 * @brief Returns the diameter of the triangle.
	 * @return The diameter (maximum edge length).
	 */
	[[nodiscard]] constexpr T diameter() const noexcept
	{
		return std::sqrt(
		    std::max({normSquared(points[1] - points[0]), normSquared(points[2] - points[1]),
		              normSquared(points[0] - points[2])}));
	}

	/**
	 * @brief Equality operator.
	 */
	[[nodiscard]] bool operator==(Triangle const&) const = default;
};

//
// Deduction guides
//

template <std::size_t Dim, std::floating_point T>
Triangle(Vec<Dim, T>, Vec<Dim, T>, Vec<Dim, T>) -> Triangle<Dim, T>;

/**
 * @brief Output stream operator for Triangle.
 */
template <std::size_t Dim, std::floating_point T>
std::ostream& operator<<(std::ostream& out, Triangle<Dim, T> const& triangle)
{
	return out << "Point 1: [" << triangle[0] << "], Point 2: [" << triangle[1]
	           << "], Point 3: [" << triangle[2] << "]";
}

template <std::floating_point T>
using Triangle1 = Triangle<1, T>;
template <std::floating_point T>
using Triangle2 = Triangle<2, T>;
template <std::floating_point T>
using Triangle3 = Triangle<3, T>;
template <std::floating_point T>
using Triangle4 = Triangle<4, T>;

using Triangle1f = Triangle<1, float>;
using Triangle2f = Triangle<2, float>;
using Triangle3f = Triangle<3, float>;
using Triangle4f = Triangle<4, float>;

using Triangle1d = Triangle<1, double>;
using Triangle2d = Triangle<2, double>;
using Triangle3d = Triangle<3, double>;
using Triangle4d = Triangle<4, double>;

}  // namespace ufo

template <std::size_t Dim, std::floating_point T>
  requires std::formattable<T, char>
struct std::formatter<ufo::Triangle<Dim, T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::Triangle<Dim, T> const& triangle, std::format_context& ctx) const
	{
		return std::format_to(ctx.out(), "Points: [{}], [{}], [{}]", triangle[0], triangle[1],
		                      triangle[2]);
	}
};

#endif  // UFO_GEOMETRY_TRIANGLE_HPP