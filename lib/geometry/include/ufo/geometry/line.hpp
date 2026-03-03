/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se), Ramona Häuselmann (ramonaha@kth.se)
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

#ifndef UFO_GEOMETRY_LINE_HPP
#define UFO_GEOMETRY_LINE_HPP

// UFO
#include <ufo/geometry/aabb.hpp>
#include <ufo/math/vec.hpp>

// STL
#include <concepts>
#include <cstddef>
#include <format>
#include <limits>
#include <ostream>

namespace ufo
{
template <std::size_t Dim = 3, std::floating_point T = float>
struct Line {
	using value_type = T;

	/**
	 * @brief The origin point of the line.
	 */
	Vec<Dim, T> origin;

	/**
	 * @brief The direction of the line.
	 */
	Vec<Dim, T> direction;

	/**
	 * @brief Default constructor.
	 */
	constexpr Line() noexcept = default;

	/**
	 * @brief Copy constructor.
	 */
	constexpr Line(Line const&) noexcept = default;

	/**
	 * @brief Constructs a line from an origin and a direction.
	 *
	 * @param [in] origin    The origin of the line.
	 * @param [in] direction The direction of the line.
	 */
	constexpr Line(Vec<Dim, T> const& origin, Vec<Dim, T> const& direction) noexcept
	    : origin(origin), direction(normalize(direction))
	{
	}

	/**
	 * @brief Constructs a line from two points.
	 *
	 * @param [in] v_1 The first point.
	 * @param [in] v_2 The second point.
	 * @return The line passing through v_1 and v_2.
	 */
	[[nodiscard]] static constexpr Line fromPoints(Vec<Dim, T> const& v_1,
	                                               Vec<Dim, T> const& v_2) noexcept
	{
		return Line(v_1, v_2 - v_1);
	}

	/**
	 * @brief Converting constructor from a line with a different scalar type.
	 *
	 * @tparam U         The scalar type of the other line.
	 * @param [in] other The other line.
	 */
	template <std::convertible_to<T> U>
	constexpr explicit Line(Line<Dim, U> const& other) noexcept
	    : origin(Vec<Dim, T>(other.origin)), direction(Vec<Dim, T>(other.direction))
	{
	}

	/**
	 * @brief Returns the dimensionality of the line.
	 * @return The dimensionality.
	 */
	[[nodiscard]] static constexpr std::size_t dimension() noexcept { return Dim; }

	/**
	 * @brief Copy assignment operator.
	 */
	[[nodiscard]] constexpr Line& operator=(Line const&) noexcept = default;

	/**
	 * @brief Returns whether the line is degenerate.
	 * @return true if degenerate, false otherwise.
	 */
	[[nodiscard]] constexpr bool isDegenerate() const noexcept
	{
		return dot(direction, direction) < std::numeric_limits<T>::epsilon();
	}

	/**
	 * @brief Returns the center of the line.
	 * @return The origin of the line.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> center() const noexcept { return origin; }

	/**
	 * @brief Returns the point at distance t along the line.
	 * @param [in] t The distance along the line.
	 * @return The point at distance t.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> at(T t) const noexcept
	{
		return origin + t * direction;
	}

	/**
	 * @brief Returns the diameter of the line.
	 * @return The diameter (always infinity).
	 */
	[[nodiscard]] constexpr T diameter() const noexcept
	{
		return std::numeric_limits<T>::infinity();
	}

	/**
	 * @brief Returns the volume of the line.
	 * @return The volume (always 0).
	 */
	[[nodiscard]] constexpr T volume() const noexcept { return T(0); }

	/**
	 * @brief Returns the AABB of the line.
	 * @return The AABB.
	 */
	[[nodiscard]] constexpr AABB<Dim, T> aabb() const noexcept
	{
		// A line is infinite
		return AABB<Dim, T>(Vec<Dim, T>(-std::numeric_limits<T>::infinity()),
		                    Vec<Dim, T>(std::numeric_limits<T>::infinity()));
	}
};

/**
 * @brief Computes the intersection point of two lines.
 *
 * @tparam Dim   The dimensionality.
 * @tparam T     The numeric type.
 * @param [in] a The first line.
 * @param [in] b The second line.
 * @return The intersection point.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr Vec<Dim, T> intersectionPoint(Line<Dim, T> const& a,
                                                      Line<Dim, T> const& b)
{
	if constexpr (2 == Dim) {
		T det = a.direction.x() * b.direction.y() - a.direction.y() * b.direction.x();
		if (std::abs(det) < std::numeric_limits<T>::epsilon()) {
			return Vec<Dim, T>(std::numeric_limits<T>::infinity());
		}
		T t = ((b.origin.x() - a.origin.x()) * b.direction.y() -
		       (b.origin.y() - a.origin.y()) * b.direction.x()) /
		      det;
		return a.origin + t * a.direction;
	} else if constexpr (3 == Dim) {
		Vec<3, T> w0     = a.origin - b.origin;
		T         a_dot  = T(1);  // a.direction is normalized
		T         b_dot  = T(1);  // b.direction is normalized
		T         ab_dot = dot(a.direction, b.direction);
		T         aw_dot = dot(a.direction, w0);
		T         bw_dot = dot(b.direction, w0);
		T         denom  = a_dot * b_dot - ab_dot * ab_dot;
		if (std::abs(denom) < std::numeric_limits<T>::epsilon()) {
			return Vec<Dim, T>(std::numeric_limits<T>::infinity());
		}
		T sc = (ab_dot * bw_dot - b_dot * aw_dot) / denom;
		return a.origin + sc * a.direction;
	} else {
		// TODO: Implement for 4D
		return Vec<Dim, T>(std::numeric_limits<T>::quiet_NaN());
	}
}

/**
 * @brief Equality operator for Line.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] bool operator==(Line<Dim, T> const& lhs, Line<Dim, T> const& rhs) noexcept
{
	// Lines are equal if they are collinear (same direction and origin lies on the other
	// line) Or same origin and direction/opposite direction
	T d = dot(lhs.direction, rhs.direction);
	if (std::abs(std::abs(d) - T(1)) > std::numeric_limits<T>::epsilon()) {
		return false;
	}
	Vec<Dim, T> diff = lhs.origin - rhs.origin;
	if (normSquared(diff) < std::numeric_limits<T>::epsilon()) {
		return true;
	}
	diff = normalize(diff);
	return std::abs(std::abs(dot(diff, lhs.direction)) - T(1)) <
	       std::numeric_limits<T>::epsilon();
}

/**
 * @brief Output stream operator for Line.
 */
template <std::size_t Dim, std::floating_point T>
std::ostream& operator<<(std::ostream& out, Line<Dim, T> const& line)
{
	return out << "Origin: [" << line.origin << "], Direction: [" << line.direction << "]";
}

template <std::floating_point T>
using Line1 = Line<1, T>;
template <std::floating_point T>
using Line2 = Line<2, T>;
template <std::floating_point T>
using Line3 = Line<3, T>;
template <std::floating_point T>
using Line4 = Line<4, T>;

using Line1f = Line<1, float>;
using Line2f = Line<2, float>;
using Line3f = Line<3, float>;
using Line4f = Line<4, float>;

using Line1d = Line<1, double>;
using Line2d = Line<2, double>;
using Line3d = Line<3, double>;
using Line4d = Line<4, double>;

}  // namespace ufo

template <std::size_t Dim, std::floating_point T>
  requires std::formattable<T, char>
struct std::formatter<ufo::Line<Dim, T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::Line<Dim, T> const& line, std::format_context& ctx) const
	{
		return std::format_to(ctx.out(), "Origin: [{}], Direction: [{}]", line.origin,
		                      line.direction);
	}
};

#endif  // UFO_GEOMETRY_LINE_HPP