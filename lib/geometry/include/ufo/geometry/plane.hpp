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

#ifndef UFO_GEOMETRY_PLANE_HPP
#define UFO_GEOMETRY_PLANE_HPP

// UFO
#include <ufo/math/vec.hpp>

// STL
#include <concepts>
#include <cstddef>
#include <limits>
#include <ostream>

#include "ufo/geometry/aabb.hpp"

namespace ufo
{
/**
 * @struct Plane
 * @brief Plane in space.
 * @tparam Dim The dimensionality of the plane (default: 3).
 * @tparam T The numeric type (default: float), must be a floating-point type.
 * @details
 * Represents a plane defined by a normal and a distance from the origin.
 * The plane is defined by the equation:
 *   normal . x = distance
 */
template <std::size_t Dim = 3, std::floating_point T = float>
struct Plane {
	using value_type = T;

	/**
	 * @brief The normal of the plane.
	 */
	Vec<Dim, T> normal;

	/**
	 * @brief The distance from the origin.
	 */
	T distance{};

	/**
	 * @brief Default constructor.
	 */
	constexpr Plane() noexcept = default;

	/**
	 * @brief Copy constructor.
	 */
	constexpr Plane(Plane const&) noexcept = default;

	/**
	 * @brief Constructs a plane from a normal.
	 * @param [in] normal The normal of the plane.
	 */
	constexpr explicit Plane(Vec<Dim, T> const& normal) noexcept : normal(normal) {}

	/**
	 * @brief Constructs a plane from a normal and a distance.
	 * @param [in] normal   The normal of the plane.
	 * @param [in] distance The distance from the origin.
	 */
	constexpr Plane(Vec<Dim, T> const& normal, T distance) noexcept
	    : normal(normal), distance(distance)
	{
	}

	/**
	 * @brief Constructs a plane from two points.
	 * @param [in] v_1 The first point.
	 * @param [in] v_2 The second point.
	 */
	constexpr Plane(Vec<Dim, T> const& v_1, Vec<Dim, T> const& v_2) noexcept
	  requires(2 == Dim)
	{
		auto aux = v_2 - v_1;
		normal   = normalize(Vec<2, T>(-aux.y(), aux.x()));
		distance = dot(normal, v_1);
	}

	/**
	 * @brief Constructs a plane from three points.
	 * @param [in] v_1 The first point.
	 * @param [in] v_2 The second point.
	 * @param [in] v_3 The third point.
	 */
	constexpr Plane(Vec<Dim, T> const& v_1, Vec<Dim, T> const& v_2,
	                Vec<Dim, T> const& v_3) noexcept
	  requires(3 == Dim)
	{
		auto aux_1 = v_1 - v_2;
		auto aux_2 = v_3 - v_2;
		normal     = normalize(cross(aux_2, aux_1));
		distance   = dot(normal, v_2);
	}

	/**
	 * @brief Conversion operator to a plane with a different scalar type.
	 */
	template <std::convertible_to<T> U>
	constexpr explicit Plane(Plane<Dim, U> const& other) noexcept
	    : normal(Vec<Dim, T>(other.normal)), distance(static_cast<T>(other.distance))
	{
	}

	/**
	 * @brief Returns the dimensionality of the plane.
	 * @return The dimensionality.
	 */
	[[nodiscard]] static constexpr std::size_t dimension() noexcept { return Dim; }

	/**
	 * @brief Equality operator.
	 * @param [in] other The other plane.
	 * @retval true if the planes are equal
	 * @retval false otherwise
	 */
	[[nodiscard]] bool operator==(Plane const&) const = default;

	/**
	 * @brief Returns whether the plane is degenerate.
	 * @return true if degenerate, false otherwise.
	 */
	[[nodiscard]] constexpr bool isDegenerate() const noexcept
	{
		return dot(normal, normal) < std::numeric_limits<T>::epsilon();
	}

	/**
	 * @brief Returns the volume of the plane.
	 * @return The volume (always 0).
	 */
	[[nodiscard]] constexpr T volume() const noexcept { return T(0); }

	/**
	 * @brief Returns the diameter of the plane.
	 * @return The diameter (always infinity).
	 */
	[[nodiscard]] constexpr T diameter() const noexcept
	{
		return std::numeric_limits<T>::infinity();
	}

	/**
	 * @brief Returns the AABB of the plane.
	 * @return The AABB.
	 */
	[[nodiscard]] constexpr AABB<Dim, T> aabb() const noexcept
	{
		// A plane is infinite
		return AABB<Dim, T>(Vec<Dim, T>(-std::numeric_limits<T>::infinity()),
		                    Vec<Dim, T>(std::numeric_limits<T>::infinity()));
	}
};

/**
 * @brief Output stream operator for Plane.
 */
template <std::size_t Dim, std::floating_point T>
std::ostream& operator<<(std::ostream& out, Plane<Dim, T> const& plane)
{
	return out << "Normal: [" << plane.normal << "], Distance: " << plane.distance;
}

using Plane3f = Plane<3, float>;
using Plane3d = Plane<3, double>;

}  // namespace ufo

template <std::size_t Dim, std::floating_point T>
  requires std::formattable<T, char>
struct std::formatter<ufo::Plane<Dim, T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::Plane<Dim, T> const& plane, std::format_context& ctx) const
	{
		return std::format_to(ctx.out(), "Normal: [{}], Distance: {}", plane.normal,
		                      plane.distance);
	}
};

#endif  // UFO_GEOMETRY_PLANE_HPP