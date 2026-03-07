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

#ifndef UFO_GEOMETRY_SPHERE_HPP
#define UFO_GEOMETRY_SPHERE_HPP

// UFO
#include <ufo/geometry/aabb.hpp>
#include <ufo/numeric/vec.hpp>

// STL
#include <concepts>
#include <cstddef>
#include <format>
#include <iostream>
#include <numbers>

namespace ufo
{
/**
 * @struct Sphere
 * @brief Sphere in Dim-dimensional space.
 * @tparam Dim The dimensionality of the space (default: 3).
 * @tparam T   The numeric type (default: float), must be a floating-point type.
 * @details
 * Represents a sphere defined by a center point and a radius.
 */
template <std::size_t Dim = 3, std::floating_point T = float>
struct Sphere {
	using value_type = T;

	/**
	 * @brief The center of the sphere.
	 */
	Vec<Dim, T> center_;

	/**
	 * @brief The radius of the sphere.
	 */
	T radius;

	/**
	 * @brief Default constructor.
	 */
	constexpr Sphere() noexcept = default;

	/**
	 * @brief Constructs a sphere from a center point and a radius.
	 * @param [in] center The center of the sphere.
	 * @param [in] radius The radius of the sphere.
	 */
	constexpr Sphere(Vec<Dim, T> const& center, T radius) noexcept
	    : center_(center), radius(radius)
	{
	}

	/**
	 * @brief Copy constructor.
	 */
	constexpr Sphere(Sphere const&) noexcept = default;

	/**
	 * @brief Converting constructor from a sphere with a different scalar type.
	 * @tparam U The scalar type of the other sphere.
	 * @param [in] other The other sphere.
	 */
	template <std::convertible_to<T> U>
	constexpr explicit Sphere(Sphere<Dim, U> const& other) noexcept
	    : center_(Vec<Dim, T>(other.center())), radius(static_cast<T>(other.radius))
	{
	}

	/**
	 * @brief Returns the dimensionality of the sphere.
	 * @return The dimensionality.
	 */
	[[nodiscard]] static constexpr std::size_t dimension() noexcept { return Dim; }

	/**
	 * @brief Returns the center of the sphere.
	 * @return The center.
	 */
	[[nodiscard]] constexpr Vec<Dim, T>& center() noexcept { return center_; }

	/**
	 * @brief Returns the center of the sphere.
	 * @return The center.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> const& center() const noexcept { return center_; }

	/**
	 * @brief Returns the diameter of the sphere.
	 * @return The diameter.
	 */
	[[nodiscard]] constexpr T diameter() const noexcept { return radius * T(2); }

	/**
	 * @brief Returns the AABB of the sphere.
	 * @return The AABB.
	 */
	[[nodiscard]] constexpr AABB<Dim, T> aabb() const noexcept
	{
		return AABB<Dim, T>(center_, radius);
	}

	/**
	 * @brief Returns whether the sphere is degenerate.
	 * @return true if degenerate, false otherwise.
	 */
	[[nodiscard]] constexpr bool isDegenerate() const noexcept { return radius < T(0); }

	/**
	 * @brief Returns the volume of the sphere.
	 * @return The volume.
	 * @details
	 * The volume is calculated using the formula V = (pi^n * r^n) / (n * Gamma(n/2 + 1)),
	 * where n is the dimensionality of the space and r is the radius.
	 */
	[[nodiscard]] constexpr T volume() const noexcept
	{
		if constexpr (1 == Dim) {
			return diameter();
		} else if constexpr (2 == Dim) {
			return std::numbers::pi_v<T> * radius * radius;
		} else if constexpr (3 == Dim) {
			return (T(4) / T(3)) * std::numbers::pi_v<T> * radius * radius * radius;
		} else if constexpr (4 == Dim) {
			return (std::numbers::pi_v<T> * std::numbers::pi_v<T> * radius * radius * radius *
			        radius) /
			       T(2);
		} else {
			return T(0);
		}
	}

	/**
	 * @brief Equality operator.
	 */
	[[nodiscard]] bool operator==(Sphere const&) const = default;
};

//
// Deduction guides
//

template <std::size_t Dim, std::floating_point T>
Sphere(Vec<Dim, T>, T) -> Sphere<Dim, T>;

/**
 * @brief Output stream operator for Sphere.
 */
template <std::size_t Dim, std::floating_point T>
std::ostream& operator<<(std::ostream& out, Sphere<Dim, T> const& sphere)
{
	return out << "Center: [" << sphere.center() << "], Radius: " << sphere.radius;
}

template <std::floating_point T>
using Sphere1 = Sphere<1, T>;
template <std::floating_point T>
using Sphere2 = Sphere<2, T>;
template <std::floating_point T>
using Sphere3 = Sphere<3, T>;
template <std::floating_point T>
using Sphere4 = Sphere<4, T>;

using Sphere1f = Sphere<1, float>;
using Sphere2f = Sphere<2, float>;
using Sphere3f = Sphere<3, float>;
using Sphere4f = Sphere<4, float>;

using Sphere1d = Sphere<1, double>;
using Sphere2d = Sphere<2, double>;
using Sphere3d = Sphere<3, double>;
using Sphere4d = Sphere<4, double>;

}  // namespace ufo

template <std::size_t Dim, std::floating_point T>
  requires std::formattable<T, char>
struct std::formatter<ufo::Sphere<Dim, T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::Sphere<Dim, T> const& sphere, std::format_context& ctx) const
	{
		return std::format_to(ctx.out(), "Center: [{}], Radius: {}", sphere.center(),
		                      sphere.radius);
	}
};

#endif  // UFO_GEOMETRY_SPHERE_HPP