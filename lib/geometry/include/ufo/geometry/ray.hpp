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

#ifndef UFO_GEOMETRY_RAY_HPP
#define UFO_GEOMETRY_RAY_HPP

// UFO
#include <ufo/geometry/aabb.hpp>
#include <ufo/numeric/vec.hpp>

// STL
#include <concepts>
#include <cstddef>
#include <format>
#include <ostream>

namespace ufo
{
/**
 * @struct Ray
 * @brief Ray in Dim-dimensional space.
 * @tparam Dim The dimensionality of the space (default: 3).
 * @tparam T   The numeric type (default: float), must be a floating-point type.
 * @details
 * Represents a ray defined by an origin point and a direction vector.
 */
template <std::size_t Dim = 3, std::floating_point T = float>
struct Ray {
	using value_type = T;

	/**
	 * @brief The origin of the ray.
	 */
	Vec<Dim, T> origin;

	/**
	 * @brief The direction of the ray.
	 * @details
	 * It is recommended that the direction is normalized for most geometric tests.
	 */
	Vec<Dim, T> direction;

	/**
	 * @brief Default constructor.
	 */
	constexpr Ray() noexcept = default;

	/**
	 * @brief Copy constructor.
	 */
	constexpr Ray(Ray const&) noexcept = default;

	/**
	 * @brief Constructs a ray from an origin point and a direction vector.
	 * @param [in] origin    The origin of the ray.
	 * @param [in] direction The direction of the ray.
	 * @details
	 * The direction is not automatically normalized.
	 */
	constexpr Ray(Vec<Dim, T> origin, Vec<Dim, T> direction) noexcept
	    : origin(origin), direction(direction)
	{
	}

	/**
	 * @brief Converting constructor from a ray with a different scalar type.
	 * @tparam U     The scalar type of the other ray.
	 * @param [in] other The other ray.
	 */
	template <std::convertible_to<T> U>
	constexpr explicit Ray(Ray<Dim, U> const& other) noexcept
	    : origin(other.origin), direction(other.direction)
	{
	}

	/**
	 * @brief Returns the dimensionality of the ray.
	 * @return The dimensionality.
	 */
	[[nodiscard]] static constexpr std::size_t dimension() noexcept { return Dim; }

	/**
	 * @brief Returns the point at distance t along the ray.
	 * @param [in] t The distance along the ray.
	 * @return The point at distance t.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> at(T t) const { return origin + t * direction; }

	/**
	 * @brief Equality operator.
	 */
	[[nodiscard]] bool operator==(Ray const&) const = default;

	/**
	 * @brief Returns whether the ray is degenerate.
	 * @return true if degenerate, false otherwise.
	 */
	[[nodiscard]] constexpr bool isDegenerate() const noexcept
	{
		return dot(direction, direction) < std::numeric_limits<T>::epsilon();
	}

	/**
	 * @brief Returns the volume of the ray.
	 * @return The volume (always 0).
	 */
	[[nodiscard]] constexpr T volume() const noexcept { return T(0); }

	/**
	 * @brief Returns the diameter of the ray.
	 * @return The diameter (always infinity).
	 */
	[[nodiscard]] constexpr T diameter() const noexcept
	{
		return std::numeric_limits<T>::infinity();
	}

	/**
	 * @brief Returns the AABB of the ray.
	 * @return The AABB.
	 */
	[[nodiscard]] constexpr AABB<Dim, T> aabb() const noexcept
	{
		Vec<Dim, T> min = origin;
		Vec<Dim, T> max = origin;
		for (std::size_t i = 0; i < Dim; ++i) {
			if (direction[i] > T(0)) {
				max[i] = std::numeric_limits<T>::infinity();
			} else if (direction[i] < T(0)) {
				min[i] = -std::numeric_limits<T>::infinity();
			}
		}
		return AABB<Dim, T>(min, max);
	}
};

//
// Deduction guides
//

template <std::size_t Dim, std::floating_point T>
Ray(Vec<Dim, T>, Vec<Dim, T>) -> Ray<Dim, T>;

/**
 * @brief Output stream operator for Ray.
 */
template <std::size_t Dim, std::floating_point T>
std::ostream& operator<<(std::ostream& out, Ray<Dim, T> const& ray)
{
	return out << "Origin: [" << ray.origin << "], Direction: [" << ray.direction << "]";
}

template <std::floating_point T>
using Ray1 = Ray<1, T>;
template <std::floating_point T>
using Ray2 = Ray<2, T>;
template <std::floating_point T>
using Ray3 = Ray<3, T>;
template <std::floating_point T>
using Ray4 = Ray<4, T>;

using Ray1f = Ray<1, float>;
using Ray2f = Ray<2, float>;
using Ray3f = Ray<3, float>;
using Ray4f = Ray<4, float>;

using Ray1d = Ray<1, double>;
using Ray2d = Ray<2, double>;
using Ray3d = Ray<3, double>;
using Ray4d = Ray<4, double>;

}  // namespace ufo

template <std::size_t Dim, std::floating_point T>
  requires std::formattable<T, char>
struct std::formatter<ufo::Ray<Dim, T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::Ray<Dim, T> const& ray, std::format_context& ctx) const
	{
		return std::format_to(ctx.out(), "Origin: [{}], Direction: [{}]", ray.origin,
		                      ray.direction);
	}
};

#endif  // UFO_GEOMETRY_RAY_HPP