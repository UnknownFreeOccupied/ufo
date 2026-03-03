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

#ifndef UFO_GEOMETRY_CYLINDER_HPP
#define UFO_GEOMETRY_CYLINDER_HPP

// UFO
#include <ufo/geometry/aabb.hpp>
#include <ufo/math/vec.hpp>

// STL
#include <concepts>
#include <cstddef>
#include <format>
#include <numbers>
#include <ostream>

namespace ufo
{
/**
 * @struct Cylinder
 * @brief Cylinder in Dim-dimensional space.
 * @tparam Dim The dimensionality of the space (default: 3).
 * @tparam T   The numeric type (default: float), must be a floating-point type.
 * @details
 * Represents a cylinder defined by two center points (caps) and a radius.
 */
template <std::size_t Dim = 3, std::floating_point T = float>
struct Cylinder {
	using value_type = T;

	/**
	 * @brief The center of the first cap of the cylinder.
	 */
	Vec<Dim, T> start;

	/**
	 * @brief The end point of the second cap of the cylinder.
	 */
	Vec<Dim, T> end;

	/**
	 * @brief The radius of the cylinder.
	 */
	T radius{};

	/**
	 * @brief Default constructor.
	 */
	constexpr Cylinder() noexcept = default;

	/**
	 * @brief Constructs a cylinder from two cap centers and a radius.
	 * @param [in] center_1 The center of the first cap.
	 * @param [in] center_2 The center of the second cap.
	 * @param [in] radius   The radius.
	 */
	constexpr Cylinder(Vec<Dim, T> start, Vec<Dim, T> end, T radius) noexcept
	    : start(start), end(end), radius(radius)
	{
	}

	/**
	 * @brief Copy constructor.
	 */
	constexpr Cylinder(Cylinder const&) noexcept = default;

	/**
	 * @brief Converting constructor from a cylinder with a different scalar type.
	 * @tparam U     The scalar type of the other cylinder.
	 * @param [in] other The other cylinder.
	 */
	template <std::convertible_to<T> U>
	constexpr explicit Cylinder(Cylinder<Dim, U> const& other) noexcept
	    : start(Vec<Dim, T>(other.start))
	    , end(Vec<Dim, T>(other.end))
	    , radius(static_cast<T>(other.radius))
	{
	}

	/**
	 * @brief Returns the dimensionality of the cylinder.
	 * @return The dimensionality.
	 */
	[[nodiscard]] static constexpr std::size_t dimension() noexcept { return Dim; }

	/**
	 * @brief Returns the center of the cylinder.
	 * @return The center.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> center() const noexcept
	{
		return (start + end) * T(0.5);
	}

	/**
	 * @brief Returns the diameter of the cylinder.
	 * @return The diameter.
	 */
	[[nodiscard]] constexpr T diameter() const noexcept
	{
		return std::max(radius * T(2), length());
	}

	/**
	 * @brief Returns the point at parameter t along the cylinder axis.
	 * @param [in] t The parameter [0..1].
	 * @return The point at parameter t.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> at(T t) const noexcept
	{
		return start + t * (end - start);
	}

	/**
	 * @brief Returns the height (length) of the cylinder.
	 * @return The height.
	 */
	[[nodiscard]] constexpr T length() const noexcept { return ufo::length(end - start); }

	/**
	 * @brief Returns the AABB of the cylinder.
	 * @return The AABB.
	 */
	[[nodiscard]] constexpr AABB<Dim, T> aabb() const noexcept
	{
		Vec<Dim, T> axis   = end - start;
		T           length = ufo::length(axis);
		if (length < std::numeric_limits<T>::epsilon()) {
			return AABB<Dim, T>(start, radius);
		}
		axis /= length;
		Vec<Dim, T> min_ext;
		Vec<Dim, T> max_ext;
		for (std::size_t i = 0; i < Dim; ++i) {
			T s        = std::sqrt(std::max(T(0), T(1) - axis[i] * axis[i]));
			min_ext[i] = std::min(start[i], end[i]) - radius * s;
			max_ext[i] = std::max(start[i], end[i]) + radius * s;
		}
		return AABB<Dim, T>(min_ext, max_ext);
	}

	/**
	 * @brief Returns whether the cylinder is degenerate.
	 * @return true if degenerate, false otherwise.
	 */
	[[nodiscard]] constexpr bool isDegenerate() const noexcept { return radius < T(0); }

	/**
	 * @brief Returns the volume (or area/length) of the cylinder.
	 * @return The volume.
	 */
	[[nodiscard]] constexpr T volume() const noexcept
	{
		if constexpr (1 == Dim) {
			return length();
		} else if constexpr (2 == Dim) {
			return radius * T(2) * length();
		} else if constexpr (3 == Dim) {
			return std::numbers::pi_v<T> * radius * radius * length();
		} else {
			// Higher dimensions would need the general n-cylinder volume formula
			return T(0);
		}
	}

	/**
	 * @brief Equality operator.
	 * @param [in] other The other cylinder.
	 * @retval true if the cylinders are equal
	 * @retval false otherwise
	 */
	[[nodiscard]] bool operator==(Cylinder const&) const = default;
};

//
// Deduction guides
//

template <std::size_t Dim, std::floating_point T>
Cylinder(Vec<Dim, T>, Vec<Dim, T>, T) -> Cylinder<Dim, T>;

/**
 * @brief Output stream operator for Cylinder.
 * @param [in] out The output stream.
 * @param [in] cylinder The cylinder.
 * @return The output stream.
 */
template <std::size_t Dim, std::floating_point T>
std::ostream& operator<<(std::ostream& out, Cylinder<Dim, T> const& cylinder)
{
	return out << "Start: [" << cylinder.start << "], End: [" << cylinder.end
	           << "], Radius: " << cylinder.radius;
}

template <std::floating_point T>
using Cylinder1 = Cylinder<1, T>;
template <std::floating_point T>
using Cylinder2 = Cylinder<2, T>;
template <std::floating_point T>
using Cylinder3 = Cylinder<3, T>;
template <std::floating_point T>
using Cylinder4 = Cylinder<4, T>;

using Cylinder1f = Cylinder<1, float>;
using Cylinder2f = Cylinder<2, float>;
using Cylinder3f = Cylinder<3, float>;
using Cylinder4f = Cylinder<4, float>;

using Cylinder1d = Cylinder<1, double>;
using Cylinder2d = Cylinder<2, double>;
using Cylinder3d = Cylinder<3, double>;
using Cylinder4d = Cylinder<4, double>;

}  // namespace ufo

template <std::size_t Dim, std::floating_point T>
  requires std::formattable<T, char>
struct std::formatter<ufo::Cylinder<Dim, T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::Cylinder<Dim, T> const& cylinder, std::format_context& ctx) const
	{
		return std::format_to(ctx.out(), "Start: [{}], End: [{}], Radius: {}", cylinder.start,
		                      cylinder.end, cylinder.radius);
	}
};

#endif  // UFO_GEOMETRY_CYLINDER_HPP