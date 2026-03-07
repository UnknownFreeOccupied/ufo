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

#ifndef UFO_GEOMETRY_ELLIPSOID_HPP
#define UFO_GEOMETRY_ELLIPSOID_HPP

// UFO
#include <ufo/geometry/aabb.hpp>
#include <ufo/numeric/vec.hpp>

// STL
#include <concepts>
#include <cstddef>
#include <format>
#include <numbers>
#include <ostream>

namespace ufo
{
/**
 * @struct Ellipsoid
 * @brief Ellipsoid in Dim-dimensional space.
 * @tparam Dim The dimensionality of the space (default: 3).
 * @tparam T   The numeric type (default: float), must be a floating-point type.
 * @details
 * Represents an ellipsoid defined by a center point and radii along each axis.
 */
template <std::size_t Dim = 3, std::floating_point T = float>
struct Ellipsoid {
	using value_type = T;

	/**
	 * @brief The center of the ellipsoid.
	 */
	Vec<Dim, T> center_;

	/**
	 * @brief The radii of the ellipsoid along each axis.
	 */
	Vec<Dim, T> radii;

	/**
	 * @brief Default constructor.
	 */
	constexpr Ellipsoid() noexcept = default;

	/**
	 * @brief Constructs an ellipsoid from a center and radii.
	 * @param [in] center The center point.
	 * @param [in] radii The radii along each axis.
	 */
	constexpr Ellipsoid(Vec<Dim, T> center, Vec<Dim, T> radii) noexcept
	    : center_(center), radii(radii)
	{
	}

	/**
	 * @brief Copy constructor.
	 */
	constexpr Ellipsoid(Ellipsoid const&) noexcept = default;

	/**
	 * @brief Converting constructor from an ellipsoid with a different scalar type.
	 * @tparam U     The scalar type of the other ellipsoid.
	 * @param [in] other The other ellipsoid.
	 */
	template <std::convertible_to<T> U>
	constexpr explicit Ellipsoid(Ellipsoid<Dim, U> const& other) noexcept
	    : center_(Vec<Dim, T>(other.center())), radii(Vec<Dim, T>(other.radii))
	{
	}

	/**
	 * @brief Returns the center of the ellipsoid.
	 * @return The center.
	 */
	[[nodiscard]] constexpr Vec<Dim, T>& center() noexcept { return center_; }

	/**
	 * @brief Returns the center of the ellipsoid.
	 * @return The center.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> const& center() const noexcept { return center_; }

	/**
	 * @brief Returns the dimensionality of the ellipsoid.
	 * @return The dimensionality.
	 */
	[[nodiscard]] static constexpr std::size_t dimension() noexcept { return Dim; }

	/**
	 * @brief Returns the minimum point of the ellipsoid's AABB.
	 * @return The minimum point.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> min() const noexcept { return center_ - radii; }

	/**
	 * @brief Returns the maximum point of the ellipsoid's AABB.
	 * @return The maximum point.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> max() const noexcept { return center_ + radii; }

	/**
	 * @brief Returns the AABB of the ellipsoid.
	 * @return The AABB.
	 */
	[[nodiscard]] constexpr AABB<Dim, T> aabb() const noexcept
	{
		return AABB<Dim, T>(center_, radii);
	}

	/**
	 * @brief Returns whether the ellipsoid is degenerate.
	 * @return true if degenerate, false otherwise.
	 */
	[[nodiscard]] constexpr bool isDegenerate() const noexcept
	{
		return any(lessThan(radii, Vec<Dim, T>()));
	}

	/**
	 * @brief Returns the diameter of the ellipsoid.
	 * @return The diameter (twice the maximum radius).
	 */
	[[nodiscard]] constexpr T diameter() const noexcept { return T(2) * ufo::max(radii); }

	/**
	 * @brief Returns the volume of the ellipsoid.
	 * @return The volume.
	 */
	[[nodiscard]] constexpr T volume() const noexcept
	{
		if constexpr (1 == Dim) {
			return radii[0] * T(2);
		} else if constexpr (2 == Dim) {
			return std::numbers::pi_v<T> * radii[0] * radii[1];
		} else if constexpr (3 == Dim) {
			return (T(4) / T(3)) * std::numbers::pi_v<T> * radii[0] * radii[1] * radii[2];
		} else if constexpr (4 == Dim) {
			return (std::numbers::pi_v<T> * std::numbers::pi_v<T> * radii[0] * radii[1] *
			        radii[2] * radii[3]) /
			       T(2);
		} else {
			// For higher dimensions, it would need a more general formula (Gamma function)
			return T(0);
		}
	}

	/**
	 * @brief Equality operator.
	 * @param [in] other The other ellipsoid.
	 * @retval true if the ellipsoids are equal
	 * @retval false otherwise
	 */
	[[nodiscard]] bool operator==(Ellipsoid const&) const = default;
};

//
// Deduction guides
//

template <std::size_t Dim, std::floating_point T>
Ellipsoid(Vec<Dim, T>, Vec<Dim, T>) -> Ellipsoid<Dim, T>;

/**
 * @brief Output stream operator for Ellipsoid.
 * @param [in] out The output stream.
 * @param [in] ellipsoid The ellipsoid.
 * @return The output stream.
 */
template <std::size_t Dim, std::floating_point T>
std::ostream& operator<<(std::ostream& out, Ellipsoid<Dim, T> const& ellipsoid)
{
	return out << "Center: [" << ellipsoid.center() << "], radii: [" << ellipsoid.radii
	           << "]";
}

template <std::floating_point T>
using Ellipsoid1 = Ellipsoid<1, T>;
template <std::floating_point T>
using Ellipsoid2 = Ellipsoid<2, T>;
template <std::floating_point T>
using Ellipsoid3 = Ellipsoid<3, T>;
template <std::floating_point T>
using Ellipsoid4 = Ellipsoid<4, T>;

using Ellipsoid1f = Ellipsoid<1, float>;
using Ellipsoid2f = Ellipsoid<2, float>;
using Ellipsoid3f = Ellipsoid<3, float>;
using Ellipsoid4f = Ellipsoid<4, float>;

using Ellipsoid1d = Ellipsoid<1, double>;
using Ellipsoid2d = Ellipsoid<2, double>;
using Ellipsoid3d = Ellipsoid<3, double>;
using Ellipsoid4d = Ellipsoid<4, double>;

}  // namespace ufo

template <std::size_t Dim, std::floating_point T>
  requires std::formattable<T, char>
struct std::formatter<ufo::Ellipsoid<Dim, T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::Ellipsoid<Dim, T> const& ellipsoid, std::format_context& ctx) const
	{
		return std::format_to(ctx.out(), "Center: [{}], radii: [{}]", ellipsoid.center(),
		                      ellipsoid.radii);
	}
};

#endif  // UFO_GEOMETRY_ELLIPSOID_HPP
