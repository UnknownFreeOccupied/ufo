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

#ifndef UFO_GEOMETRY_CAPSULE_HPP
#define UFO_GEOMETRY_CAPSULE_HPP

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
 * @struct Capsule
 * @brief Capsule in Dim-dimensional space.
 * @tparam Dim The dimensionality of the space (default: 3).
 * @tparam T   The numeric type (default: float), must be a floating-point type.
 * @details
 * Represents a capsule defined by a line segment between two points and a radius.
 */
template <std::size_t Dim = 3, std::floating_point T = float>
struct Capsule {
	using value_type = T;

	/**
	 * @brief The start point of the capsule segment.
	 */
	Vec<Dim, T> start;

	/**
	 * @brief The end point of the capsule segment.
	 */
	Vec<Dim, T> end;

	/**
	 * @brief The radius of the capsule.
	 */
	T radius{};

	/**
	 * @brief Default constructor.
	 */
	constexpr Capsule() noexcept = default;

	/**
	 * @brief Constructs a capsule from two points and a radius.
	 * @param [in] start  The start point of the segment.
	 * @param [in] end    The end point of the segment.
	 * @param [in] radius The radius of the capsule.
	 */
	constexpr Capsule(Vec<Dim, T> const& start, Vec<Dim, T> const& end, T radius) noexcept
	    : start(start), end(end), radius(radius)
	{
	}

	/**
	 * @brief Copy constructor.
	 */
	constexpr Capsule(Capsule const&) noexcept = default;

	/**
	 * @brief Converting constructor from a capsule with a different scalar type.
	 * @tparam U     The scalar type of the other capsule.
	 * @param [in] other The other capsule.
	 */
	template <std::convertible_to<T> U>
	constexpr explicit Capsule(Capsule<Dim, U> const& other) noexcept
	    : start(Vec<Dim, T>(other.start))
	    , end(Vec<Dim, T>(other.end))
	    , radius(static_cast<T>(other.radius))
	{
	}

	/**
	 * @brief Returns the dimensionality of the capsule.
	 * @return The dimensionality.
	 */
	[[nodiscard]] static constexpr std::size_t dimension() noexcept { return Dim; }

	/**
	 * @brief Returns the center of the capsule.
	 * @return The center.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> center() const noexcept
	{
		return (start + end) * T(0.5);
	}

	/**
	 * @brief Returns the length of the segment between start and end.
	 * @return The length of the segment.
	 */
	[[nodiscard]] constexpr T length() const noexcept { return ufo::length(end - start); }

	/**
	 * @brief Returns the AABB of the capsule.
	 * @return The AABB.
	 */
	[[nodiscard]] constexpr AABB<Dim, T> aabb() const noexcept
	{
		return AABB<Dim, T>(ufo::min(start, end) - radius, ufo::max(start, end) + radius);
	}

	/**
	 * @brief Returns whether the capsule is degenerate.
	 * @return true if degenerate, false otherwise.
	 */
	[[nodiscard]] constexpr bool isDegenerate() const noexcept { return radius < T(0); }

	/**
	 * @brief Returns the diameter of the capsule.
	 * @return The diameter.
	 */
	[[nodiscard]] constexpr T diameter() const noexcept { return radius * T(2); }

	/**
	 * @brief Returns the volume (or area/length) of the capsule.
	 * @return The volume.
	 */
	[[nodiscard]] constexpr T volume() const noexcept
	{
		T h = length();
		if constexpr (1 == Dim) {
			return h + diameter();
		} else if constexpr (2 == Dim) {
			return diameter() * h + std::numbers::pi_v<T> * radius * radius;
		} else if constexpr (3 == Dim) {
			T r2 = radius * radius;
			return std::numbers::pi_v<T> * r2 * h +
			       (T(4) / T(3)) * std::numbers::pi_v<T> * r2 * radius;
		} else {
			// TODO: Add 4D if requested
			return T(0);
		}
	}

	/**
	 * @brief Equality operator.
	 */
	[[nodiscard]] bool operator==(Capsule const&) const = default;
};

//
// Deduction guides
//

template <std::size_t Dim, std::floating_point T>
Capsule(Vec<Dim, T>, Vec<Dim, T>, T) -> Capsule<Dim, T>;

/**
 * @brief Output stream operator for Capsule.
 */
template <std::size_t Dim, std::floating_point T>
std::ostream& operator<<(std::ostream& out, Capsule<Dim, T> const& capsule)
{
	return out << "Start: [" << capsule.start << "], End: [" << capsule.end
	           << "], Radius: " << capsule.radius;
}

template <std::floating_point T>
using Capsule1 = Capsule<1, T>;
template <std::floating_point T>
using Capsule2 = Capsule<2, T>;
template <std::floating_point T>
using Capsule3 = Capsule<3, T>;
template <std::floating_point T>
using Capsule4 = Capsule<4, T>;

using Capsule1f = Capsule<1, float>;
using Capsule2f = Capsule<2, float>;
using Capsule3f = Capsule<3, float>;
using Capsule4f = Capsule<4, float>;

using Capsule1d = Capsule<1, double>;
using Capsule2d = Capsule<2, double>;
using Capsule3d = Capsule<3, double>;
using Capsule4d = Capsule<4, double>;

}  // namespace ufo

template <std::size_t Dim, std::floating_point T>
  requires std::formattable<T, char>
struct std::formatter<ufo::Capsule<Dim, T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::Capsule<Dim, T> const& capsule, std::format_context& ctx) const
	{
		return std::format_to(ctx.out(), "Start: [{}], End: [{}], Radius: {}", capsule.start,
		                      capsule.end, capsule.radius);
	}
};

#endif  // UFO_GEOMETRY_CAPSULE_HPP
