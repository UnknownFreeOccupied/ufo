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

#ifndef UFO_GEOMETRY_AABB_HPP
#define UFO_GEOMETRY_AABB_HPP

// UFO
#include <ufo/math/vec.hpp>

// STL
#include <concepts>
#include <cstddef>
#include <format>
#include <ostream>

/**
 * @ingroup geometry
 * @{
 */

namespace ufo
{
/**
 * @struct AABB
 * @brief Axis-Aligned Bounding Box (AABB) in Dim-dimensional space.
 * @tparam Dim The dimensionality of the space (default: 3).
 * @tparam T   The numeric type (default: float), must be a floating-point type.
 * @details
 * Represents a bounding box defined by a minimum and maximum point.
 */
template <std::size_t Dim = 3, std::floating_point T = float>
struct AABB {
	using value_type = T;

	/**
	 * @brief The minimum point.
	 */
	Vec<Dim, T> min;

	/**
	 * @brief The maximum point.
	 */
	Vec<Dim, T> max;

	/**
	 * @brief Default constructor.
	 */
	constexpr AABB() noexcept = default;

	/**
	 * @brief Constructs an AABB from a minimum and maximum point.
	 *
	 * @param [in] min The minimum point.
	 * @param [in] max The maximum point.
	 */
	constexpr AABB(Vec<Dim, T> const& min, Vec<Dim, T> const& max) noexcept
	    : min(min), max(max)
	{
	}

	/**
	 * @brief Constructs an AABB from a center point and a half-length.
	 *
	 * @param [in] center The center point.
	 * @param [in] half_length The half-length.
	 */
	constexpr AABB(Vec<Dim, T> const& center, T half_length) noexcept
	    : min(center - half_length), max(center + half_length)
	{
	}

	/**
	 * @brief Copy constructor.
	 */
	constexpr AABB(AABB const&) noexcept = default;

	/**
	 * @brief Converting constructor from an AABB with a different scalar type.
	 *
	 * @tparam U The scalar type of the other AABB.
	 * @param [in] other The other AABB.
	 */
	template <std::convertible_to<T> U>
	constexpr explicit AABB(AABB<Dim, U> const& other) noexcept
	    : min(other.min), max(other.max)
	{
	}

	/**
	 * @brief Returns the dimensionality of the AABB.
	 * @return The dimensionality.
	 */
	[[nodiscard]] static constexpr std::size_t dimension() noexcept { return Dim; }

	/**
	 * @brief Returns the center of the AABB.
	 * @return The center.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> center() const noexcept
	{
		return min + halfLength();
	}

	/**
	 * @brief Returns the length (extent) of the AABB in each dimension.
	 * @return The length.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> length() const noexcept { return max - min; }

	/**
	 * @brief Returns the half-length (extent) of the AABB in each dimension.
	 * @return The half-length.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> halfLength() const noexcept
	{
		return length() * T(0.5);
	}

	/**
	 * @brief Returns the diagonal length of the AABB.
	 * @return The diagonal length.
	 */
	[[nodiscard]] constexpr T diagonal() const noexcept { return ufo::length(length()); }

	/**
	 * @brief Returns the volume of the AABB.
	 * @return The volume.
	 */
	[[nodiscard]] constexpr T volume() const noexcept
	{
		auto l = length();
		T    v = l[0];
		for (std::size_t i = 1; i < Dim; ++i) {
			v *= l[i];
		}
		return v;
	}

	/**
	 * @brief Returns the AABB of the AABB.
	 * @return The AABB.
	 */
	[[nodiscard]] constexpr AABB aabb() const noexcept { return *this; }

	/**
	 * @brief Returns whether the AABB is degenerate.
	 * @return true if degenerate, false otherwise.
	 */
	[[nodiscard]] constexpr bool isDegenerate() const noexcept
	{
		return any(greaterThan(min, max));
	}

	/**
	 * @brief Equality operator.
	 * @param [in] other The other AABB.
	 * @retval true if the AABBs are equal
	 * @retval false otherwise
	 */
	[[nodiscard]] bool operator==(AABB const&) const = default;
};

//
// Deduction guides
//

template <std::size_t Dim, std::floating_point T>
AABB(Vec<Dim, T>, Vec<Dim, T>) -> AABB<Dim, T>;

template <std::size_t Dim, std::floating_point T>
AABB(Vec<Dim, T>, T) -> AABB<Dim, T>;

/**
 * @brief Output stream operator for AABB.
 * @param [in] out The output stream.
 * @param [in] aabb The AABB.
 * @return The output stream.
 */
template <std::size_t Dim, std::floating_point T>
std::ostream& operator<<(std::ostream& out, AABB<Dim, T> const& aabb)
{
	return out << "Min: [" << aabb.min << "], Max: [" << aabb.max << "]";
}

template <std::floating_point T>
using AABB1 = AABB<1, T>;
template <std::floating_point T>
using AABB2 = AABB<2, T>;
template <std::floating_point T>
using AABB3 = AABB<3, T>;
template <std::floating_point T>
using AABB4 = AABB<4, T>;

using AABB1f = AABB<1, float>;
using AABB2f = AABB<2, float>;
using AABB3f = AABB<3, float>;
using AABB4f = AABB<4, float>;

using AABB1d = AABB<1, double>;
using AABB2d = AABB<2, double>;
using AABB3d = AABB<3, double>;
using AABB4d = AABB<4, double>;

}  // namespace ufo

/**
 * @}
 */

template <std::size_t Dim, std::floating_point T>
  requires std::formattable<T, char>
struct std::formatter<ufo::AABB<Dim, T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::AABB<Dim, T> const& aabb, std::format_context& ctx) const
	{
		return std::format_to(ctx.out(), "Min: [{}], Max: [{}]", aabb.min, aabb.max);
	}
};

#endif  // UFO_GEOMETRY_AABB_HPP