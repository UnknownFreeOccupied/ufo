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

#ifndef UFO_GEOMETRY_LINE_SEGMENT_HPP
#define UFO_GEOMETRY_LINE_SEGMENT_HPP

// UFO
#include <ufo/geometry/aabb.hpp>
#include <ufo/math/vec.hpp>

// STL
#include <concepts>
#include <cstddef>
#include <format>
#include <ostream>

namespace ufo
{
/**
 * @struct LineSegment
 * @brief Line segment in Dim-dimensional space.
 *
 * @details
 * Represents a line segment defined by a start and an end point.
 *
 * @tparam Dim The dimensionality of the space (default: 3).
 * @tparam T   The numeric type (default: float), must be a floating-point type.
 */
template <std::size_t Dim = 3, std::floating_point T = float>
struct LineSegment {
	using value_type = T;

	/**
	 * @brief The start point of the line segment.
	 */
	Vec<Dim, T> start;

	/**
	 * @brief The end point of the line segment.
	 */
	Vec<Dim, T> end;

	/**
	 * @brief Default constructor.
	 */
	constexpr LineSegment() noexcept = default;

	/**
	 * @brief Constructs a line segment from a start and an end point.
	 *
	 * @param [in] start The start point.
	 * @param [in] end   The end point.
	 */
	constexpr LineSegment(Vec<Dim, T> const& start, Vec<Dim, T> const& end) noexcept
	    : start(start), end(end)
	{
	}

	/**
	 * @brief Copy constructor.
	 */
	constexpr LineSegment(LineSegment const&) noexcept = default;

	/**
	 * @brief Converting constructor from a line segment with a different scalar type.
	 *
	 * @tparam U     The scalar type of the other segment.
	 * @param [in] other The other segment.
	 */
	template <std::convertible_to<T> U>
	constexpr explicit LineSegment(LineSegment<Dim, U> const& other) noexcept
	    : start(Vec<Dim, T>(other.start)), end(Vec<Dim, T>(other.end))
	{
	}

	/**
	 * @brief Returns the dimensionality of the line segment.
	 * @return The dimensionality.
	 */
	[[nodiscard]] static constexpr std::size_t dimension() noexcept { return Dim; }

	/**
	 * @brief Returns the length of the line segment.
	 *
	 * @return The length.
	 */
	[[nodiscard]] constexpr T length() const noexcept { return norm(end - start); }

	/**
	 * @brief Returns the point at parameter t along the line segment.
	 *
	 * @param [in] t The parameter [0..1].
	 * @return The point at parameter t.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> at(T t) const noexcept
	{
		return start + t * (end - start);
	}

	/**
	 * @brief Returns the center of the line segment.
	 * @return The center.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> center() const noexcept
	{
		return (start + end) * T(0.5);
	}

	/**
	 * @brief Returns the diameter of the line segment.
	 * @return The length of the segment.
	 */
	[[nodiscard]] constexpr T diameter() const noexcept { return length(); }

	/**
	 * @brief Returns whether the line segment is degenerate.
	 * @param [in] eps The epsilon value for the check.
	 * @return true if degenerate, false otherwise.
	 */
	[[nodiscard]] constexpr bool isDegenerate(
	    T eps = std::numeric_limits<T>::epsilon()) const noexcept
	{
		return length() <= eps;
	}

	/**
	 * @brief Returns the AABB of the line segment.
	 * @return The AABB.
	 */
	[[nodiscard]] constexpr AABB<Dim, T> aabb() const noexcept
	{
		return AABB<Dim, T>(ufo::min(start, end), ufo::max(start, end));
	}

	/**
	 * @brief Returns the volume of the line segment.
	 * @return The volume (always 0).
	 */
	[[nodiscard]] constexpr T volume() const noexcept { return T(0); }

	/**
	 * @brief Equality operator.
	 */
	[[nodiscard]] bool operator==(LineSegment const&) const = default;
};

//
// Deduction guides
//

template <std::size_t Dim, std::floating_point T>
LineSegment(Vec<Dim, T>, Vec<Dim, T>) -> LineSegment<Dim, T>;

/**
 * @brief Output stream operator for LineSegment.
 */
template <std::size_t Dim, std::floating_point T>
std::ostream& operator<<(std::ostream& out, LineSegment<Dim, T> const& ls)
{
	return out << "Start: [" << ls.start << "], End: [" << ls.end << "]";
}

template <std::floating_point T>
using LineSegment1 = LineSegment<1, T>;
template <std::floating_point T>
using LineSegment2 = LineSegment<2, T>;
template <std::floating_point T>
using LineSegment3 = LineSegment<3, T>;
template <std::floating_point T>
using LineSegment4 = LineSegment<4, T>;

using LineSegment1f = LineSegment<1, float>;
using LineSegment2f = LineSegment<2, float>;
using LineSegment3f = LineSegment<3, float>;
using LineSegment4f = LineSegment<4, float>;

using LineSegment1d = LineSegment<1, double>;
using LineSegment2d = LineSegment<2, double>;
using LineSegment3d = LineSegment<3, double>;
using LineSegment4d = LineSegment<4, double>;

}  // namespace ufo

template <std::size_t Dim, std::floating_point T>
  requires std::formattable<T, char>
struct std::formatter<ufo::LineSegment<Dim, T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::LineSegment<Dim, T> const& ls, std::format_context& ctx) const
	{
		return std::format_to(ctx.out(), "Start: [{}], End: [{}]", ls.start, ls.end);
	}
};

#endif  // UFO_GEOMETRY_LINE_SEGMENT_HPP