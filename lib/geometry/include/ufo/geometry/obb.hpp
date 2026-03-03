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

#ifndef UFO_GEOMETRY_OBB_HPP
#define UFO_GEOMETRY_OBB_HPP

// UFO
#include <ufo/geometry/aabb.hpp>
#include <ufo/math/mat.hpp>
#include <ufo/math/quat.hpp>
#include <ufo/math/vec.hpp>

// STL
#include <cmath>
#include <concepts>
#include <cstddef>
#include <format>
#include <ostream>

namespace ufo
{
template <std::size_t Dim = 3, std::floating_point T = float>
struct OBB;

/**
 * @struct OBB
 * @brief Oriented Bounding Box in 2D space.
 *
 * @tparam T The numeric type (default: float), must be a floating-point type.
 */
template <std::floating_point T>
struct OBB<2, T> {
	using value_type = T;

	/**
	 * @brief The center of the OBB.
	 */
	Vec<2, T> center_;

	/**
	 * @brief The half-lengths of the OBB along its local axes.
	 */
	Vec<2, T> half_length;

	/**
	 * @brief The rotation matrix of the OBB.
	 */
	Mat<2, 2, T> rotation;

	/**
	 * @brief Default constructor.
	 */
	constexpr OBB() noexcept = default;

	/**
	 * @brief Constructs an OBB from a line segment and a half-thickness.
	 *
	 * @param [in] start       The start of the segment.
	 * @param [in] end         The end of the segment.
	 * @param [in] half_length The half-thickness (1D vector).
	 */
	constexpr OBB(Vec<2, T> const& start, Vec<2, T> const& end,
	              Vec<1, T> const& half_length)
	    : center_((start + end) * T(0.5))
	{
		auto dir = end - start;

		this->half_length = Vec<2, T>(norm(dir) * T(0.5), half_length[0]);

		auto theta     = std::atan2(dir.y, dir.x);
		auto cos_theta = std::cos(theta);
		auto sin_theta = std::sin(theta);

		rotation[0][0] = cos_theta;
		rotation[0][1] = sin_theta;
		rotation[1][0] = -sin_theta;
		rotation[1][1] = cos_theta;
	}

	/**
	 * @brief Constructs an OBB from a line segment and a half-thickness.
	 *
	 * @param [in] start       The start of the segment.
	 * @param [in] end         The end of the segment.
	 * @param [in] half_length The half-thickness (scalar).
	 */
	constexpr OBB(Vec<2, T> const& start, Vec<2, T> const& end, T const& half_length)
	    : OBB(start, end, Vec<1, T>(half_length))
	{
	}

	/**
	 * @brief Constructs an OBB from center and half-lengths with identity rotation.
	 *
	 * @param [in] center      The center.
	 * @param [in] half_length The half-lengths.
	 */
	constexpr OBB(Vec<2, T> const& center, Vec<2, T> const& half_length) noexcept
	    : center_(center), half_length(half_length)
	{
	}

	/**
	 * @brief Constructs an OBB from center, half-lengths, and rotation.
	 *
	 * @param [in] center      The center.
	 * @param [in] half_length The half-lengths.
	 * @param [in] rotation    The rotation matrix.
	 */
	constexpr OBB(Vec<2, T> const& center, Vec<2, T> const& half_length,
	              Mat<2, 2, T> const& rotation) noexcept
	    : center_(center), half_length(half_length), rotation(rotation)
	{
	}

	/**
	 * @brief Copy constructor.
	 */
	constexpr OBB(OBB const&) noexcept = default;

	/**
	 * @brief Converting constructor from an OBB with a different scalar type.
	 *
	 * @tparam U     The scalar type of the other OBB.
	 * @param [in] other The other OBB.
	 */
	template <std::convertible_to<T> U>
	constexpr explicit OBB(OBB<2, U> const& other) noexcept
	    : center_(Vec<2, T>(other.center()))
	    , half_length(Vec<2, T>(other.half_length))
	    , rotation(Mat<2, 2, T>(other.rotation))
	{
	}

	/**
	 * @brief Returns the center of the OBB.
	 * @return The center.
	 */
	[[nodiscard]] constexpr Vec<2, T>& center() noexcept { return center_; }

	/**
	 * @brief Returns the center of the OBB.
	 * @return The center.
	 */
	[[nodiscard]] constexpr Vec<2, T> const& center() const noexcept { return center_; }

	/**
	 * @brief Returns the dimensionality of the OBB.
	 * @return The dimensionality.
	 */
	[[nodiscard]] static constexpr std::size_t dimension() noexcept { return 2; }

	/**
	 * @brief Returns the distance from the center to a corner.
	 *
	 * @return The diagonal length.
	 */
	[[nodiscard]] constexpr T diagonal() const noexcept { return norm(half_length) * T(2); }

	/**
	 * @brief Returns the AABB of the OBB.
	 * @return The AABB.
	 */
	[[nodiscard]] constexpr AABB<2, T> aabb() const noexcept
	{
		Vec<2, T> extent;
		for (std::size_t i = 0; i < 2; ++i) {
			extent[i] = std::abs(rotation[0][i]) * half_length[0] +
			            std::abs(rotation[1][i]) * half_length[1];
		}
		return AABB<2, T>(center_ - extent, center_ + extent);
	}

	/**
	 * @brief Returns whether the OBB is degenerate.
	 */
	[[nodiscard]] constexpr bool isDegenerate() const noexcept
	{
		return any(lessThan(half_length, Vec<2, T>()));
	}

	/**
	 * @brief Returns the area of the OBB.
	 *
	 * @return The area.
	 */
	[[nodiscard]] constexpr T volume() const noexcept
	{
		return T(4) * half_length[0] * half_length[1];
	}

	/**
	 * @brief Returns the rotated half-length vector.
	 */
	[[nodiscard]] constexpr Vec<2, T> rotatedHalfLength() const
	{
		return half_length * rotation;
	}

	/**
	 * @brief Sets the rotation of the OBB.
	 *
	 * @param [in] angle The rotation angle in radians.
	 */
	void setRotation(T angle)
	{
		auto cos_theta = std::cos(angle);
		auto sin_theta = std::sin(angle);

		rotation[0][0] = cos_theta;
		rotation[0][1] = sin_theta;
		rotation[1][0] = -sin_theta;
		rotation[1][1] = cos_theta;
	}

	/**
	 * @brief Equality operator.
	 */
	[[nodiscard]] bool operator==(OBB const&) const = default;
};

/**
 * @struct OBB
 * @brief Oriented Bounding Box in 3D space.
 *
 * @tparam T The numeric type (default: float), must be a floating-point type.
 */
template <std::floating_point T>
struct OBB<3, T> {
	using value_type = T;

	/**
	 * @brief The center of the OBB.
	 */
	Vec<3, T> center_;

	/**
	 * @brief The half-lengths of the OBB along its local axes.
	 */
	Vec<3, T> half_length;

	/**
	 * @brief The rotation matrix of the OBB.
	 */
	Mat<3, 3, T> rotation;

	/**
	 * @brief Default constructor.
	 */
	constexpr OBB() noexcept = default;

	/**
	 * @brief Constructs an OBB from a line segment, half-thicknesses, and an up vector.
	 */
	constexpr OBB(Vec<3, T> const& start, Vec<3, T> const& end,
	              Vec<2, T> const& half_length,
	              Vec<3, T> const& up = Vec<3, T>(T(0), T(0), T(1)))
	    : center_((start + end) * T(0.5))
	{
		auto dir = end - start;

		this->half_length = Vec<3, T>(norm(dir) * T(0.5), half_length);

		// Similar to right handed lookAt
		Vec<3, T> const f(normalize(dir));
		Vec<3, T> const s(normalize(cross(f, up)));
		Vec<3, T> const u(cross(s, f));

		rotation[0][0] = s.x;
		rotation[1][0] = s.y;
		rotation[2][0] = s.z;
		rotation[0][1] = u.x;
		rotation[1][1] = u.y;
		rotation[2][1] = u.z;
		rotation[0][2] = -f.x;
		rotation[1][2] = -f.y;
		rotation[2][2] = -f.z;
	}

	/**
	 * @brief Constructs an OBB from center and half-lengths with identity rotation.
	 */
	constexpr OBB(Vec<3, T> const& center, Vec<3, T> const& half_length) noexcept
	    : center_(center), half_length(half_length)
	{
	}

	/**
	 * @brief Constructs an OBB from center, half-lengths, and rotation matrix.
	 */
	constexpr OBB(Vec<3, T> const& center, Vec<3, T> const& half_length,
	              Mat<3, 3, T> const& rotation) noexcept
	    : center_(center), half_length(half_length), rotation(rotation)
	{
	}

	/**
	 * @brief Constructs an OBB from center, half-lengths, and quaternion rotation.
	 */
	constexpr OBB(Vec<3, T> const& center, Vec<3, T> const& half_length,
	              Quat<T> const& rotation) noexcept
	    : center_(center), half_length(half_length), rotation(rotation)
	{
	}

	/**
	 * @brief Copy constructor.
	 */
	constexpr OBB(OBB const&) noexcept = default;

	/**
	 * @brief Converting constructor from an OBB with a different scalar type.
	 */
	template <std::convertible_to<T> U>
	constexpr explicit OBB(OBB<3, U> const& other) noexcept
	    : center_(Vec<3, T>(other.center()))
	    , half_length(Vec<3, T>(other.half_length))
	    , rotation(Mat<3, 3, T>(other.rotation))
	{
	}

	/**
	 * @brief Returns the center of the OBB.
	 * @return The center.
	 */
	[[nodiscard]] constexpr Vec<3, T>& center() noexcept { return center_; }

	/**
	 * @brief Returns the center of the OBB.
	 * @return The center.
	 */
	[[nodiscard]] constexpr Vec<3, T> const& center() const noexcept { return center_; }

	/**
	 * @brief Returns the dimensionality of the OBB.
	 * @return The dimensionality.
	 */
	[[nodiscard]] static constexpr std::size_t dimension() noexcept { return 3; }

	/**
	 * @brief Returns the diagonal length of the OBB.
	 */
	[[nodiscard]] constexpr T diagonal() const noexcept { return norm(half_length) * T(2); }

	/**
	 * @brief Returns the volume of the OBB.
	 */
	[[nodiscard]] constexpr T volume() const noexcept
	{
		return T(8) * half_length[0] * half_length[1] * half_length[2];
	}

	/**
	 * @brief Returns the AABB of the OBB.
	 * @return The AABB.
	 */
	[[nodiscard]] constexpr AABB<3, T> aabb() const noexcept
	{
		Vec<3, T> extent;
		for (std::size_t i = 0; i < 3; ++i) {
			extent[i] = std::abs(rotation[0][i]) * half_length[0] +
			            std::abs(rotation[1][i]) * half_length[1] +
			            std::abs(rotation[2][i]) * half_length[2];
		}
		return AABB<3, T>(center_ - extent, center_ + extent);
	}

	/**
	 * @brief Returns whether the OBB is degenerate.
	 */
	[[nodiscard]] constexpr bool isDegenerate() const noexcept
	{
		return any(lessThan(half_length, Vec<3, T>()));
	}

	/**
	 * @brief Returns the rotated half-length vector.
	 */
	[[nodiscard]] constexpr Vec<3, T> rotatedHalfLength() const
	{
		return half_length * rotation;
	}

	/**
	 * @brief Sets the rotation of the OBB from a quaternion.
	 */
	void setRotation(Quat<T> const& rotation) { this->rotation = rotation; }

	/**
	 * @brief Equality operator.
	 */
	[[nodiscard]] bool operator==(OBB const&) const = default;
};

/**
 * @struct OBB
 * @brief Oriented Bounding Box in 4D space.
 */
template <std::floating_point T>
struct OBB<4, T> {
	using value_type = T;

	Vec<4, T>    center_;
	Vec<4, T>    half_length;
	Mat<4, 4, T> rotation;

	constexpr OBB() noexcept = default;

	constexpr OBB(Vec<4, T> const& start, Vec<4, T> const& end,
	              Vec<3, T> const& half_length)
	{
		auto dir          = end - start;
		center_           = (start + end) * T(0.5);
		this->half_length = Vec<4, T>(norm(dir) * T(0.5), half_length);
		// TODO: Implement rotation
	}

	constexpr OBB(Vec<4, T> const& center, Vec<4, T> const& half_length) noexcept
	    : center_(center), half_length(half_length)
	{
	}

	constexpr OBB(Vec<4, T> const& center, Vec<4, T> const& half_length,
	              Mat<4, 4, T> const& rotation) noexcept
	    : center_(center), half_length(half_length), rotation(rotation)
	{
	}

	constexpr OBB(OBB const&) noexcept = default;

	template <std::convertible_to<T> U>
	constexpr explicit OBB(OBB<4, U> const& other) noexcept
	    : center_(Vec<4, T>(other.center()))
	    , half_length(Vec<4, T>(other.half_length))
	    , rotation(Mat<4, 4, T>(other.rotation))
	{
	}

	/**
	 * @brief Returns the center of the OBB.
	 * @return The center.
	 */
	[[nodiscard]] constexpr Vec<4, T>& center() noexcept { return center_; }

	/**
	 * @brief Returns the center of the OBB.
	 * @return The center.
	 */
	[[nodiscard]] constexpr Vec<4, T> const& center() const noexcept { return center_; }

	/**
	 * @brief Returns the dimensionality of the OBB.
	 * @return The dimensionality.
	 */
	[[nodiscard]] static constexpr std::size_t dimension() noexcept { return 4; }

	/**
	 * @brief Returns the diagonal length of the OBB.
	 */
	[[nodiscard]] constexpr T diagonal() const noexcept { return norm(half_length) * T(2); }

	/**
	 * @brief Returns the volume of the OBB.
	 */
	[[nodiscard]] constexpr T volume() const noexcept
	{
		return T(16) * half_length[0] * half_length[1] * half_length[2] * half_length[3];
	}

	/**
	 * @brief Returns the AABB of the OBB.
	 * @return The AABB.
	 */
	[[nodiscard]] constexpr AABB<4, T> aabb() const noexcept
	{
		Vec<4, T> extent;
		for (std::size_t i = 0; i < 4; ++i) {
			extent[i] = std::abs(rotation[0][i]) * half_length[0] +
			            std::abs(rotation[1][i]) * half_length[1] +
			            std::abs(rotation[2][i]) * half_length[2] +
			            std::abs(rotation[3][i]) * half_length[3];
		}
		return AABB<4, T>(center_ - extent, center_ + extent);
	}

	/**
	 * @brief Returns whether the OBB is degenerate.
	 */
	[[nodiscard]] constexpr bool isDegenerate() const noexcept
	{
		return any(lessThan(half_length, Vec<4, T>()));
	}

	/**
	 * @brief Returns the rotated half-length vector.
	 */
	[[nodiscard]] constexpr Vec<4, T> rotatedHalfLength() const
	{
		return half_length * rotation;
	}

	/**
	 * @brief Equality operator.
	 */
	[[nodiscard]] bool operator==(OBB const&) const = default;
};

/**
 * @brief Output stream operator for 2D OBB.
 */
template <std::floating_point T>
std::ostream& operator<<(std::ostream& out, OBB<2, T> const& obb)
{
	return out << "Center: [" << obb.center() << "], Half length: [" << obb.half_length
	           << "], Rotation: [" << obb.rotation << "]";
}

/**
 * @brief Output stream operator for 3D OBB.
 */
template <std::floating_point T>
std::ostream& operator<<(std::ostream& out, OBB<3, T> const& obb)
{
	return out << "Center: [" << obb.center() << "], Half length: [" << obb.half_length
	           << "], Rotation: [" << obb.rotation << "]";
}

/**
 * @brief Output stream operator for 4D OBB.
 */
template <std::floating_point T>
std::ostream& operator<<(std::ostream& out, OBB<4, T> const& obb)
{
	return out << "Center: [" << obb.center() << "], Half length: [" << obb.half_length
	           << "], Rotation: [" << obb.rotation << "]";
}

template <class T>
using OBB1 = OBB<1, T>;
template <class T>
using OBB2 = OBB<2, T>;
template <class T>
using OBB3 = OBB<3, T>;
template <class T>
using OBB4 = OBB<4, T>;

using OBB1f = OBB<1, float>;
using OBB2f = OBB<2, float>;
using OBB3f = OBB<3, float>;
using OBB4f = OBB<4, float>;

using OBB1d = OBB<1, double>;
using OBB2d = OBB<2, double>;
using OBB3d = OBB<3, double>;
using OBB4d = OBB<4, double>;

}  // namespace ufo

template <std::size_t Dim, std::floating_point T>
  requires std::formattable<T, char>
struct std::formatter<ufo::OBB<Dim, T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::OBB<Dim, T> const& obb, std::format_context& ctx) const
	{
		return std::format_to(ctx.out(), "Center: [{}], Half length: [{}], Rotation: [{}]",
		                      obb.center(), obb.half_length, obb.rotation);
	}
};

#endif  // UFO_GEOMETRY_OBB_HPP