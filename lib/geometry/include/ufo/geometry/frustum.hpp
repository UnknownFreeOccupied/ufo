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

#ifndef UFO_GEOMETRY_FRUSTUM_HPP
#define UFO_GEOMETRY_FRUSTUM_HPP

#include <array>
#include <ufo/geometry/plane.hpp>
#include <ufo/math/vec.hpp>

// STL
#include <cmath>
#include <concepts>
#include <cstddef>
#include <format>
#include <limits>
#include <ostream>

namespace ufo
{
/**
 * @struct Frustum
 * @brief Frustum in Dim-dimensional space.
 * @tparam Dim The dimensionality of the space (default: 3).
 * @tparam T The numeric type (default: float), must be a floating-point type.
 * @details
 * Represents a frustum defined by 2*Dim planes.
 */
template <std::size_t Dim = 3, std::floating_point T = float>
struct Frustum {
	using value_type = T;

	/**
	 * @brief The planes defining the frustum.
	 *
	 * Standard order:
	 * 0, 1: X-axis (Left, Right)
	 * 2, 3: Y-axis (Bottom, Top) - if Dim >= 2
	 * 4, 5: Z-axis (Near, Far)  - if Dim >= 3
	 */
	std::array<Plane<Dim, T>, 2 * Dim> planes;

	/**
	 * @brief Default constructor.
	 */
	constexpr Frustum() noexcept = default;

	/**
	 * @brief Constructs a 2D frustum from four points.
	 *
	 * @param [in] far_right  The far-right point.
	 * @param [in] far_left   The far-left point.
	 * @param [in] near_left  The near-left point.
	 * @param [in] near_right The near-right point.
	 */
	constexpr Frustum(Vec<2, T> const& far_right, Vec<2, T> const& far_left,
	                  Vec<2, T> const& near_left, Vec<2, T> const& near_right)
	  requires(2 == Dim)
	{
		planes[0] = Plane<Dim, T>(near_left, far_left);    // left
		planes[1] = Plane<Dim, T>(far_right, near_right);  // right
		planes[2] = Plane<Dim, T>(near_right, near_left);  // bottom/near
		planes[3] = Plane<Dim, T>(far_left, far_right);    // top/far
	}

	/**
	 * @brief Constructs a 2D frustum from a camera-like setup.
	 *
	 * @param [in] pos       The position of the eye.
	 * @param [in] target    The target point.
	 * @param [in] fov       The field of view in radians.
	 * @param [in] near_dist The near clip distance.
	 * @param [in] far_dist  The far clip distance.
	 */
	constexpr Frustum(Vec<2, T> const& pos, Vec<2, T> const& target, T fov, T near_dist,
	                  T far_dist)
	  requires(2 == Dim)
	{
		auto      dir = normalize(target - pos);
		Vec<2, T> right_dir(dir.y(), -dir.x());
		auto      half_fov        = fov * T(0.5);
		auto      half_width_near = near_dist * std::tan(half_fov);
		auto      half_width_far  = far_dist * std::tan(half_fov);

		Vec<2, T> far_right  = pos + dir * far_dist + half_width_far * right_dir;
		Vec<2, T> far_left   = pos + dir * far_dist - half_width_far * right_dir;
		Vec<2, T> near_left  = pos + dir * near_dist - half_width_near * right_dir;
		Vec<2, T> near_right = pos + dir * near_dist + half_width_near * right_dir;

		planes[0] = Plane<Dim, T>(near_left, far_left);
		planes[1] = Plane<Dim, T>(far_right, near_right);
		planes[2] = Plane<Dim, T>(near_right, near_left);
		planes[3] = Plane<Dim, T>(far_left, far_right);
	}

	/**
	 * @brief Constructs a 3D frustum from eight points.
	 *
	 * @param [in] far_top_right     The far-top-right point.
	 * @param [in] far_top_left      The far-top-left point.
	 * @param [in] far_bottom_left   The far-bottom-left point.
	 * @param [in] far_bottom_right  The far-bottom-right point.
	 * @param [in] near_top_right    The near-top-right point.
	 * @param [in] near_top_left     The near-top-left point.
	 * @param [in] near_bottom_left  The near-bottom-left point.
	 * @param [in] near_bottom_right The near-bottom-right point.
	 */
	constexpr Frustum(Vec<3, T> const& far_top_right, Vec<3, T> const& far_top_left,
	                  Vec<3, T> const& far_bottom_left, Vec<3, T> const& far_bottom_right,
	                  Vec<3, T> const& near_top_right, Vec<3, T> const& near_top_left,
	                  Vec<3, T> const& near_bottom_left, Vec<3, T> const& near_bottom_right)
	  requires(3 == Dim)
	{
		planes[0] = Plane<Dim, T>(near_top_left, near_bottom_left, far_bottom_left);  // left
		planes[1] =
		    Plane<Dim, T>(near_bottom_right, near_top_right, far_bottom_right);  // right
		planes[2] =
		    Plane<Dim, T>(near_bottom_left, near_bottom_right, far_bottom_right);  // bottom
		planes[3] = Plane<Dim, T>(near_top_right, near_top_left, far_top_left);    // top
		planes[4] = Plane<Dim, T>(near_top_left, near_top_right, near_bottom_right);  // near
		planes[5] = Plane<Dim, T>(far_top_right, far_top_left, far_bottom_left);      // far
	}

	/**
	 * @brief Constructs a 3D frustum from a camera-like setup.
	 *
	 * @param [in] pos            The position of the eye.
	 * @param [in] target         The target point.
	 * @param [in] up             The up vector.
	 * @param [in] vertical_fov   The vertical field of view in radians.
	 * @param [in] horizontal_fov The horizontal field of view in radians.
	 * @param [in] near_distance  The near clip distance.
	 * @param [in] far_distance   The far clip distance.
	 */
	constexpr Frustum(Vec<3, T> const& pos, Vec<3, T> const& target, Vec<3, T> const& up,
	                  T vertical_fov, T horizontal_fov, T near_distance, T far_distance)
	  requires(3 == Dim)
	{
		T ratio = horizontal_fov / vertical_fov;

		T tang        = std::tan(vertical_fov * static_cast<T>(0.5));
		T near_height = near_distance * tang;
		T near_width  = near_height * ratio;
		T far_height  = far_distance * tang;
		T far_width   = far_height * ratio;

		auto Z = normalize(pos - target);
		auto X = normalize(cross(up, Z));
		auto Y = cross(Z, X);

		auto nc = pos - Z * near_distance;
		auto fc = pos - Z * far_distance;

		auto ntl = nc + Y * near_height - X * near_width;
		auto ntr = nc + Y * near_height + X * near_width;
		auto nbl = nc - Y * near_height - X * near_width;
		auto nbr = nc - Y * near_height + X * near_width;

		auto ftl = fc + Y * far_height - X * far_width;
		auto ftr = fc + Y * far_height + X * far_width;
		auto fbl = fc - Y * far_height - X * far_width;
		auto fbr = fc - Y * far_height + X * far_width;

		planes[0] = Plane<Dim, T>(ntl, nbl, fbl);  // left
		planes[1] = Plane<Dim, T>(nbr, ntr, fbr);  // right
		planes[2] = Plane<Dim, T>(nbl, nbr, fbr);  // bottom
		planes[3] = Plane<Dim, T>(ntr, ntl, ftl);  // top
		planes[4] = Plane<Dim, T>(ntl, ntr, nbr);  // near
		planes[5] = Plane<Dim, T>(ftr, ftl, fbl);  // far
	}

	/**
	 * @brief Copy constructor.
	 */
	constexpr Frustum(Frustum const&) noexcept = default;

	/**
	 * @brief Converting constructor from a frustum with a different scalar type.
	 *
	 * @tparam U     The scalar type of the other frustum.
	 * @param [in] other The other frustum.
	 */
	template <std::convertible_to<T> U>
	constexpr explicit Frustum(Frustum<Dim, U> const& other) noexcept
	{
		for (std::size_t i = 0; i < 2 * Dim; ++i) {
			planes[i] = Plane<Dim, T>(other.planes[i]);
		}
	}

	/**
	 * @brief Returns the dimensionality of the frustum.
	 */
	[[nodiscard]] static constexpr std::size_t dimension() noexcept { return Dim; }

	/**
	 * @brief Accesses the plane at position pos.
	 *
	 * @param [in] pos The position of the plane [0..2*Dim-1].
	 * @return A (const) reference to the plane.
	 */
	[[nodiscard]] constexpr auto& operator[](this auto& self, std::size_t pos) noexcept
	{
		return self.planes[pos];
	}

	/**
	 * @brief Equality operator.
	 */
	[[nodiscard]] bool operator==(Frustum const&) const = default;

	/**
	 * @brief Returns the center of the frustum.
	 * @return The center.
	 */
	[[nodiscard]] constexpr Vec<Dim, T> center() const noexcept
	{
		// This is just an approximation, better would be to find the vertices
		Vec<Dim, T> c{};
		for (auto const& p : planes) {
			c += p.normal * p.distance;
		}
		return c / T(2 * Dim);
	}

	/**
	 * @brief Returns the AABB of the frustum.
	 * @return The AABB.
	 */
	[[nodiscard]] constexpr AABB<Dim, T> aabb() const noexcept
	{
		// TODO: Implement exactly by finding vertices.
		// For now, return infinite AABB as a safe placeholder if we don't have vertices.
		return AABB<Dim, T>(Vec<Dim, T>(-std::numeric_limits<T>::infinity()),
		                    Vec<Dim, T>(std::numeric_limits<T>::infinity()));
	}

	/**
	 * @brief Returns whether the frustum is degenerate.
	 * @return true if degenerate, false otherwise.
	 */
	[[nodiscard]] constexpr bool isDegenerate() const noexcept
	{
		for (auto const& p : planes) {
			if (p.isDegenerate()) {
				return true;
			}
		}
		return false;
	}
};

/**
 * @brief Output stream operator for Frustum.
 */
template <std::size_t Dim, std::floating_point T>
std::ostream& operator<<(std::ostream& out, Frustum<Dim, T> const& frustum)
{
	if constexpr (2 == Dim) {
		return out << "Left: [" << frustum[0] << "], Right: [" << frustum[1] << "], Bottom: ["
		           << frustum[2] << "], Top: [" << frustum[3] << "]";
	} else if constexpr (3 == Dim) {
		return out << "Left: [" << frustum[0] << "], Right: [" << frustum[1] << "], Bottom: ["
		           << frustum[2] << "], Top: [" << frustum[3] << "], Near: [" << frustum[4]
		           << "], Far: [" << frustum[5] << "]";
	} else {
		out << "Planes: ";
		for (std::size_t i = 0; i < 2 * Dim; ++i) {
			out << "[" << frustum[i] << "]" << (i == 2 * Dim - 1 ? "" : ", ");
		}
		return out;
	}
}

template <std::size_t Dim, std::floating_point T>
using FrustumX = Frustum<Dim, T>;

template <std::floating_point T>
using Frustum1 = Frustum<1, T>;
template <std::floating_point T>
using Frustum2 = Frustum<2, T>;
template <std::floating_point T>
using Frustum3 = Frustum<3, T>;
template <std::floating_point T>
using Frustum4 = Frustum<4, T>;

using Frustum1f = Frustum<1, float>;
using Frustum2f = Frustum<2, float>;
using Frustum3f = Frustum<3, float>;
using Frustum4f = Frustum<4, float>;

using Frustum1d = Frustum<1, double>;
using Frustum2d = Frustum<2, double>;
using Frustum3d = Frustum<3, double>;
using Frustum4d = Frustum<4, double>;

}  // namespace ufo

template <std::size_t Dim, std::floating_point T>
  requires std::formattable<T, char>
struct std::formatter<ufo::Frustum<Dim, T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::Frustum<Dim, T> const& frustum, std::format_context& ctx) const
	{
		if constexpr (2 == Dim) {
			return std::format_to(ctx.out(), "Left: [{}], Right: [{}], Bottom: [{}], Top: [{}]",
			                      frustum[0], frustum[1], frustum[2], frustum[3]);
		} else if constexpr (3 == Dim) {
			return std::format_to(
			    ctx.out(),
			    "Left: [{}], Right: [{}], Bottom: [{}], Top: [{}], Near: [{}], Far: [{}]",
			    frustum[0], frustum[1], frustum[2], frustum[3], frustum[4], frustum[5]);
		} else {
			std::format_to(ctx.out(), "Planes: ");
			for (std::size_t i = 0; i < 2 * Dim; ++i) {
				std::format_to(ctx.out(), "[{}]{}", frustum[i], (i == 2 * Dim - 1 ? "" : ", "));
			}
			return ctx.out();
		}
	}
};

#endif  // UFO_GEOMETRY_FRUSTUM_HPP