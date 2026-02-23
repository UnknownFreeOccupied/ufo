/**
 * @author Daniel Duberg (danielduberg@gmail.com)
 * @see https://github.com/UnknownFreeOccupied/ufo
 * @version 1.0
 * @date 2026-02-22
 *
 * @copyright Copyright (c) 2020-2026, Daniel Duberg
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2020-2026, Daniel Duberg
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
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
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_MATH_QUAT_HPP
#define UFO_MATH_QUAT_HPP

// UFO
#include <ufo/math/mat.hpp>
#include <ufo/math/vec.hpp>

// STL
#include <algorithm>
#include <cassert>
#include <cmath>
#include <concepts>
#include <format>
#include <limits>
#include <numbers>
#include <ostream>

namespace ufo
{

/**
 * @brief Unit quaternion representing an orientation or rotation in 3-D space.
 * @tparam T Floating-point scalar type (default: `float`).
 *
 * @details
 * Stored as `(w, x, y, z)` with the convention that the identity quaternion has `w = 1`
 * and `x = y = z = 0`.  All rotation operations assume the quaternion is unit-length; use
 * `normalize()` when in doubt.
 */
template <std::floating_point T = float>
struct Quat {
	using value_type = T;
	using size_type  = std::size_t;

	/**
	 * @brief Quaternion components.  Must satisfy `w^2 + x^2 + y^2 + z^2 == 1` for a valid
	 *        rotation.
	 */
	T w{1};  // Real (scalar) part.
	T x{};   // First imaginary component.
	T y{};   // Second imaginary component.
	T z{};   // Third imaginary component.

	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Default constructor; initializes to the identity rotation (1, 0, 0, 0).
	 */
	constexpr Quat() noexcept = default;

	/**
	 * @brief Copy constructor.
	 */
	constexpr Quat(Quat const&) noexcept = default;

	/**
	 * @brief Converting constructor from a quaternion with a different scalar type.
	 * @tparam U Source scalar type.
	 * @param [in] q Source quaternion; each component is `static_cast` to `T`.
	 * @details Implicit when `U == T`; `explicit` otherwise.
	 */
	template <std::floating_point U>
	constexpr explicit(!std::is_same_v<T, U>) Quat(Quat<U> const& q) noexcept
	    : w(static_cast<T>(q.w))
	    , x(static_cast<T>(q.x))
	    , y(static_cast<T>(q.y))
	    , z(static_cast<T>(q.z))
	{
	}

	/**
	 * @brief Constructs a quaternion from explicit `(w, x, y, z)` components.
	 * @param [in] w Real part.
	 * @param [in] x First imaginary component.
	 * @param [in] y Second imaginary component.
	 * @param [in] z Third imaginary component.
	 */
	constexpr Quat(T w, T x, T y, T z) noexcept : w(w), x(x), y(y), z(z) {}

	/**
	 * @brief Constructs the shortest-arc rotation quaternion from `u` to `v`.
	 * @param [in] u Source direction vector (need not be normalised).
	 * @param [in] v Target direction vector (need not be normalised).
	 * @details If `u` and `v` are antiparallel the rotation axis is chosen arbitrarily
	 *          (perpendicular to `u`) to produce a well-defined 180° rotation.
	 */
	Quat(Vec<3, T> const& u, Vec<3, T> const& v)
	{
		T         norm_u_norm_v = std::sqrt(dot(u, u) * dot(v, v));
		T         real_part     = norm_u_norm_v + dot(u, v);
		Vec<3, T> t;

		if (real_part < T(1.e-6) * norm_u_norm_v) {
			real_part = T(0);
			t         = std::abs(u[0]) > std::abs(u[2]) ? Vec<3, T>(-u[1], u[0], T(0))
			                                            : Vec<3, T>(T(0), -u[2], u[1]);
		} else {
			t = cross(u, v);
		}

		*this = normalize(Quat<T>(real_part, t[0], t[1], t[2]));
	}

	/**
	 * @brief Constructs a quaternion from Euler angles `(pitch, yaw, roll)` in radians.
	 * @param [in] euler_angles `Vec<3, T>` with components `(pitch, yaw, roll)` — matches
	 *             the ROS 2 `setRPY` convention.
	 */
	explicit Quat(Vec<3, T> const& euler_angles)
	{
		Vec<3, T> const c{std::cos(euler_angles[0] * T(0.5)),
		                  std::cos(euler_angles[1] * T(0.5)),
		                  std::cos(euler_angles[2] * T(0.5))};
		Vec<3, T> const s{std::sin(euler_angles[0] * T(0.5)),
		                  std::sin(euler_angles[1] * T(0.5)),
		                  std::sin(euler_angles[2] * T(0.5))};

		w = c[0] * c[1] * c[2] + s[0] * s[1] * s[2];
		x = s[0] * c[1] * c[2] - c[0] * s[1] * s[2];
		y = c[0] * s[1] * c[2] + s[0] * c[1] * s[2];
		z = c[0] * c[1] * s[2] - s[0] * s[1] * c[2];
	}

	/**
	 * @brief Constructs a quaternion from a row-major 3×3 rotation matrix.
	 * @param [in] m Row-major rotation matrix where `m[row][col] = R_{row,col}`.
	 * @details Uses Shepperd's method to select the numerically best branch.
	 */
	constexpr Quat(Mat<3, 3, T> const& m) noexcept
	{
		T const trace = m[0][0] + m[1][1] + m[2][2];

		if (trace > T(0)) {
			T s = std::sqrt(trace + T(1));
			w   = s * T(0.5);
			s   = T(0.5) / s;
			// Row-major: m[row][col] — x = (R_{2,1} - R_{1,2}) * s
			x = (m[2][1] - m[1][2]) * s;
			y = (m[0][2] - m[2][0]) * s;
			z = (m[1][0] - m[0][1]) * s;
		} else {
			unsigned const i = m[0][0] < m[1][1] ? (m[1][1] < m[2][2] ? 2u : 1u)
			                                     : (m[0][0] < m[2][2] ? 2u : 0u);
			unsigned const j = (i + 1u) % 3u;
			unsigned const k = (i + 2u) % 3u;

			T q[4]{};
			T s      = std::sqrt(m[i][i] - m[j][j] - m[k][k] + T(1));
			q[i + 1] = s * T(0.5);
			s        = T(0.5) / s;
			q[0]     = (m[k][j] - m[j][k]) * s;  // row-major: swap j,k vs column-major
			q[j + 1] = (m[j][i] + m[i][j]) * s;  // symmetric — same as column-major
			q[k + 1] = (m[k][i] + m[i][k]) * s;
			w        = q[0];
			x        = q[1];
			y        = q[2];
			z        = q[3];
		}
	}

	/**
	 * @brief Constructs a quaternion from the upper-left 3×3 block of a 4×4 matrix.
	 * @param [in] m Row-major 4×4 homogeneous matrix; only the rotation submatrix is used.
	 */
	constexpr Quat(Mat<4, 4, T> const& m) noexcept : Quat(Mat<3, 3, T>(m)) {}

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Copy assignment operator.
	 * @return Reference to `*this`.
	 */
	constexpr Quat& operator=(Quat const&) noexcept = default;

	/**
	 * @brief Converting assignment from a quaternion with a different scalar type.
	 * @tparam U Source scalar type.
	 * @param [in] rhs Source quaternion; each component is `static_cast` to `T`.
	 * @return Reference to `*this`.
	 */
	template <std::floating_point U>
	constexpr Quat& operator=(Quat<U> const& rhs) noexcept
	{
		w = static_cast<T>(rhs.w);
		x = static_cast<T>(rhs.x);
		y = static_cast<T>(rhs.y);
		z = static_cast<T>(rhs.z);
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                 Conversion operator                                 |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Converts to a row-major 3×3 rotation matrix.
	 * @return `Mat<3, 3, T>` where `m[row][col] = R_{row,col}`.
	 * @details Assumes the quaternion is unit-length.
	 */
	[[nodiscard]] constexpr explicit operator Mat<3, 3, T>() const noexcept
	{
		T const xx(x * x), yy(y * y), zz(z * z);
		T const xz(x * z), xy(x * y), yz(y * z);
		T const wx(w * x), wy(w * y), wz(w * z);

		return Mat<3, 3, T>(
		    Vec<3, T>(T(1) - T(2) * (yy + zz), T(2) * (xy - wz), T(2) * (xz + wy)),
		    Vec<3, T>(T(2) * (xy + wz), T(1) - T(2) * (xx + zz), T(2) * (yz - wx)),
		    Vec<3, T>(T(2) * (xz - wy), T(2) * (yz + wx), T(1) - T(2) * (xx + yy)));
	}

	/**
	 * @brief Converts to a row-major 4×4 homogeneous rotation matrix.
	 * @return `Mat<4, 4, T>` with the 3×3 rotation block in the upper-left corner,
	 *         identity in the last row and column, and 0 in the last column rows 0–2.
	 */
	[[nodiscard]] constexpr explicit operator Mat<4, 4, T>() const noexcept
	{
		// TODO: Should we add a constructor for this?
		Mat<4, 4, T> m(Mat<3, 3, T>(*this));
		m[3][3] = T(1);
		return m;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                   Element access                                    |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Component access by index: 0->w, 1->x, 2->y, 3->z.
	 * @param [in] pos Component index (must be `< 4`; asserted in debug builds).
	 * @return Reference to the selected component (const or non-const, deduced from
	 * `this`).
	 */
	[[nodiscard]] constexpr auto& operator[](this auto& self, size_type pos) noexcept
	{
		assert(4 > pos);
		switch (pos) {
			case 0: return self.w;
			case 1: return self.x;
			case 2: return self.y;
			default: return self.z;
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                            Compound assignment operator                             |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Component-wise addition.
	 * @param [in] q Quaternion to add.
	 * @return Reference to `*this`.
	 */
	constexpr Quat& operator+=(Quat const& q) noexcept
	{
		w += q.w;
		x += q.x;
		y += q.y;
		z += q.z;
		return *this;
	}

	/**
	 * @brief Component-wise subtraction.
	 * @param [in] q Quaternion to subtract.
	 * @return Reference to `*this`.
	 */
	constexpr Quat& operator-=(Quat const& q) noexcept
	{
		w -= q.w;
		x -= q.x;
		y -= q.y;
		z -= q.z;
		return *this;
	}

	/**
	 * @brief Hamilton (quaternion) product in place.
	 * @param [in] q Right-hand side quaternion.
	 * @return Reference to `*this`.
	 */
	constexpr Quat& operator*=(Quat const& q) noexcept
	{
		Quat const p(*this);
		w = p.w * q.w - p.x * q.x - p.y * q.y - p.z * q.z;
		x = p.w * q.x + p.x * q.w + p.y * q.z - p.z * q.y;
		y = p.w * q.y + p.y * q.w + p.z * q.x - p.x * q.z;
		z = p.w * q.z + p.z * q.w + p.x * q.y - p.y * q.x;
		return *this;
	}

	/**
	 * @brief Scales all components by scalar `s`.
	 * @param [in] s Scalar multiplier.
	 * @return Reference to `*this`.
	 */
	constexpr Quat& operator*=(T s) noexcept
	{
		w *= s;
		x *= s;
		y *= s;
		z *= s;
		return *this;
	}

	/**
	 * @brief Divides all components by scalar `s`.
	 * @param [in] s Scalar divisor.
	 * @return Reference to `*this`.
	 */
	constexpr Quat& operator/=(T s) noexcept
	{
		w /= s;
		x /= s;
		y /= s;
		z /= s;
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Capacity                                       |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Returns the number of components (always 4: w, x, y, z).
	 * @return `4`.
	 */
	[[nodiscard]] static constexpr size_type size() noexcept { return 4; }

	/**************************************************************************************
	|                                                                                     |
	|                                     Operations                                      |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Swaps all components with `other`.
	 * @param [in] other Quaternion to swap with.
	 */
	void swap(Quat& other) noexcept
	{
		using std::swap;
		swap(w, other.w);
		swap(x, other.x);
		swap(y, other.y);
		swap(z, other.z);
	}

	/**
	 * @brief Component-wise equality comparison.
	 * @param [in] other Quaternion to compare with.
	 * @return `true` if all components are equal, `false` otherwise.
	 * @details Does not consider two quaternions representing the same rotation (e.g.
	 * `q` and `-q`) as equal; use `dot(q1, q2)` to check for that instead.
	 */
	constexpr bool operator==(Quat const&) const noexcept = default;
};

/**************************************************************************************
|                                                                                     |
|                                     Aliases                                         |
|                                                                                     |
**************************************************************************************/

using Quatf = Quat<float>;
using Quatd = Quat<double>;

/**************************************************************************************
|                                                                                     |
|                                       Concept                                       |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Concept satisfied by any specialization of `Quat<U>`.
 * @tparam T Type to check.
 */
template <class T>
concept QuatType = requires(T const& t) { []<class U>(Quat<U> const&) {}(t); };

/**************************************************************************************
|                                                                                     |
|                                   Unary operators                                   |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Unary plus — returns a copy of `q` unchanged.
 * @tparam T Floating-point scalar type.
 * @param [in] q Input quaternion.
 * @return `q`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Quat<T> operator+(Quat<T> const& q) noexcept
{
	return q;
}

/**
 * @brief Unary negation — negates all four components.
 * @tparam T Floating-point scalar type.
 * @param [in] q Input quaternion.
 * @return `(-q.w, -q.x, -q.y, -q.z)`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Quat<T> operator-(Quat<T> const& q) noexcept
{
	return {-q.w, -q.x, -q.y, -q.z};
}

/**************************************************************************************
|                                                                                     |
|                                  Binary operators                                   |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Component-wise addition of two quaternions.
 * @tparam T Floating-point scalar type.
 * @param [in] lhs Left-hand side.
 * @param [in] rhs Right-hand side.
 * @return Component-wise sum.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Quat<T> operator+(Quat<T> lhs, Quat<T> const& rhs) noexcept
{
	lhs += rhs;
	return lhs;
}

/**
 * @brief Component-wise subtraction of two quaternions.
 * @tparam T Floating-point scalar type.
 * @param [in] lhs Left-hand side.
 * @param [in] rhs Right-hand side.
 * @return Component-wise difference.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Quat<T> operator-(Quat<T> lhs, Quat<T> const& rhs) noexcept
{
	lhs -= rhs;
	return lhs;
}

/**
 * @brief Hamilton (quaternion) product.
 * @tparam T Floating-point scalar type.
 * @param [in] lhs Left-hand side.
 * @param [in] rhs Right-hand side.
 * @return The Hamilton product `lhs * rhs`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Quat<T> operator*(Quat<T> lhs, Quat<T> const& rhs) noexcept
{
	lhs *= rhs;
	return lhs;
}

/**
 * @brief Rotates a 3-D column vector by the quaternion (`q * v`).
 * @details Uses the optimised sandwich formula `v' = v + 2w(qvec × v) + 2(qvec × (qvec ×
 * v))`. Assumes `q` is a unit quaternion.
 * @tparam T Floating-point scalar type.
 * @param [in] q Unit rotation quaternion.
 * @param [in] v Input vector.
 * @return Rotated vector.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Vec<3, T> operator*(Quat<T> const& q, Vec<3, T> const& v) noexcept
{
	Vec<3, T> const qvec(q.x, q.y, q.z);
	Vec<3, T> const uv(cross(qvec, v));
	Vec<3, T> const uuv(cross(qvec, uv));
	return v + ((uv * q.w) + uuv) * T(2);
}

/**
 * @brief Rotates a 3-D column vector by the inverse quaternion (`v * q`).
 * @details Equivalent to `inverse(q) * v`.  Assumes `q` is a unit quaternion.
 * @tparam T Floating-point scalar type.
 * @param [in] v Input vector.
 * @param [in] q Unit rotation quaternion.
 * @return Vector rotated by the inverse rotation.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Vec<3, T> operator*(Vec<3, T> const& v, Quat<T> const& q) noexcept
{
	return inverse(q) * v;
}

/**
 * @brief Rotates the XYZ part of a homogeneous `Vec<4>` by `q`, preserving the W
 * component.
 * @tparam T Floating-point scalar type.
 * @param [in] q Unit rotation quaternion.
 * @param [in] v Homogeneous 4-vector; `v[3]` is passed through unchanged.
 * @return Rotated 4-vector with the same `v[3]`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Vec<4, T> operator*(Quat<T> const& q, Vec<4, T> const& v) noexcept
{
	Vec<3, T> const r = q * Vec<3, T>(v[0], v[1], v[2]);
	return Vec<4, T>(r[0], r[1], r[2], v[3]);
}

/**
 * @brief Rotates the XYZ part of `v` by the inverse of `q`, preserving the W component.
 * @tparam T Floating-point scalar type.
 * @param [in] v Homogeneous 4-vector; `v[3]` is passed through unchanged.
 * @param [in] q Unit rotation quaternion.
 * @return Inversely-rotated 4-vector with the same `v[3]`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Vec<4, T> operator*(Vec<4, T> const& v, Quat<T> const& q) noexcept
{
	Vec<3, T> const r = inverse(q) * Vec<3, T>(v[0], v[1], v[2]);
	return Vec<4, T>(r[0], r[1], r[2], v[3]);
}

/**
 * @brief Scales all components of `q` by scalar `s`.
 * @tparam T Floating-point scalar type.
 * @param [in] lhs Quaternion.
 * @param [in] s   Scalar multiplier.
 * @return `q * s` component-wise.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Quat<T> operator*(Quat<T> lhs, T s) noexcept
{
	lhs *= s;
	return lhs;
}

/**
 * @brief Scales all components of `q` by scalar `s`.
 * @tparam T Floating-point scalar type.
 * @param [in] s   Scalar multiplier.
 * @param [in] rhs Quaternion.
 * @return `s * q` component-wise.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Quat<T> operator*(T s, Quat<T> rhs) noexcept
{
	rhs *= s;
	return rhs;
}

/**
 * @brief Divides all components of `q` by scalar `s`.
 * @tparam T Floating-point scalar type.
 * @param [in] lhs Quaternion.
 * @param [in] s   Scalar divisor.
 * @return `q / s` component-wise.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Quat<T> operator/(Quat<T> lhs, T s) noexcept
{
	lhs /= s;
	return lhs;
}

/**************************************************************************************
|                                                                                     |
|                                     Operations                                      |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Non-member swap — exchanges all components of `lhs` and `rhs`.
 * @tparam T Floating-point scalar type.
 * @param [in] lhs First quaternion.
 * @param [in] rhs Second quaternion.
 */
template <std::floating_point T>
void swap(Quat<T>& lhs, Quat<T>& rhs) noexcept
{
	lhs.swap(rhs);
}

/**************************************************************************************
|                                                                                     |
|                                      Functions                                      |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Computes the four-component dot product `a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z`.
 * @tparam T Floating-point scalar type.
 * @param [in] a First quaternion.
 * @param [in] b Second quaternion.
 * @return Scalar dot product.
 */
template <std::floating_point T>
[[nodiscard]] constexpr T dot(Quat<T> const& a, Quat<T> const& b) noexcept
{
	return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

/**
 * @brief Returns the squared norm (dot product with itself).
 * @tparam T Floating-point scalar type.
 * @param [in] q Input quaternion.
 * @return `dot(q, q)`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr T normSquared(Quat<T> const& q) noexcept
{
	return dot(q, q);
}

/**
 * @brief Returns the Euclidean norm `sqrt(w² + x² + y² + z²)`.
 * @tparam T Floating-point scalar type.
 * @param [in] q Input quaternion.
 * @return `sqrt(normSquared(q))`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr T norm(Quat<T> const& q) noexcept
{
	return std::sqrt(normSquared(q));
}

/**
 * @brief Returns a unit quaternion in the same direction as `q`.
 * @tparam T Floating-point scalar type.
 * @param [in] q Input quaternion.
 * @return `q / norm(q)`, or the identity quaternion `(1, 0, 0, 0)` if `norm(q) ≤ 0`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Quat<T> normalize(Quat<T> const& q) noexcept
{
	T const len = norm(q);
	if (len <= T(0)) {
		return {T(1), T(0), T(0), T(0)};
	}
	return q / len;
}

/**
 * @brief Returns the quaternion conjugate `(w, -x, -y, -z)`.
 * @tparam T Floating-point scalar type.
 * @param [in] q Input quaternion.
 * @return `{q.w, -q.x, -q.y, -q.z}`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Quat<T> conjugate(Quat<T> const& q) noexcept
{
	return {q.w, -q.x, -q.y, -q.z};
}

/**
 * @brief Returns the multiplicative inverse `conjugate(q) / normSquared(q)`.
 * @tparam T Floating-point scalar type.
 * @param [in] q Input quaternion (must be non-zero).
 * @return The inverse quaternion such that `q * inverse(q) = identity`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Quat<T> inverse(Quat<T> const& q) noexcept
{
	return conjugate(q) / normSquared(q);
}

/**
 * @brief Computes the Hamilton cross product of two quaternions.
 * @details Identical to the quaternion Hamilton product `q1 * q2`.
 * @tparam T Floating-point scalar type.
 * @param [in] q1 First quaternion.
 * @param [in] q2 Second quaternion.
 * @return Hamilton product `q1 ⊗ q2`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Quat<T> cross(Quat<T> const& q1, Quat<T> const& q2) noexcept
{
	return {q1.w * q2.w - q1.x * q2.x - q1.y * q2.y - q1.z * q2.z,
	        q1.w * q2.x + q1.x * q2.w + q1.y * q2.z - q1.z * q2.y,
	        q1.w * q2.y + q1.y * q2.w + q1.z * q2.x - q1.x * q2.z,
	        q1.w * q2.z + q1.z * q2.w + q1.x * q2.y - q1.y * q2.x};
}

/**************************************************************************************
|                                                                                     |
|                                    Interpolation                                    |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Spherical or linear interpolation between two quaternions.
 * @details Falls back to `lerp` when the quaternions are nearly parallel
 *          (cos θ > 1 − ε) to avoid division by near-zero sine.
 * @tparam T Floating-point scalar type.
 * @param [in] x Start quaternion.
 * @param [in] y End quaternion.
 * @param [in] a Interpolation factor in `[0, 1]`.
 * @return Interpolated quaternion (not necessarily unit-length if inputs are not).
 */
template <std::floating_point T>
[[nodiscard]] Quat<T> mix(Quat<T> const& x, Quat<T> const& y, T a)
{
	T const cos_theta = dot(x, y);

	if (cos_theta > T(1) - std::numeric_limits<T>::epsilon()) {
		return {std::lerp(x.w, y.w, a), std::lerp(x.x, y.x, a), std::lerp(x.y, y.y, a),
		        std::lerp(x.z, y.z, a)};
	}

	T const angle = std::acos(cos_theta);
	return (std::sin((T(1) - a) * angle) * x + std::sin(a * angle) * y) / std::sin(angle);
}

/**
 * @brief Normalised linear interpolation (NLERP) between two quaternions.
 * @details Cheaper than `slerp` but does not maintain constant angular velocity.
 *          `a` must be in `[0, 1]` (asserted in debug builds).
 * @tparam T Floating-point scalar type.
 * @param [in] x Start quaternion.
 * @param [in] y End quaternion.
 * @param [in] a Interpolation factor in `[0, 1]`.
 * @return `x * (1 - a) + y * a` (component-wise, not normalised).
 */
template <std::floating_point T>
[[nodiscard]] constexpr Quat<T> lerp(Quat<T> const& x, Quat<T> const& y, T a) noexcept
{
	assert(a >= T(0) && a <= T(1));
	return x * (T(1) - a) + y * a;
}

/**
 * @brief Spherical linear interpolation (SLERP) between two unit quaternions.
 * @details Automatically flips the sign of `y` when the dot product is negative
 *          to ensure the shorter arc is taken.  Falls back to lerp near-parallel inputs.
 * @tparam T Floating-point scalar type.
 * @param [in] x Start unit quaternion.
 * @param [in] y End unit quaternion.
 * @param [in] a Interpolation factor in `[0, 1]`.
 * @return Unit quaternion interpolated along the great-circle arc from `x` to `y`.
 */
template <std::floating_point T>
[[nodiscard]] Quat<T> slerp(Quat<T> const& x, Quat<T> const& y, T a)
{
	Quat<T> z         = y;
	T       cos_theta = dot(x, y);

	if (cos_theta < T(0)) {
		z         = -y;
		cos_theta = -cos_theta;
	}

	if (cos_theta > T(1) - std::numeric_limits<T>::epsilon()) {
		return {std::lerp(x.w, z.w, a), std::lerp(x.x, z.x, a), std::lerp(x.y, z.y, a),
		        std::lerp(x.z, z.z, a)};
	}

	T const angle = std::acos(cos_theta);
	return (std::sin((T(1) - a) * angle) * x + std::sin(a * angle) * z) / std::sin(angle);
}

/**
 * @brief SLERP with `k` extra full spins (Graphics Gems III, page 96).
 * @details Useful for animation retargeting where extra rotations are desired.
 * @tparam T Floating-point scalar type.
 * @tparam S Type of the spin count `k` (converted to `T` internally).
 * @param [in] x Start unit quaternion.
 * @param [in] y End unit quaternion.
 * @param [in] a Interpolation factor in `[0, 1]`.
 * @param [in] k Number of extra full spins to add.
 * @return Unit quaternion interpolated along the arc with `k` additional loops.
 */
// k-th extra spin slerp (Graphics Gems III, page 96)
template <std::floating_point T, std::floating_point S>
[[nodiscard]] Quat<T> slerp(Quat<T> const& x, Quat<T> const& y, T a, S k)
{
	Quat<T> z         = y;
	T       cos_theta = dot(x, y);

	if (cos_theta < T(0)) {
		z         = -y;
		cos_theta = -cos_theta;
	}

	if (cos_theta > T(1) - std::numeric_limits<T>::epsilon()) {
		return {std::lerp(x.w, z.w, a), std::lerp(x.x, z.x, a), std::lerp(x.y, z.y, a),
		        std::lerp(x.z, z.z, a)};
	}

	T const angle = std::acos(cos_theta);
	T const phi   = angle + T(k) * std::numbers::pi_v<T>;
	return (std::sin(angle - a * phi) * x + std::sin(a * phi) * z) / std::sin(angle);
}

/**************************************************************************************
|                                                                                     |
|                                    Trigonometric                                    |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Extracts the rotation angle (in radians) from a unit quaternion.
 * @tparam T Floating-point scalar type.
 * @param [in] q Unit quaternion.
 * @return Rotation angle in `[0, 2π)`.
 */
template <std::floating_point T>
[[nodiscard]] T angle(Quat<T> const& q) noexcept
{
	constexpr T cos_one_over_two = T(0.877582561890372716130286068203503191);
	if (std::abs(q.w) > cos_one_over_two) {
		T const a = std::asin(std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z)) * T(2);
		return q.w < T(0) ? std::numbers::pi_v<T> * T(2) - a : a;
	}
	return std::acos(q.w) * T(2);
}

/**
 * @brief Extracts the unit rotation axis from a unit quaternion.
 * @tparam T Floating-point scalar type.
 * @param [in] q Unit quaternion.
 * @return Normalised rotation axis; returns `(0, 0, 1)` for the identity quaternion.
 */
template <std::floating_point T>
[[nodiscard]] Vec<3, T> axis(Quat<T> const& q) noexcept
{
	T const tmp = T(1) - q.w * q.w;
	if (tmp <= T(0)) {
		return Vec<3, T>(T(0), T(0), T(1));
	}
	return Vec<3, T>(q.x, q.y, q.z) / std::sqrt(tmp);
}

/**
 * @brief Constructs a unit quaternion from an angle-axis representation.
 * @tparam T Floating-point scalar type.
 * @param [in] angle Rotation angle in radians.
 * @param [in] a     Unit rotation axis (must already be normalised).
 * @return `Quat<T>(cos(angle/2), a * sin(angle/2))`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Quat<T> angleAxis(T angle, Vec<3, T> const& a) noexcept
{
	T const s = std::sin(angle * T(0.5));
	return Quat<T>(std::cos(angle * T(0.5)), a[0] * s, a[1] * s, a[2] * s);
}

/**************************************************************************************
|                                                                                     |
|                                    Euler angles                                     |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Extracts the roll angle (rotation about the X-axis) in radians.
 * @tparam T Floating-point scalar type.
 * @param [in] q Unit quaternion.
 * @return Roll angle in `[-π, π]`.
 */
template <std::floating_point T>
[[nodiscard]] T roll(Quat<T> const& q) noexcept
{
	return T(std::atan2(T(2) * (q.x * q.y + q.w * q.z),
	                    q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z));
}

/**
 * @brief Extracts the pitch angle (rotation about the Y-axis) in radians.
 * @tparam T Floating-point scalar type.
 * @param [in] q Unit quaternion.
 * @return Pitch angle in `[-π, π]`.
 */
template <std::floating_point T>
[[nodiscard]] T pitch(Quat<T> const& q) noexcept
{
	return T(std::atan2(T(2) * (q.y * q.z + q.w * q.x),
	                    q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z));
}

/**
 * @brief Extracts the yaw angle (rotation about the Z-axis) in radians.
 * @tparam T Floating-point scalar type.
 * @param [in] q Unit quaternion.
 * @return Yaw angle clamped to `[-π/2, π/2]`.
 */
template <std::floating_point T>
[[nodiscard]] T yaw(Quat<T> const& q) noexcept
{
	return std::asin(std::clamp(T(-2) * (q.x * q.z - q.w * q.y), T(-1), T(1)));
}

/**
 * @brief Returns the Euler angles `(pitch, yaw, roll)` in radians.
 * @tparam T Floating-point scalar type.
 * @param [in] q Unit quaternion.
 * @return `Vec<3, T>{pitch(q), yaw(q), roll(q)}`.
 */
template <std::floating_point T>
[[nodiscard]] Vec<3, T> eulerAngles(Quat<T> const& q) noexcept
{
	return Vec<3, T>{pitch(q), yaw(q), roll(q)};
}

/**************************************************************************************
|                                                                                     |
|                                     Transform                                       |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Appends an angle-axis rotation to a quaternion.
 * @details Computes `q * angleAxis(angle, axis)`, normalising `axis` only if it
 *          deviates from unit length by more than 0.001.
 * @tparam T Floating-point scalar type.
 * @param [in] q     Base unit quaternion.
 * @param [in] angle Additional rotation angle in radians.
 * @param [in] axis  Rotation axis (normalised internally if needed).
 * @return Resulting unit quaternion.
 */
template <std::floating_point T>
[[nodiscard]] Quat<T> rotate(Quat<T> const& q, T angle, Vec<3, T> axis)
{
	T const len = length(axis);
	if (std::abs(len - T(1)) > T(0.001)) {
		axis /= len;
	}
	T const s = std::sin(angle * T(0.5));
	return q * Quat<T>(std::cos(angle * T(0.5)), axis[0] * s, axis[1] * s, axis[2] * s);
}

/**
 * @brief Builds a quaternion that orients an object to look in `direction`.
 * @tparam T           Floating-point scalar type.
 * @param [in] direction View direction (need not be normalised).
 * @param [in] up        World-up reference vector.
 * @return Unit quaternion representing the look-at orientation.
 */
template <std::floating_point T>
[[nodiscard]] Quat<T> quatLookAt(Vec<3, T> const& direction, Vec<3, T> const& up)
{
	Vec<3, T> const fwd   = -direction;
	Vec<3, T>       right = cross(up, fwd);
	right /= std::max(std::sqrt(T(0.00001)), norm(right));
	Vec<3, T> const up_c = cross(fwd, right);

	// Build rotation matrix in row-major where columns are right, up_c, fwd
	Mat<3, 3, T> const m(Vec<3, T>(right[0], up_c[0], fwd[0]),
	                     Vec<3, T>(right[1], up_c[1], fwd[1]),
	                     Vec<3, T>(right[2], up_c[2], fwd[2]));
	return Quat<T>(m);
}

/**************************************************************************************
|                                                                                     |
|                                    isnan / isinf                                    |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Returns a bool vector indicating which components are NaN.
 * @tparam T Floating-point scalar type.
 * @param [in] q Input quaternion.
 * @return `Vec<4, bool>` where element `i` is `true` iff component `i` is NaN.
 */
template <std::floating_point T>
[[nodiscard]] Vec<4, bool> isnan(Quat<T> const& q) noexcept
{
	return Vec<4, bool>(std::isnan(q.w), std::isnan(q.x), std::isnan(q.y), std::isnan(q.z));
}

/**
 * @brief Returns a bool vector indicating which components are infinite.
 * @tparam T Floating-point scalar type.
 * @param [in] q Input quaternion.
 * @return `Vec<4, bool>` where element `i` is `true` iff component `i` is ±∞.
 */
template <std::floating_point T>
[[nodiscard]] Vec<4, bool> isinf(Quat<T> const& q) noexcept
{
	return Vec<4, bool>(std::isinf(q.w), std::isinf(q.x), std::isinf(q.y), std::isinf(q.z));
}

/**************************************************************************************
|                                                                                     |
|                                       Print                                         |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Writes the quaternion to an output stream as `"qw: W qx: X qy: Y qz: Z"`.
 * @tparam T Floating-point scalar type.
 * @param [in] out Output stream.
 * @param [in] q   Quaternion to print.
 * @return Reference to `out`.
 */
template <std::floating_point T>
std::ostream& operator<<(std::ostream& out, Quat<T> const& q)
{
	return out << "qw: " << q.w << " qx: " << q.x << " qy: " << q.y << " qz: " << q.z;
}

}  // namespace ufo

template <std::floating_point T>
  requires std::formattable<T, char>
struct std::formatter<ufo::Quat<T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::Quat<T> const& q, std::format_context& ctx) const
	{
		return std::format_to(ctx.out(), "qw: {} qx: {} qy: {} qz: {}", q.w, q.x, q.y, q.z);
	}
};

#endif  // UFO_MATH_QUAT_HPP
