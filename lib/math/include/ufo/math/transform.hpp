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

#ifndef UFO_MATH_TRANSFORM_HPP
#define UFO_MATH_TRANSFORM_HPP

// UFO
#include <ufo/math/mat.hpp>
#include <ufo/math/quat.hpp>
#include <ufo/math/vec.hpp>

// STL
#include <cmath>
#include <concepts>
#include <cstddef>
#include <format>
#include <ostream>

/**
 * @ingroup math
 * @{
 */

namespace ufo
{

/**************************************************************************************
|                                                                                     |
|                                   Transform<Dim, T>                                 |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Rigid-body transform: a rotation matrix plus a translation vector.
 * @tparam Dim Spatial dimension; must be `2` or `3`.
 * @tparam T   Floating-point scalar type (default: `float`).
 *
 * @details Applying the transform to a vector `v` computes `R * v + t` where `R` is
 *          the rotation matrix and `t` is the translation.  Composition (`*`) follows
 *          the convention `(t1 * t2)(v) = t1(t2(v))`.
 */
template <std::size_t Dim, std::floating_point T = float>
  requires(Dim == 2 || Dim == 3)
struct Transform {
	using value_type = T;
	using size_type  = std::size_t;

	/**
	 * @brief Rotation component of the transform, represented as a `Dim × Dim` matrix.
	 */
	Mat<Dim, Dim, T> rotation = Mat<Dim, Dim, T>::identity();
	/**
	 * @brief Translation component of the transform.
	 */
	Vec<Dim, T> translation{};

	/**************************************************************************************
	|                                                                                     |
	|                                    Constructors                                     |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Default constructor; initializes to the identity transform (zero translation
	 * and identity rotation).
	 */
	constexpr Transform() noexcept = default;

	/**
	 * @brief Copy constructor.
	 */
	constexpr Transform(Transform const&) noexcept = default;

	/**
	 * @brief Constructs from a rotation matrix and a translation vector (same scalar type).
	 * @param [in] rotation    Rotation component.
	 * @param [in] translation Translation component.
	 */
	constexpr Transform(Mat<Dim, Dim, T> const& rotation, Vec<Dim, T> const& translation)
	    : rotation(rotation), translation(translation)
	{
	}

	/**
	 * @brief Constructs a pure-rotation transform (zero translation).
	 * @param [in] rotation Rotation matrix.
	 */
	constexpr explicit Transform(Mat<Dim, Dim, T> const& rotation) : rotation(rotation) {}

	/**
	 * @brief Constructs from rotation and translation with potentially different scalar
	 * types.
	 * @tparam T1 Scalar type of the rotation matrix.
	 * @tparam T2 Scalar type of the translation vector.
	 * @param [in] rotation    Rotation component (converted to `T`).
	 * @param [in] translation Translation component (converted to `T`).
	 */
	template <std::floating_point T1, std::floating_point T2>
	constexpr Transform(Mat<Dim, Dim, T1> const& rotation, Vec<Dim, T2> const& translation)
	    : rotation(rotation), translation(translation)
	{
	}

	/**
	 * @brief Constructs a pure-rotation transform from a rotation matrix of a different
	 * type.
	 * @tparam U Source scalar type.
	 * @param [in] rotation Rotation matrix (converted to `T`).
	 */
	template <std::floating_point U>
	constexpr explicit Transform(Mat<Dim, Dim, U> const& rotation) : rotation(rotation)
	{
	}

	/**
	 * @brief (2D only) Constructs from a rotation angle and a translation vector.
	 * @tparam T1 Scalar type of the angle.
	 * @tparam T2 Scalar type of the translation vector.
	 * @param [in] angle       Rotation angle in radians.
	 * @param [in] translation Translation vector.
	 */
	template <std::floating_point T1, std::floating_point T2>
	  requires(Dim == 2)
	constexpr Transform(T1 const& angle, Vec<2, T2> const& translation)
	    : translation(translation)
	{
		auto const s   = std::sin(angle);
		auto const c   = std::cos(angle);
		this->rotation = Mat<2, 2, T>(c, -s, s, c);
	}

	/**
	 * @brief (2D only) Constructs a pure-rotation transform from an angle.
	 * @tparam U Scalar type of the angle.
	 * @param [in] angle Rotation angle in radians.
	 */
	template <std::floating_point U>
	  requires(Dim == 2)
	constexpr explicit Transform(U const& angle)
	{
		auto const s   = std::sin(angle);
		auto const c   = std::cos(angle);
		this->rotation = Mat<2, 2, T>(c, -s, s, c);
	}

	/**
	 * @brief (3D only) Constructs from a quaternion and a translation vector.
	 * @tparam T1 Scalar type of the quaternion.
	 * @tparam T2 Scalar type of the translation vector.
	 * @param [in] rotation    Unit quaternion specifying the rotation.
	 * @param [in] translation Translation vector.
	 */
	template <std::floating_point T1, std::floating_point T2>
	  requires(Dim == 3)
	constexpr Transform(Quat<T1> const& rotation, Vec<3, T2> const& translation)
	    : Transform(Mat<3, 3, T1>(rotation), translation)
	{
	}

	/**
	 * @brief (3D only) Constructs a pure-rotation transform from a quaternion.
	 * @tparam U Scalar type of the quaternion.
	 * @param [in] rotation Unit quaternion specifying the rotation.
	 */
	template <std::floating_point U>
	  requires(Dim == 3)
	constexpr explicit Transform(Quat<U> const& rotation)
	    : Transform(Mat<3, 3, U>(rotation))
	{
	}

	/**
	 * @brief Constructs from a `(Dim+1) × (Dim+1)` homogeneous matrix.
	 * @details Extracts the upper-left `Dim × Dim` block as the rotation and column
	 *          `Dim` rows `0..Dim-1` as the translation.
	 * @tparam U Scalar type of the source matrix.
	 * @param [in] m Row-major homogeneous transformation matrix.
	 */
	template <std::floating_point U>
	constexpr explicit Transform(Mat<Dim + 1, Dim + 1, U> const& m)
	{
		rotation = Mat<Dim, Dim, U>(m);
		for (std::size_t i = 0; i < Dim; ++i) {
			translation[i] = m[i][Dim];
		}
	}

	/**
	 * @brief Converting constructor from a `Transform<Dim, U>` with a different scalar
	 * type.
	 * @tparam U Source scalar type.
	 * @param [in] other Source transform; components are converted to `T`.
	 */
	template <std::floating_point U>
	constexpr explicit Transform(Transform<Dim, U> const& other) noexcept
	    : rotation(other.rotation), translation(other.translation)
	{
	}

	/**************************************************************************************
	|                                                                                     |
	|                                 Assignment operator                                 |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Default copy assignment operator.
	 * @return Reference to `*this`.
	 */
	constexpr Transform& operator=(Transform const&) noexcept = default;

	/**
	 * @brief Converting assignment from a `Transform<Dim, U>` with a different scalar type.
	 * @tparam U Source scalar type.
	 * @param [in] rhs Source transform.
	 * @return Reference to `*this`.
	 */
	template <std::floating_point U>
	constexpr Transform& operator=(Transform<Dim, U> const& rhs) noexcept
	{
		rotation    = rhs.rotation;
		translation = rhs.translation;
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                                 Conversion operators                                |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Extracts the rotation component as a `Dim × Dim` matrix.
	 * @tparam U Target scalar type.
	 * @return A copy of the rotation matrix converted to `U`.
	 */
	template <std::floating_point U>
	constexpr explicit operator Mat<Dim, Dim, U>() const
	{
		return Mat<Dim, Dim, U>(rotation);
	}

	/**
	 * @brief Converts to a `(Dim+1) × (Dim+1)` row-major homogeneous matrix.
	 * @details The upper-left `Dim × Dim` block holds the rotation, column `Dim` rows
	 *          `0..Dim-1` hold the translation, and `m[Dim][Dim] = 1`.
	 * @tparam U Target scalar type.
	 * @return Homogeneous transformation matrix.
	 */
	template <std::floating_point U>
	constexpr explicit operator Mat<Dim + 1, Dim + 1, U>() const
	{
		Mat<Dim + 1, Dim + 1, U> m{Mat<Dim, Dim, U>(rotation)};
		for (std::size_t i = 0; i < Dim; ++i) {
			m[i][Dim] = static_cast<U>(translation[i]);
		}
		m[Dim][Dim] = U(1);
		return m;
	}

	/**
	 * @brief (3D only) Converts the rotation component to a quaternion.
	 * @tparam U Target scalar type.
	 * @return `Quat<U>` representing the rotation matrix.
	 */
	template <std::floating_point U>
	  requires(Dim == 3)
	constexpr explicit operator Quat<U>() const
	{
		return Quat<U>(rotation);
	}

	/**************************************************************************************
	|                                                                                     |
	|                                      Operations                                     |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Applies the transform to a vector: `result = R * v + t`.
	 * @tparam U Scalar type of the vector (may differ from `T`).
	 * @param [in] v Input vector.
	 * @return Transformed vector.
	 */
	template <std::floating_point U>
	[[nodiscard]] constexpr Vec<Dim, U> operator()(Vec<Dim, U> const& v) const
	{
		return Mat<Dim, Dim, U>(rotation) * v + Vec<Dim, U>(translation);
	}

	/**
	 * @brief (3D only) Applies the rotation component to a quaternion.
	 * @details Computes `Quat<U>(*this) * q`, i.e., pre-multiplies `q` by the rotation.
	 * @tparam U Scalar type of the quaternion.
	 * @param [in] q Input quaternion.
	 * @return Rotated quaternion.
	 */
	template <std::floating_point U>
	  requires(Dim == 3)
	[[nodiscard]] constexpr Quat<U> operator()(Quat<U> const& q) const
	{
		return Quat<U>(*this) * q;
	}

	/**
	 * @brief (2D only) Returns the rotation angle θ in `[-π, π]`.
	 * @return `atan2(rotation[0][1], rotation[0][0])`.
	 */
	[[nodiscard]] constexpr T theta() const
	  requires(Dim == 2)
	{
		return std::atan2(rotation[1][0], rotation[0][0]);
	}

	/**
	 * @brief Composes this transform with `t` in place: `*this = *this * t`.
	 * @param [in] t Right-hand transform to append.
	 * @return Reference to `*this`.
	 */
	constexpr Transform& operator*=(Transform const& t)
	{
		auto tmp = (*this)(t.translation);
		rotation *= t.rotation;
		translation = tmp;
		return *this;
	}

	/**
	 * @brief Equality comparison; two transforms are equal if their rotation and
	 * translation components are equal.
	 * @param [in] other Transform to compare with.
	 * @return `true` if the transforms are equal; `false` otherwise.
	 */
	[[nodiscard]] constexpr bool operator==(Transform const& other) const noexcept =
	    default;
};

/**************************************************************************************
|                                                                                     |
|                                     Aliases                                         |
|                                                                                     |
**************************************************************************************/

template <std::floating_point T = float>
using Transform2 = Transform<2, T>;
template <std::floating_point T = float>
using Transform3 = Transform<3, T>;

using Transform2f = Transform2<float>;
using Transform2d = Transform2<double>;
using Transform3f = Transform3<float>;
using Transform3d = Transform3<double>;

/**************************************************************************************
|                                                                                     |
|                                  Binary operators                                   |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Composes two transforms: `(t1 * t2)(v) = t1(t2(v))`.
 * @tparam Dim Spatial dimension (2 or 3).
 * @tparam T   Floating-point scalar type.
 * @param [in] t1 Outer transform.
 * @param [in] t2 Inner transform.
 * @return The composed transform.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr Transform<Dim, T> operator*(Transform<Dim, T>        t1,
                                                    Transform<Dim, T> const& t2)
{
	t1 *= t2;
	return t1;
}

/**
 * @brief Applies a transform to a vector: `result = R * v + t`.
 * @tparam Dim Spatial dimension.
 * @tparam T   Floating-point scalar type.
 * @param [in] t Transform.
 * @param [in] v Input vector.
 * @return Transformed vector.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr Vec<Dim, T> operator*(Transform<Dim, T> const& t,
                                              Vec<Dim, T> const&       v)
{
	return t(v);
}

/**
 * @brief (3D only) Applies the rotation component of a transform to a quaternion.
 * @tparam T Floating-point scalar type.
 * @param [in] t 3D transform.
 * @param [in] q Input quaternion.
 * @return Rotated quaternion.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Quat<T> operator*(Transform<3, T> const& t, Quat<T> const& q)
{
	return t(q);
}

/**
 * @brief Writes a human-readable representation of the transform to an output stream.
 * @details 2D: `"Translation: <t>, Theta: <θ>"`. 3D: `"Translation: <t>, Rotation: <q>"`.
 * @tparam Dim Spatial dimension.
 * @tparam T   Floating-point scalar type.
 * @param [in] out Output stream.
 * @param [in] t   Transform to print.
 * @return Reference to `out`.
 */
template <std::size_t Dim, std::floating_point T>
std::ostream& operator<<(std::ostream& out, Transform<Dim, T> const& t)
{
	out << "Translation: " << t.translation << " ";
	if constexpr (Dim == 2) {
		out << "Theta: " << t.theta();
	} else {
		out << "Rotation: " << Quat<T>(t);
	}
	return out;
}

/**************************************************************************************
|                                                                                     |
|                                    Free functions                                   |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Returns the inverse of a rigid-body transform.
 * @details For an orthogonal rotation matrix `R`, the inverse is `R^T`.
 *          The inverse translation is `-(R^T * t)`, so `inverse(t)(v) = R^T * (v - t)`.
 * @tparam Dim Spatial dimension.
 * @tparam T   Floating-point scalar type.
 * @param [in] t The transform to invert (rotation must be orthogonal).
 * @return The inverse transform such that `inverse(t) * t = identity`.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr Transform<Dim, T> inverse(Transform<Dim, T> const& t)
{
	Mat<Dim, Dim, T> const inv_r = transpose(t.rotation);
	return {inv_r, inv_r * -t.translation};
}
}  // namespace ufo

/**************************************************************************************
|                                                                                     |
|                               std::formatter support                                |
|                                                                                     |
**************************************************************************************/

template <std::size_t Dim, std::floating_point T>
  requires(std::formattable<T, char>)
struct std::formatter<ufo::Transform<Dim, T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::Transform<Dim, T> const& t, std::format_context& ctx) const
	{
		if constexpr (Dim == 2) {
			return std::format_to(ctx.out(), "translation: {} Theta: {}", t.translation,
			                      t.theta());
		} else {
			return std::format_to(ctx.out(), "translation: {} Rotation: {}", t.translation,
			                      ufo::Quat<T>(t));
		}
	}
};

/**
 * @}
 */

#endif  // UFO_MATH_TRANSFORM_HPP
