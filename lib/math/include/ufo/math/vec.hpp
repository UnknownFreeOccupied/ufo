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

#ifndef UFO_MATH_VEC_HPP
#define UFO_MATH_VEC_HPP

// UFO
#include <ufo/math/math.hpp>

// STL
#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <format>
#include <functional>
#include <limits>
#include <ostream>
#include <ranges>
#include <type_traits>
#include <utility>

namespace ufo
{

/**
 * @brief A fixed-size arithmetic vector of up to 4 dimensions.
 * @tparam Dim Number of dimensions (1–4).
 * @tparam T Arithmetic element type (e.g., `int`, `float`, `double`).
 */
template <std::size_t Dim, class T>
  requires(Dim <= 4 && std::is_arithmetic_v<T>)
struct Vec {
	using value_type = T;
	using size_type  = std::size_t;

	std::array<T, Dim> fields{};

	/**
	 * @brief Default constructor; initializes all elements to zero.
	 */
	constexpr Vec() noexcept = default;

	/**
	 * @brief Constructs a vector from up to `Dim` individual element values.
	 * @tparam Args Argument types, each convertible to `T`.
	 * @param [in] args Element values. Unspecified trailing elements are zero-initialized.
	 *
	 * @details
	 * The constructor is `explicit` unless exactly one argument is provided,
	 * in which case implicit conversion (broadcast) is allowed.
	 */
	template <std::convertible_to<T>... Args>
	  requires(sizeof...(Args) <= Dim)
	explicit(sizeof...(Args) == 1) constexpr Vec(Args... args) noexcept
	    : fields{static_cast<T>(args)...}
	{
	}

	/**
	 * @brief Converting constructor from a vector with a different element type.
	 * @tparam U Source element type.
	 * @param [in] other The source vector; each element is `static_cast` to `T`.
	 *
	 * @details Implicit when `U` is the same as `T`; `explicit` otherwise.
	 */
	template <class U>
	constexpr explicit(!std::is_same_v<T, U>) Vec(Vec<Dim, U> const& other) noexcept
	{
		std::ranges::transform(other, begin(), [](U const& v) { return static_cast<T>(v); });
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Accessors                                      |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Accesses the element at index `i`.
	 * @param [in] i Zero-based element index.
	 * @return Reference to the element (const or non-const, deduced from `this`).
	 */
	constexpr auto& operator[](this auto& self, size_type i) noexcept
	{
		return self.fields[i];
	}

	/**
	 * @brief Accesses the first component (x).
	 * @return Reference to the first element (const or non-const, deduced from `this`).
	 */
	[[nodiscard]] constexpr auto& x(this auto& self) noexcept
	  requires(Dim >= 1)
	{
		return self[0];
	}

	/**
	 * @brief Accesses the second component (y).
	 * @return Reference to the second element (const or non-const, deduced from `this`).
	 */
	[[nodiscard]] constexpr auto& y(this auto& self) noexcept
	  requires(Dim >= 2)
	{
		return self[1];
	}

	/**
	 * @brief Accesses the third component (z).
	 * @return Reference to the third element (const or non-const, deduced from `this`).
	 */
	[[nodiscard]] constexpr auto& z(this auto& self) noexcept
	  requires(Dim >= 3)
	{
		return self[2];
	}

	/**
	 * @brief Accesses the fourth component (w).
	 * @return Reference to the fourth element (const or non-const, deduced from `this`).
	 */
	[[nodiscard]] constexpr auto& w(this auto& self) noexcept
	  requires(Dim >= 4)
	{
		return self[3];
	}

	/**
	 * @brief Returns an iterator to the first element.
	 * @return Iterator (const or non-const, deduced from `this`).
	 */
	[[nodiscard]] constexpr auto begin(this auto& self) noexcept
	{
		return self.fields.begin();
	}

	/**
	 * @brief Returns a past-the-end iterator.
	 * @return Iterator (const or non-const, deduced from `this`).
	 */
	[[nodiscard]] constexpr auto end(this auto& self) noexcept { return self.fields.end(); }

	/**
	 * @brief Returns a pointer to the underlying element array.
	 * @return Pointer to the first element (const or non-const, deduced from `this`).
	 */
	[[nodiscard]] constexpr auto data(this auto& self) noexcept
	{
		return self.fields.data();
	}

	/**
	 * @brief Returns the number of dimensions.
	 * @return `Dim`.
	 */
	[[nodiscard]] static constexpr std::size_t size() noexcept { return Dim; }

	/**
	 * @brief Compares two vectors for equality (component-wise).
	 * @param [in] rhs The vector to compare against.
	 * @return `true` iff all corresponding elements are equal.
	 */
	constexpr bool operator==(Vec const&) const = default;

	/**************************************************************************************
	|                                                                                     |
	|                              Unary arithmetic operator                              |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Unary identity operator.
	 * @return A copy of this vector, unchanged.
	 */
	constexpr Vec operator+() const noexcept { return *this; }

	/**
	 * @brief Unary negation operator.
	 * @return A vector with each element negated.
	 */
	constexpr Vec operator-() const noexcept
	{
		Vec result;
		std::ranges::transform(*this, result.begin(), std::negate{});
		return result;
	}

	/**
	 * @brief Bitwise NOT operator (integral types only).
	 * @return A vector with each element bitwise-complemented.
	 */
	constexpr Vec operator~() const noexcept
	  requires std::is_integral_v<T>
	{
		Vec result;
		std::ranges::transform(*this, result.begin(), std::bit_not{});
		return result;
	}

	/**************************************************************************************
	|                                                                                     |
	|                            Compound assignment operator                             |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Adds each component of `rhs` to the corresponding component.
	 * @param [in] rhs Right-hand side vector.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator+=(Vec const& rhs) noexcept
	{
		std::ranges::transform(*this, rhs, begin(), std::plus{});
		return *this;
	}

	/**
	 * @brief Subtracts each component of `rhs` from the corresponding component.
	 * @param [in] rhs Right-hand side vector.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator-=(Vec const& rhs) noexcept
	{
		std::ranges::transform(*this, rhs, begin(), std::minus{});
		return *this;
	}

	/**
	 * @brief Multiplies each component by the corresponding component of `rhs`.
	 * @param [in] rhs Right-hand side vector.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator*=(Vec const& rhs) noexcept
	{
		std::ranges::transform(*this, rhs, begin(), std::multiplies{});
		return *this;
	}

	/**
	 * @brief Divides each component by the corresponding component of `rhs`.
	 * @param [in] rhs Right-hand side vector.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator/=(Vec const& rhs) noexcept
	{
		std::ranges::transform(*this, rhs, begin(), std::divides{});
		return *this;
	}

	/**
	 * @brief Computes the remainder of each component divided by the corresponding
	 * component of `rhs` (integral types only).
	 * @param [in] rhs Right-hand side vector.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator%=(Vec const& rhs) noexcept
	  requires std::is_integral_v<T>
	{
		std::ranges::transform(*this, rhs, begin(), std::modulus{});
		return *this;
	}

	/**
	 * @brief Bitwise ANDs each component with the corresponding component of `rhs`
	 * (integral types only).
	 * @param [in] rhs Right-hand side vector.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator&=(Vec const& rhs) noexcept
	  requires std::is_integral_v<T>
	{
		std::ranges::transform(*this, rhs, begin(), std::bit_and{});
		return *this;
	}

	/**
	 * @brief Bitwise ORs each component with the corresponding component of `rhs` (integral
	 * types only).
	 * @param [in] rhs Right-hand side vector.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator|=(Vec const& rhs) noexcept
	  requires std::is_integral_v<T>
	{
		std::ranges::transform(*this, rhs, begin(), std::bit_or{});
		return *this;
	}

	/**
	 * @brief Bitwise XORs each component with the corresponding component of `rhs`
	 * (integral types only).
	 * @param [in] rhs Right-hand side vector.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator^=(Vec const& rhs) noexcept
	  requires std::is_integral_v<T>
	{
		std::ranges::transform(*this, rhs, begin(), std::bit_xor{});
		return *this;
	}

	/**
	 * @brief Left-shifts each component by the corresponding component of `rhs` (integral
	 * types only).
	 * @param [in] rhs Per-component shift amounts.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator<<=(Vec const& rhs) noexcept
	  requires std::is_integral_v<T>
	{
		std::ranges::transform(*this, rhs, begin(), [](T a, T b) { return a << b; });
		return *this;
	}

	/**
	 * @brief Right-shifts each component by the corresponding component of `rhs` (integral
	 * types only).
	 * @param [in] rhs Per-component shift amounts.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator>>=(Vec const& rhs) noexcept
	  requires std::is_integral_v<T>
	{
		std::ranges::transform(*this, rhs, begin(), [](T a, T b) { return a >> b; });
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                             Scalar compound assignment operator                     |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Adds scalar `rhs` to every component.
	 * @param [in] rhs Scalar value.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator+=(T rhs) noexcept
	{
		std::ranges::transform(*this, begin(), [rhs](T v) { return v + rhs; });
		return *this;
	}

	/**
	 * @brief Subtracts scalar `rhs` from every component.
	 * @param [in] rhs Scalar value.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator-=(T rhs) noexcept
	{
		std::ranges::transform(*this, begin(), [rhs](T v) { return v - rhs; });
		return *this;
	}

	/**
	 * @brief Multiplies every component by scalar `rhs`.
	 * @param [in] rhs Scalar value.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator*=(T rhs) noexcept
	{
		std::ranges::transform(*this, begin(), [rhs](T v) { return v * rhs; });
		return *this;
	}

	/**
	 * @brief Divides every component by scalar `rhs`.
	 * @param [in] rhs Scalar value.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator/=(T rhs) noexcept
	{
		std::ranges::transform(*this, begin(), [rhs](T v) { return v / rhs; });
		return *this;
	}

	/**
	 * @brief Computes the remainder of every component divided by scalar `rhs`
	 *        (integral types only).
	 * @param [in] rhs Scalar value.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator%=(T rhs) noexcept
	  requires std::is_integral_v<T>
	{
		std::ranges::transform(*this, begin(), [rhs](T v) { return v % rhs; });
		return *this;
	}

	/**
	 * @brief Bitwise ANDs every component with scalar `rhs` (integral types only).
	 * @param [in] rhs Scalar value.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator&=(T rhs) noexcept
	  requires std::is_integral_v<T>
	{
		std::ranges::transform(*this, begin(), [rhs](T v) { return v & rhs; });
		return *this;
	}

	/**
	 * @brief Bitwise ORs every component with scalar `rhs` (integral types only).
	 * @param [in] rhs Scalar value.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator|=(T rhs) noexcept
	  requires std::is_integral_v<T>
	{
		std::ranges::transform(*this, begin(), [rhs](T v) { return v | rhs; });
		return *this;
	}

	/**
	 * @brief Bitwise XORs every component with scalar `rhs` (integral types only).
	 * @param [in] rhs Scalar value.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator^=(T rhs) noexcept
	  requires std::is_integral_v<T>
	{
		std::ranges::transform(*this, begin(), [rhs](T v) { return v ^ rhs; });
		return *this;
	}

	/**
	 * @brief Left-shifts every component by scalar `rhs` (integral types only).
	 * @param [in] rhs Shift amount.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator<<=(T rhs) noexcept
	  requires std::is_integral_v<T>
	{
		std::ranges::transform(*this, begin(), [rhs](T v) { return v <<= rhs; });
		return *this;
	}

	/**
	 * @brief Right-shifts every component by scalar `rhs` (integral types only).
	 * @param [in] rhs Shift amount.
	 * @return Reference to `*this`.
	 */
	constexpr Vec& operator>>=(T rhs) noexcept
	  requires std::is_integral_v<T>
	{
		std::ranges::transform(*this, begin(), [rhs](T v) { return v >>= rhs; });
		return *this;
	}
};

/**************************************************************************************
|                                                                                     |
|                                   Type Aliases                                      |
|                                                                                     |
**************************************************************************************/

using Vec1b = Vec<1, bool>;
using Vec2b = Vec<2, bool>;
using Vec3b = Vec<3, bool>;
using Vec4b = Vec<4, bool>;
using Vec1i = Vec<1, int>;
using Vec2i = Vec<2, int>;
using Vec3i = Vec<3, int>;
using Vec4i = Vec<4, int>;
using Vec1u = Vec<1, unsigned>;
using Vec2u = Vec<2, unsigned>;
using Vec3u = Vec<3, unsigned>;
using Vec4u = Vec<4, unsigned>;
using Vec1f = Vec<1, float>;
using Vec2f = Vec<2, float>;
using Vec3f = Vec<3, float>;
using Vec4f = Vec<4, float>;
using Vec1d = Vec<1, double>;
using Vec2d = Vec<2, double>;
using Vec3d = Vec<3, double>;
using Vec4d = Vec<4, double>;

/**************************************************************************************
|                                                                                     |
|                                      Concepts                                       |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Concept satisfied by any specialization of `Vec<Dim, U>`.
 * @tparam T Type to check.
 */
template <class T>
concept VecType =
    requires(T const& t) { []<std::size_t Dim, class U>(Vec<Dim, U> const&) {}(t); };

/**************************************************************************************
|                                                                                     |
|                                  Binary operators                                   |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Component-wise addition of two vectors.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs[i] + rhs[i]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> operator+(Vec<Dim, T>        lhs,
                                              Vec<Dim, T> const& rhs) noexcept
{
	lhs += rhs;
	return lhs;
}

/**
 * @brief Component-wise subtraction of two vectors.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs[i] - rhs[i]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> operator-(Vec<Dim, T>        lhs,
                                              Vec<Dim, T> const& rhs) noexcept
{
	lhs -= rhs;
	return lhs;
}

/**
 * @brief Component-wise multiplication of two vectors.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs[i] * rhs[i]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> operator*(Vec<Dim, T>        lhs,
                                              Vec<Dim, T> const& rhs) noexcept
{
	lhs *= rhs;
	return lhs;
}

/**
 * @brief Component-wise division of two vectors.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs[i] / rhs[i]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> operator/(Vec<Dim, T>        lhs,
                                              Vec<Dim, T> const& rhs) noexcept
{
	lhs /= rhs;
	return lhs;
}

/**
 * @brief Component-wise modulo of two integral vectors.
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs[i] % rhs[i]`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator%(Vec<Dim, T>        lhs,
                                              Vec<Dim, T> const& rhs) noexcept
{
	lhs %= rhs;
	return lhs;
}

/**
 * @brief Component-wise bitwise AND of two integral vectors.
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs[i] & rhs[i]`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator&(Vec<Dim, T>        lhs,
                                              Vec<Dim, T> const& rhs) noexcept
{
	lhs &= rhs;
	return lhs;
}

/**
 * @brief Component-wise bitwise OR of two integral vectors.
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs[i] | rhs[i]`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator|(Vec<Dim, T>        lhs,
                                              Vec<Dim, T> const& rhs) noexcept
{
	lhs |= rhs;
	return lhs;
}

/**
 * @brief Component-wise bitwise XOR of two integral vectors.
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs[i] ^ rhs[i]`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator^(Vec<Dim, T>        lhs,
                                              Vec<Dim, T> const& rhs) noexcept
{
	lhs ^= rhs;
	return lhs;
}

/**
 * @brief Component-wise left shift of an integral vector by a per-element amount.
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Per-component shift amounts.
 * @return A vector whose element `i` is `lhs[i] << rhs[i]`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator<<(Vec<Dim, T>        lhs,
                                               Vec<Dim, T> const& rhs) noexcept
{
	lhs <<= rhs;
	return lhs;
}

/**
 * @brief Component-wise right shift of an integral vector by a per-element amount.
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Per-component shift amounts.
 * @return A vector whose element `i` is `lhs[i] >> rhs[i]`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator>>(Vec<Dim, T>        lhs,
                                               Vec<Dim, T> const& rhs) noexcept
{
	lhs >>= rhs;
	return lhs;
}

/**
 * @brief Component-wise logical AND of two bool vectors.
 * @tparam Dim Number of dimensions.
 * @param [in] lhs Left-hand side bool vector.
 * @param [in] rhs Right-hand side bool vector.
 * @return A `Vec<Dim, bool>` where element `i` is `lhs[i] && rhs[i]`.
 */
template <std::size_t Dim>
[[nodiscard]] constexpr Vec<Dim, bool> operator&&(Vec<Dim, bool>        lhs,
                                                  Vec<Dim, bool> const& rhs) noexcept
{
	std::ranges::transform(lhs, rhs, lhs.begin(), std::logical_and{});
	return lhs;
}

/**
 * @brief Component-wise logical OR of two bool vectors.
 * @tparam Dim Number of dimensions.
 * @param [in] lhs Left-hand side bool vector.
 * @param [in] rhs Right-hand side bool vector.
 * @return A `Vec<Dim, bool>` where element `i` is `lhs[i] || rhs[i]`.
 */
template <std::size_t Dim>
[[nodiscard]] constexpr Vec<Dim, bool> operator||(Vec<Dim, bool>        lhs,
                                                  Vec<Dim, bool> const& rhs) noexcept
{
	std::ranges::transform(lhs, rhs, lhs.begin(), std::logical_or{});
	return lhs;
}

/**************************************************************************************
|                                                                                     |
|                              Scalar binary operators                                |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Adds scalar `rhs` to every component of `lhs`.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Scalar value.
 * @return A vector whose element `i` is `lhs[i] + rhs`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> operator+(Vec<Dim, T> lhs, T rhs) noexcept
{
	lhs += rhs;
	return lhs;
}

/**
 * @brief Subtracts scalar `rhs` from every component of `lhs`.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Scalar value.
 * @return A vector whose element `i` is `lhs[i] - rhs`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> operator-(Vec<Dim, T> lhs, T rhs) noexcept
{
	lhs -= rhs;
	return lhs;
}

/**
 * @brief Multiplies every component of `lhs` by scalar `rhs`.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Scalar value.
 * @return A vector whose element `i` is `lhs[i] * rhs`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> operator*(Vec<Dim, T> lhs, T rhs) noexcept
{
	lhs *= rhs;
	return lhs;
}

/**
 * @brief Divides every component of `lhs` by scalar `rhs`.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Scalar value.
 * @return A vector whose element `i` is `lhs[i] / rhs`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> operator/(Vec<Dim, T> lhs, T rhs) noexcept
{
	lhs /= rhs;
	return lhs;
}

/**
 * @brief Computes the remainder of every component of `lhs` divided by scalar `rhs`
 *        (integral types only).
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Scalar value.
 * @return A vector whose element `i` is `lhs[i] % rhs`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator%(Vec<Dim, T> lhs, T rhs) noexcept
{
	lhs %= rhs;
	return lhs;
}

/**
 * @brief Bitwise ANDs every component of `lhs` with scalar `rhs` (integral types only).
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Scalar value.
 * @return A vector whose element `i` is `lhs[i] & rhs`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator&(Vec<Dim, T> lhs, T rhs) noexcept
{
	lhs &= rhs;
	return lhs;
}

/**
 * @brief Bitwise ORs every component of `lhs` with scalar `rhs` (integral types only).
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Scalar value.
 * @return A vector whose element `i` is `lhs[i] | rhs`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator|(Vec<Dim, T> lhs, T rhs) noexcept
{
	lhs |= rhs;
	return lhs;
}

/**
 * @brief Bitwise XORs every component of `lhs` with scalar `rhs` (integral types only).
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Scalar value.
 * @return A vector whose element `i` is `lhs[i] ^ rhs`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator^(Vec<Dim, T> lhs, T rhs) noexcept
{
	lhs ^= rhs;
	return lhs;
}

/**
 * @brief Left-shifts every component of `lhs` by scalar `rhs` (integral types only).
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Shift amount.
 * @return A vector whose element `i` is `lhs[i] << rhs`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator<<(Vec<Dim, T> lhs, T rhs) noexcept
{
	lhs <<= rhs;
	return lhs;
}

/**
 * @brief Right-shifts every component of `lhs` by scalar `rhs` (integral types only).
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Left-hand side vector.
 * @param [in] rhs Shift amount.
 * @return A vector whose element `i` is `lhs[i] >> rhs`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator>>(Vec<Dim, T> lhs, T rhs) noexcept
{
	lhs >>= rhs;
	return lhs;
}

/**
 * @brief Adds scalar `lhs` to every component of `rhs`.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] lhs Scalar value.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs + rhs[i]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> operator+(T lhs, Vec<Dim, T> rhs) noexcept
{
	rhs += lhs;
	return rhs;
}

/**
 * @brief Subtracts every component of `rhs` from scalar `lhs`.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] lhs Scalar value.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs - rhs[i]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> operator-(T lhs, Vec<Dim, T> rhs) noexcept
{
	std::ranges::transform(rhs, rhs.begin(), [lhs](T v) { return lhs - v; });
	return rhs;
}

/**
 * @brief Multiplies scalar `lhs` by every component of `rhs`.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] lhs Scalar value.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs * rhs[i]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> operator*(T lhs, Vec<Dim, T> rhs) noexcept
{
	rhs *= lhs;
	return rhs;
}

/**
 * @brief Divides scalar `lhs` by every component of `rhs`.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] lhs Scalar value.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs / rhs[i]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> operator/(T lhs, Vec<Dim, T> rhs) noexcept
{
	std::ranges::transform(rhs, rhs.begin(), [lhs](T v) { return lhs / v; });
	return rhs;
}

/**
 * @brief Computes the remainder of scalar `lhs` divided by every component of `rhs`
 *        (integral types only).
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Scalar value.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs % rhs[i]`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator%(T lhs, Vec<Dim, T> rhs) noexcept
{
	std::ranges::transform(rhs, rhs.begin(), [lhs](T v) { return lhs % v; });
	return rhs;
}

/**
 * @brief Bitwise ANDs scalar `lhs` with every component of `rhs` (integral types only).
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Scalar value.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs & rhs[i]`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator&(T lhs, Vec<Dim, T> rhs) noexcept
{
	rhs &= lhs;
	return rhs;
}

/**
 * @brief Bitwise ORs scalar `lhs` with every component of `rhs` (integral types only).
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Scalar value.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs | rhs[i]`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator|(T lhs, Vec<Dim, T> rhs) noexcept
{
	rhs |= lhs;
	return rhs;
}

/**
 * @brief Bitwise XORs scalar `lhs` with every component of `rhs` (integral types only).
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Scalar value.
 * @param [in] rhs Right-hand side vector.
 * @return A vector whose element `i` is `lhs ^ rhs[i]`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator^(T lhs, Vec<Dim, T> rhs) noexcept
{
	rhs ^= lhs;
	return rhs;
}

/**
 * @brief Left-shifts scalar `lhs` by every component of `rhs` (integral types only).
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Scalar value.
 * @param [in] rhs Per-component shift amounts.
 * @return A vector whose element `i` is `lhs << rhs[i]`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator<<(T lhs, Vec<Dim, T> rhs) noexcept
{
	std::ranges::transform(rhs, rhs.begin(), [lhs](T v) { return lhs << v; });
	return rhs;
}

/**
 * @brief Right-shifts scalar `lhs` by every component of `rhs` (integral types only).
 * @tparam Dim Number of dimensions.
 * @tparam T Integral element type.
 * @param [in] lhs Scalar value.
 * @param [in] rhs Per-component shift amounts.
 * @return A vector whose element `i` is `lhs >> rhs[i]`.
 */
template <std::size_t Dim, std::integral T>
[[nodiscard]] constexpr Vec<Dim, T> operator>>(T lhs, Vec<Dim, T> rhs) noexcept
{
	std::ranges::transform(rhs, rhs.begin(), [lhs](T v) { return lhs >> v; });
	return rhs;
}

/**************************************************************************************
|                                                                                     |
|                                      Functions                                      |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Casts each element of a vector to a new type.
 * @tparam T Target element type.
 * @tparam Dim Number of dimensions.
 * @tparam U Source element type.
 * @param [in] v The source vector.
 * @return A new vector with elements cast to type `T`.
 */
template <class T, std::size_t Dim, class U>
[[nodiscard]] constexpr Vec<Dim, T> cast(Vec<Dim, U> const& v) noexcept
{
	return Vec<Dim, T>(v);
}

/**
 * @brief Converts a vector to a different Vec type, truncating or zero-padding
 * dimensions.
 * @tparam To Target Vec type (e.g., `Vec3f`).
 * @tparam Dim Source number of dimensions.
 * @tparam U Source element type.
 * @param [in] v The source vector.
 * @return A new vector of type `To`, copying the overlapping dimensions.
 */
template <VecType To, std::size_t Dim, class U>
[[nodiscard]] constexpr To convert(Vec<Dim, U> const& v) noexcept
{
	using T = typename To::value_type;
	To                res{};
	std::size_t const D = std::min(res.size(), v.size());
	for (std::size_t i{}; D > i; ++i) {
		res[i] = static_cast<T>(v[i]);
	}
	return res;
}

/**
 * @brief Computes the dot (inner) product of two vectors.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] a First vector.
 * @param [in] b Second vector.
 * @return The scalar dot product `sum(a[i] * b[i])`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr T dot(Vec<Dim, T> const& a, Vec<Dim, T> const& b) noexcept
{
	return std::ranges::fold_left(std::views::zip_transform(std::multiplies{}, a, b), T{},
	                              std::plus{});
}

/**
 * @brief Computes the Euclidean length (magnitude) of a vector.
 * @tparam Dim Number of dimensions.
 * @tparam T Floating-point element type.
 * @param [in] v The vector.
 * @return `sqrt(dot(v, v))`.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr T length(Vec<Dim, T> const& v) noexcept
{
	return std::sqrt(dot(v, v));
}

/**
 * @brief Computes the Euclidean distance between two points.
 * @tparam Dim Number of dimensions.
 * @tparam T Floating-point element type.
 * @param [in] a First point.
 * @param [in] b Second point.
 * @return `length(a - b)`.
 */
template <std::size_t Dim, class T>
  requires std::floating_point<T>
[[nodiscard]] constexpr T distance(Vec<Dim, T> const& a, Vec<Dim, T> const& b) noexcept
{
	return length(a - b);
}

/**
 * @brief Computes the squared Euclidean distance between two points.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] a First point.
 * @param [in] b Second point.
 * @return `dot(a - b, a - b)`.
 *
 * @details Avoids the square root of `distance()`, making it cheaper for comparisons.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr T distanceSquared(Vec<Dim, T> const& a,
                                          Vec<Dim, T> const& b) noexcept
{
	return dot(a - b, a - b);
}

/**
 * @brief Returns a unit-length vector in the same direction as `v`.
 * @tparam Dim Number of dimensions.
 * @tparam T Floating-point element type.
 * @param [in] v The vector to normalize.
 * @return `v / length(v)`.
 *
 * @details Undefined behaviour if `v` is the zero vector.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr Vec<Dim, T> normalize(Vec<Dim, T> const& v) noexcept
{
	return v / length(v);
}

/**
 * @brief Checks whether a vector has unit length.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v The vector to check.
 * @return `true` if `|dot(v,v) - 1| <= epsilon * 8` for floating-point types,
 *         or `dot(v,v) == 1` for integral types.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr bool isNormalized(Vec<Dim, T> const& v) noexcept
{
	if constexpr (std::floating_point<T>) {
		return std::abs(dot(v, v) - T(1)) <= std::numeric_limits<T>::epsilon() * T(8);
	} else {
		return dot(v, v) == T(1);
	}
}

/**
 * @brief Computes the cross product of two 3D vectors.
 * @tparam T Element type.
 * @param [in] a First vector.
 * @param [in] b Second vector.
 * @return A vector perpendicular to both `a` and `b` with magnitude `|a||b|sin(θ)`.
 */
template <class T>
[[nodiscard]] constexpr Vec<3, T> cross(Vec<3, T> const& a, Vec<3, T> const& b) noexcept
{
	return Vec<3, T>{(a[1] * b[2]) - (a[2] * b[1]), (a[2] * b[0]) - (a[0] * b[2]),
	                 (a[0] * b[1]) - (a[1] * b[0])};
}

/**
 * @brief Reflects an incident vector about a surface normal.
 * @tparam Dim Number of dimensions.
 * @tparam T Floating-point element type.
 * @param [in] v The incident vector.
 * @param [in] n The surface normal (should be normalized).
 * @return `v - 2 * dot(v, n) * n`.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr Vec<Dim, T> reflect(Vec<Dim, T> const& v,
                                            Vec<Dim, T> const& n) noexcept
{
	return v - T(2) * dot(v, n) * n;
}

/**
 * @brief Computes the refraction vector for an incident ray entering a new medium.
 * @tparam Dim Number of dimensions.
 * @tparam T Floating-point element type.
 * @param [in] v The normalized incident direction.
 * @param [in] n The normalized surface normal.
 * @param [in] eta The ratio of indices of refraction (n_incident / n_transmitted).
 * @return The refracted direction, or the zero vector for total internal reflection.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr Vec<Dim, T> refract(Vec<Dim, T> const& v, Vec<Dim, T> const& n,
                                            T eta) noexcept
{
	assert(isNormalized(v) && isNormalized(n) && "Input vectors should be normalized");
	Vec<Dim, T> tmp = dot(v, n);
	Vec<Dim, T> k   = T(1) - eta * eta * (T(1) - tmp * tmp);
	return k < T(0) ? Vec<Dim, T>{} : eta * v - (eta * tmp + sqrt(k)) * n;
}

/**
 * @brief Returns the component-wise minimum of two vectors.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v1 First vector.
 * @param [in] v2 Second vector.
 * @return A vector whose each element is `min(v1[i], v2[i])`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> min(Vec<Dim, T> v1, Vec<Dim, T> const& v2) noexcept
{
	std::ranges::transform(v1, v2, v1.begin(), [](T a, T b) { return std::min(a, b); });
	return v1;
}

/**
 * @brief Returns the smallest element in a vector.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v The vector.
 * @return The minimum element value.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr T min(Vec<Dim, T> const& v) noexcept
{
	return *std::ranges::min_element(v);
}

/**
 * @brief Returns the component-wise maximum of two vectors.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v1 First vector.
 * @param [in] v2 Second vector.
 * @return A vector whose each element is `max(v1[i], v2[i])`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> max(Vec<Dim, T> v1, Vec<Dim, T> const& v2) noexcept
{
	std::ranges::transform(v1, v2, v1.begin(), [](T a, T b) { return std::max(a, b); });
	return v1;
}

/**
 * @brief Returns the largest element in a vector.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v The vector.
 * @return The maximum element value.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr T max(Vec<Dim, T> const& v) noexcept
{
	return *std::ranges::max_element(v);
}

/**
 * @brief Returns the index of the smallest element in a vector.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v The vector.
 * @return The zero-based index of the minimum element.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr std::size_t minIndex(Vec<Dim, T> const& v) noexcept
{
	return static_cast<std::size_t>(std::ranges::min_element(v) - v.begin());
}

/**
 * @brief Returns the index of the largest element in a vector.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v The vector.
 * @return The zero-based index of the maximum element.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr std::size_t maxIndex(Vec<Dim, T> const& v) noexcept
{
	return static_cast<std::size_t>(std::ranges::max_element(v) - v.begin());
}

/**
 * @brief Computes the sum of all elements in a vector.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v The vector.
 * @return `v[0] + v[1] + ... + v[Dim-1]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr T sum(Vec<Dim, T> const& v) noexcept
{
	return std::ranges::fold_left(v, T{}, std::plus{});
}

/**
 * @brief Computes the product of all elements in a vector.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v The vector.
 * @return `v[0] * v[1] * ... * v[Dim-1]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr T product(Vec<Dim, T> const& v) noexcept
{
	return std::ranges::fold_left(v, T{1}, std::multiplies{});
}

/**
 * @brief Returns the component-wise absolute value of a vector.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v The vector.
 * @return A vector whose each element is `abs(v[i])`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> abs(Vec<Dim, T> v) noexcept
{
	std::ranges::transform(v, v.begin(), [](T x) { return std::abs(x); });
	return v;
}

/**
 * @brief Clamps each component of a vector to the range `[lo[i], hi[i]]`.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v The vector to clamp.
 * @param [in] lo Per-component lower bounds.
 * @param [in] hi Per-component upper bounds.
 * @return A vector whose each element is `clamp(v[i], lo[i], hi[i])`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> clamp(Vec<Dim, T> v, Vec<Dim, T> const& lo,
                                          Vec<Dim, T> const& hi) noexcept
{
	for (auto [vi, li, hi_i] : std::views::zip(v, lo, hi)) {
		vi = std::clamp(vi, li, hi_i);
	}
	return v;
}

/**
 * @brief Returns the component-wise ceiling of a floating-point vector.
 * @tparam Dim Number of dimensions.
 * @tparam T Floating-point element type.
 * @param [in] v The vector.
 * @return A vector whose each element is `ceil(v[i])`.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr Vec<Dim, T> ceil(Vec<Dim, T> v) noexcept
{
	std::ranges::transform(v, v.begin(), [](T x) { return std::ceil(x); });
	return v;
}

/**
 * @brief Returns the component-wise floor of a floating-point vector.
 * @tparam Dim Number of dimensions.
 * @tparam T Floating-point element type.
 * @param [in] v The vector.
 * @return A vector whose each element is `floor(v[i])`.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr Vec<Dim, T> floor(Vec<Dim, T> v) noexcept
{
	std::ranges::transform(v, v.begin(), [](T x) { return std::floor(x); });
	return v;
}

/**
 * @brief Returns the component-wise truncation toward zero of a floating-point vector.
 * @tparam Dim Number of dimensions.
 * @tparam T Floating-point element type.
 * @param [in] v The vector.
 * @return A vector whose each element is `trunc(v[i])`.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr Vec<Dim, T> trunc(Vec<Dim, T> v) noexcept
{
	std::ranges::transform(v, v.begin(), [](T x) { return std::trunc(x); });
	return v;
}

/**
 * @brief Returns the component-wise rounding to nearest integer of a floating-point
 * vector.
 * @tparam Dim Number of dimensions.
 * @tparam T Floating-point element type.
 * @param [in] v The vector.
 * @return A vector whose each element is `round(v[i])`.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr Vec<Dim, T> round(Vec<Dim, T> v) noexcept
{
	std::ranges::transform(v, v.begin(), [](T x) { return std::round(x); });
	return v;
}

/**
 * @brief Returns a bool vector indicating component-wise equality.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v1 First vector.
 * @param [in] v2 Second vector.
 * @return A `Vec<Dim, bool>` where element `i` is `v1[i] == v2[i]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, bool> equal(Vec<Dim, T> const& v1,
                                             Vec<Dim, T> const& v2) noexcept
{
	Vec<Dim, bool> res;
	std::ranges::transform(v1, v2, res.begin(), std::equal_to{});
	return res;
}

/**
 * @brief Returns a bool vector indicating component-wise inequality.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v1 First vector.
 * @param [in] v2 Second vector.
 * @return A `Vec<Dim, bool>` where element `i` is `v1[i] != v2[i]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, bool> notEqual(Vec<Dim, T> const& v1,
                                                Vec<Dim, T> const& v2) noexcept
{
	Vec<Dim, bool> res;
	std::ranges::transform(v1, v2, res.begin(), std::not_equal_to{});
	return res;
}

/**
 * @brief Returns a bool vector indicating component-wise less-than.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v1 First vector.
 * @param [in] v2 Second vector.
 * @return A `Vec<Dim, bool>` where element `i` is `v1[i] < v2[i]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, bool> less(Vec<Dim, T> const& v1,
                                            Vec<Dim, T> const& v2) noexcept
{
	Vec<Dim, bool> res;
	std::ranges::transform(v1, v2, res.begin(), std::less{});
	return res;
}

/**
 * @brief Returns a bool vector indicating component-wise less-than-or-equal.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v1 First vector.
 * @param [in] v2 Second vector.
 * @return A `Vec<Dim, bool>` where element `i` is `v1[i] <= v2[i]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, bool> lessEqual(Vec<Dim, T> const& v1,
                                                 Vec<Dim, T> const& v2) noexcept
{
	Vec<Dim, bool> res;
	std::ranges::transform(v1, v2, res.begin(), std::less_equal{});
	return res;
}

/**
 * @brief Returns a bool vector indicating component-wise greater-than.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v1 First vector.
 * @param [in] v2 Second vector.
 * @return A `Vec<Dim, bool>` where element `i` is `v1[i] > v2[i]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, bool> greater(Vec<Dim, T> const& v1,
                                               Vec<Dim, T> const& v2) noexcept
{
	Vec<Dim, bool> res;
	std::ranges::transform(v1, v2, res.begin(), std::greater{});
	return res;
}

/**
 * @brief Returns a bool vector indicating component-wise greater-than-or-equal.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v1 First vector.
 * @param [in] v2 Second vector.
 * @return A `Vec<Dim, bool>` where element `i` is `v1[i] >= v2[i]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, bool> greaterEqual(Vec<Dim, T> const& v1,
                                                    Vec<Dim, T> const& v2) noexcept
{
	Vec<Dim, bool> res;
	std::ranges::transform(v1, v2, res.begin(), std::greater_equal{});
	return res;
}

/**
 * @brief Returns `true` if all components of a bool vector are `true`.
 * @tparam Dim Number of dimensions.
 * @param [in] v The bool vector.
 * @return `true` iff every element is `true`.
 */
template <std::size_t Dim>
[[nodiscard]] constexpr bool all(Vec<Dim, bool> const& v) noexcept
{
	return std::ranges::all_of(v, std::identity{});
}

/**
 * @brief Returns `true` if at least one component of a bool vector is `true`.
 * @tparam Dim Number of dimensions.
 * @param [in] v The bool vector.
 * @return `true` iff at least one element is `true`.
 */
template <std::size_t Dim>
[[nodiscard]] constexpr bool any(Vec<Dim, bool> const& v) noexcept
{
	return std::ranges::any_of(v, std::identity{});
}

/**
 * @brief Returns `true` if some but not all components of a bool vector are `true`.
 * @tparam Dim Number of dimensions.
 * @param [in] v The bool vector.
 * @return `any(v) && !all(v)`.
 */
template <std::size_t Dim>
[[nodiscard]] constexpr bool some(Vec<Dim, bool> const& v) noexcept
{
	return any(v) && !all(v);
}

/**
 * @brief Returns `true` if no component of a bool vector is `true`.
 * @tparam Dim Number of dimensions.
 * @param [in] v The bool vector.
 * @return `true` iff every element is `false`.
 */
template <std::size_t Dim>
[[nodiscard]] constexpr bool none(Vec<Dim, bool> const& v) noexcept
{
	return std::ranges::none_of(v, std::identity{});
}

/**
 * @brief Returns `true` if any component of a floating-point vector is NaN.
 * @tparam Dim Number of dimensions.
 * @tparam T Floating-point element type.
 * @param [in] v The vector.
 * @return `true` iff at least one element satisfies `std::isnan`.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr bool isnan(Vec<Dim, T> const& v) noexcept
{
	return std::ranges::any_of(v, [](T x) { return std::isnan(x); });
}

/**
 * @brief Returns `true` if all components of a floating-point vector are finite.
 * @tparam Dim Number of dimensions.
 * @tparam T Floating-point element type.
 * @param [in] v The vector.
 * @return `true` iff every element satisfies `std::isfinite`.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr bool isfinite(Vec<Dim, T> const& v) noexcept
{
	return std::ranges::all_of(v, [](T x) { return std::isfinite(x); });
}

/**
 * @brief Returns `true` if all components of a floating-point vector are normal.
 * @tparam Dim Number of dimensions.
 * @tparam T Floating-point element type.
 * @param [in] v The vector.
 * @return `true` iff every element satisfies `std::isnormal` (not zero, subnormal, NaN,
 * or inf).
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr bool isnormal(Vec<Dim, T> const& v) noexcept
{
	return std::ranges::all_of(v, [](T x) { return std::isnormal(x); });
}

/**
 * @brief Linearly interpolates between two vectors component-wise.
 * @tparam Dim Number of dimensions.
 * @tparam T Floating-point element type.
 * @param [in] a Start vector (`t == 0`).
 * @param [in] b End vector (`t == 1`).
 * @param [in] t Interpolation parameter.
 * @return A vector whose element `i` is `std::lerp(a[i], b[i], t)`.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr Vec<Dim, T> lerp(Vec<Dim, T> a, Vec<Dim, T> const& b,
                                         T t) noexcept
{
	std::ranges::transform(a, b, a.begin(), [t](T x, T y) { return std::lerp(x, y, t); });
	return a;
}

/**
 * @brief Mixes two vectors using per-component scalar weights.
 * @tparam Dim Number of dimensions.
 * @tparam T Result element type.
 * @tparam U Weight element type.
 * @param [in] x First vector (weight `1 - a[i]`).
 * @param [in] y Second vector (weight `a[i]`).
 * @param [in] a Per-component blend weights.
 * @return `x * (1 - a) + y * a` cast back to `Vec<Dim, T>`.
 */
template <std::size_t Dim, class T, class U>
[[nodiscard]] constexpr Vec<Dim, T> mix(Vec<Dim, T> const& x, Vec<Dim, T> const& y,
                                        Vec<Dim, U> const& a) noexcept
{
	return Vec<Dim, T>(Vec<Dim, U>(x) * (U{1} - a) + Vec<Dim, U>(y) * a);
}

/**
 * @brief Selects between two vectors component-wise using a bool mask.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] x Value selected when `a[i]` is `false`.
 * @param [in] y Value selected when `a[i]` is `true`.
 * @param [in] a Per-component bool selector.
 * @return A vector whose element `i` is `a[i] ? y[i] : x[i]`.
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> mix(Vec<Dim, T> const& x, Vec<Dim, T> const& y,
                                        Vec<Dim, bool> const& a) noexcept
{
	Vec<Dim, T> res;
	for (auto [ri, xi, yi, ai] : std::views::zip(res, x, y, a)) {
		ri = ai ? yi : xi;
	}
	return res;
}

/**
 * @brief Returns the component-wise sign of a vector.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] x The vector.
 * @return A vector whose element `i` is `sign(x[i])` (−1, 0, or 1).
 */
template <std::size_t Dim, class T>
[[nodiscard]] constexpr Vec<Dim, T> sign(Vec<Dim, T> const& x) noexcept
{
	Vec<Dim, T> res;
	std::ranges::transform(x, res.begin(), [](T v) { return static_cast<T>(sign(v)); });
	return res;
}

/**
 * @brief Multiplies each component of a floating-point vector by `2^exp`.
 * @tparam Dim Number of dimensions.
 * @tparam T Floating-point element type.
 * @param [in] v The vector.
 * @param [in] exp The exponent.
 * @return A vector whose element `i` is `std::ldexp(v[i], exp)`.
 */
template <std::size_t Dim, std::floating_point T>
[[nodiscard]] constexpr Vec<Dim, T> ldexp(Vec<Dim, T> v, int exp) noexcept
{
	std::ranges::transform(v, v.begin(), [exp](T x) { return std::ldexp(x, exp); });
	return v;
}

/**************************************************************************************
|                                                                                     |
|                               Structured bindings                                   |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Returns a reference to the `I`-th element for structured bindings.
 * @tparam I Zero-based compile-time index (must be less than `Dim`).
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v The vector.
 * @return Lvalue reference to `v[I]`.
 */
template <std::size_t I, std::size_t Dim, class T>
  requires(I < Dim)
[[nodiscard]] constexpr T& get(Vec<Dim, T>& v) noexcept
{
	return v[I];
}

/**
 * @brief Returns a const reference to the `I`-th element for structured bindings.
 * @tparam I Zero-based compile-time index (must be less than `Dim`).
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v The vector.
 * @return Const lvalue reference to `v[I]`.
 */
template <std::size_t I, std::size_t Dim, class T>
  requires(I < Dim)
[[nodiscard]] constexpr T const& get(Vec<Dim, T> const& v) noexcept
{
	return v[I];
}

/**
 * @brief Returns an rvalue reference to the `I`-th element for structured bindings.
 * @tparam I Zero-based compile-time index (must be less than `Dim`).
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in] v The vector (rvalue).
 * @return Rvalue reference to `v[I]`.
 */
template <std::size_t I, std::size_t Dim, class T>
  requires(I < Dim)
[[nodiscard]] constexpr T&& get(Vec<Dim, T>&& v) noexcept
{
	return std::move(v[I]);
}

/**************************************************************************************
|                                                                                     |
|                                       Print                                         |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Writes a human-readable representation of a vector to a stream.
 * @tparam Dim Number of dimensions.
 * @tparam T Element type.
 * @param [in,out] out The output stream.
 * @param [in] v The vector to print.
 * @return Reference to `out`.
 *
 * @details
 * Outputs each component labeled by its name (`x`, `y`, `z`, `w`), separated by spaces.
 * Example output for a `Vec3f{1, 2, 3}`: `x: 1 y: 2 z: 3`.
 */
template <std::size_t Dim, class T>
std::ostream& operator<<(std::ostream& out, Vec<Dim, T> const& v)
{
	static constexpr std::array names = {'x', 'y', 'z', 'w'};
	for (auto const [i, name] : std::views::enumerate(names)) {
		if (i) out << ' ';
		out << name << ": " << v[i];
	}
	return out;
}
}  // namespace ufo

template <std::size_t Dim, class T>
struct std::tuple_size<ufo::Vec<Dim, T>> : std::integral_constant<std::size_t, Dim> {
};

template <std::size_t I, std::size_t Dim, class T>
struct std::tuple_element<I, ufo::Vec<Dim, T>> {
	using type = T;
};

template <std::size_t Dim, class T>
  requires std::formattable<T, char>
struct std::formatter<ufo::Vec<Dim, T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::Vec<Dim, T> const& v, std::format_context& ctx) const
	{
		static constexpr std::array names = {'x', 'y', 'z', 'w'};
		auto                        out   = ctx.out();
		for (auto const [i, name] : std::views::enumerate(names)) {
			if (i) out = std::format_to(out, " ");
			out = std::format_to(out, "{}: {}", name, v[i]);
		}
		return out;
	}
};

#endif  // UFO_MATH_VEC_HPP
