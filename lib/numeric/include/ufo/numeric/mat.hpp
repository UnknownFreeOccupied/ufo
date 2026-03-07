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

#ifndef UFO_NUMERIC_MAT_HPP
#define UFO_NUMERIC_MAT_HPP

// UFO
#include <ufo/numeric/vec.hpp>

// STL
#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <format>
#include <functional>
#include <limits>
#include <numbers>
#include <optional>
#include <ostream>
#include <ranges>
#include <span>
#include <type_traits>
#include <utility>

/**
 * @ingroup numeric
 * @{
 */

namespace ufo
{
// Forward declaration
template <std::size_t Rows, std::size_t Cols, class T>
struct Mat;

/**************************************************************************************
|                                                                                     |
|                                      Concepts                                       |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Concept satisfied by any specialization of `Mat<Rows, Cols, U>`.
 * @tparam T Type to check.
 */
template <class T>
concept Matrix = requires(T const& t) {
	[]<std::size_t R, std::size_t C, class U>(Mat<R, C, U> const&) {}(t);
};

/**
 * @brief Concept satisfied by square matrix types (`Rows == Cols`).
 * @tparam M Matrix type satisfying `Matrix`.
 */
template <class M>
concept SquareMatrix = Matrix<M> && (M::rows() == M::cols());

/**
 * @brief Concept satisfied by invertible matrices (square with floating-point elements).
 * @tparam M Matrix type satisfying `SquareMatrix`.
 */
template <class M>
concept InvertibleMatrix = SquareMatrix<M> && std::floating_point<typename M::value_type>;

/**
 * @brief A fixed-size matrix with `Rows` rows and `Cols` columns.
 * @tparam Rows Number of rows.
 * @tparam Cols Number of columns.
 * @tparam T Arithmetic element type (e.g., `float`, `double`).
 *
 * @details Storage is row-major: `fields[r]` gives row `r` as a `Vec<Cols, T>`.
 */
template <std::size_t Rows, std::size_t Cols, class T>
struct Mat {
	using value_type = T;
	using size_type  = std::size_t;
	using row_type   = Vec<Cols, T>;
	using col_type   = Vec<Rows, T>;

	std::array<row_type, Rows> fields{};

	/// @brief Default-constructs a zero-initialized matrix.
	constexpr Mat() noexcept = default;

	/**
	 * @brief Constructs a matrix from exactly `Rows * Cols` element values (row-major).
	 * @tparam Args Argument types, each convertible to `T`.
	 * @param [in] args Element values in row-major order.
	 */
	template <std::convertible_to<T>... Args>
	  requires(sizeof...(Args) == Rows * Cols)
	explicit(sizeof...(Args) == 1) constexpr Mat(Args... args) noexcept
	{
		auto elements = std::array<T, Rows * Cols>{static_cast<T>(args)...};
		for (std::size_t r = 0; r < Rows; ++r) {
			for (std::size_t c = 0; c < Cols; ++c) {
				fields[r][c] = elements[r * Cols + c];
			}
		}
	}

	/**
	 * @brief Returns a zero-initialized matrix.
	 * @return A matrix with all elements set to `T(0)`.
	 */
	[[nodiscard]] static consteval Mat zeros() noexcept { return {}; }

	/**
	 * @brief Returns the identity matrix (square matrices only).
	 * @return A matrix with `T(1)` on the diagonal and `T(0)` elsewhere.
	 */
	[[nodiscard]] static consteval Mat identity() noexcept
	  requires(Rows == Cols)
	{
		Mat m{};
		for (std::size_t i = 0; i < Rows; ++i) {
			m[i][i] = T(1);
		}
		return m;
	}

	/**
	 * @brief Returns a matrix with all elements set to `T(1)`.
	 * @return A matrix filled with ones.
	 */
	[[nodiscard]] static consteval Mat ones() noexcept
	{
		Mat m{};
		for (auto& row : m.fields) {
			std::ranges::fill(row.fields, T(1));
		}
		return m;
	}

	/**
	 * @brief Constructs a matrix from `Rows` row vectors.
	 * @tparam Rs Row vector types, each convertible to `row_type`.
	 * @param [in] rs Row vectors in order from row 0 to row `Rows - 1`.
	 */
	template <std::convertible_to<row_type>... Rs>
	  requires(sizeof...(Rs) == Rows)
	constexpr Mat(Rs&&... rs) noexcept
	    : fields{static_cast<row_type>(std::forward<Rs>(rs))...}
	{
	}

	/**
	 * @brief Converting constructor from a matrix with a different element type.
	 * @tparam U Source element type.
	 * @param [in] other The source matrix; each element is `static_cast` to `T`.
	 *
	 * @details Implicit when `U` is the same as `T`; `explicit` otherwise.
	 */
	template <class U>
	constexpr explicit(!std::is_same_v<T, U>) Mat(Mat<Rows, Cols, U> const& other) noexcept
	{
		std::ranges::transform(other.fields, fields.begin(),
		                       [](auto const& row) { return row_type(row); });
	}

	/**
	 * @brief Cross-size constructor — copies the overlapping region, zero-fills the rest.
	 * @tparam R2 Source number of rows.
	 * @tparam C2 Source number of columns.
	 * @param [in] other The source matrix (must differ in at least one dimension).
	 */
	template <std::size_t R2, std::size_t C2>
	  requires(R2 != Rows || C2 != Cols)
	constexpr explicit Mat(Mat<R2, C2, T> const& other) noexcept
	{
		// Copy overlapping region, leave rest as zero
		auto const max_r = std::min(Rows, R2);
		auto const max_c = std::min(Cols, C2);
		for (std::size_t r = 0; r < max_r; ++r) {
			for (std::size_t c = 0; c < max_c; ++c) {
				fields[r][c] = other.fields[r][c];
			}
		}
	}

	/**************************************************************************************
	|                                                                                     |
	|                                     Accessors                                      |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Accesses the element at row `r`, column `c`.
	 * @param [in] r Row index.
	 * @param [in] c Column index.
	 * @return Reference to the element (const or non-const, deduced from `this`).
	 */
	constexpr auto& operator[](this auto& self, size_type r, size_type c) noexcept
	{
		return self.fields[r][c];
	}

	/**
	 * @brief Accesses row `i` as a `Vec<Cols, T>`.
	 * @param [in] i Row index.
	 * @return Reference to the row vector (const or non-const, deduced from `this`).
	 */
	constexpr auto& operator[](this auto& self, size_type i) noexcept
	{
		return self.fields[i];
	}

	/**
	 * @brief Bounds-checked access to the element at row `r`, column `c`.
	 * @param [in] r Row index.
	 * @param [in] c Column index.
	 * @return Reference to the element (const or non-const, deduced from `this`).
	 * @throws std::out_of_range if `r >= Rows` or `c >= Cols`.
	 */
	constexpr auto& at(this auto& self, size_type r, size_type c)
	{
		if (r >= Rows || c >= Cols) [[unlikely]] {
			throw std::out_of_range("Matrix index out of bounds");
		}
		return self.fields[r][c];
	}

	/**
	 * @brief Returns a const reference to row `r`.
	 * @param [in] r Row index.
	 * @return Const reference to the `r`-th row vector.
	 */
	[[nodiscard]] constexpr row_type const& row(size_type r) const noexcept
	{
		return fields[r];
	}

	/**
	 * @brief Returns column `c` as a `Vec<Rows, T>` (by value).
	 * @param [in] c Column index.
	 * @return A vector containing all elements in column `c`.
	 */
	[[nodiscard]] constexpr col_type col(size_type c) const noexcept
	{
		col_type result{};
		for (std::size_t r = 0; r < Rows; ++r) {
			result[r] = fields[r][c];
		}
		return result;
	}

	/**
	 * @brief Returns an iterator to the first row.
	 * @return Iterator (const or non-const, deduced from `this`).
	 */
	[[nodiscard]] constexpr auto begin(this auto& self) noexcept
	{
		return self.fields.begin();
	}

	/**
	 * @brief Returns a past-the-end row iterator.
	 * @return Iterator (const or non-const, deduced from `this`).
	 */
	[[nodiscard]] constexpr auto end(this auto& self) noexcept { return self.fields.end(); }

	/**
	 * @brief Returns a pointer to the first row.
	 * @return Pointer (const or non-const, deduced from `this`).
	 */
	[[nodiscard]] constexpr auto data(this auto& self) noexcept
	{
		return self.fields.data();
	}

	/**
	 * @brief Returns the number of rows.
	 * @return `Rows`.
	 */
	[[nodiscard]] static constexpr std::size_t rows() noexcept { return Rows; }

	/**
	 * @brief Returns the number of columns.
	 * @return `Cols`.
	 */
	[[nodiscard]] static constexpr std::size_t cols() noexcept { return Cols; }

	/**
	 * @brief Returns the total number of elements (`Rows * Cols`).
	 * @return `Rows * Cols`.
	 */
	[[nodiscard]] static constexpr std::size_t size() noexcept { return Rows * Cols; }

	/**
	 * @brief Compares two matrices for equality (element-wise).
	 * @param [in] rhs The matrix to compare against.
	 * @return `true` iff all corresponding elements are equal.
	 */
	constexpr bool operator==(Mat const&) const = default;

	/**************************************************************************************
	|                                                                                     |
	|                                Essential Operations                                  |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Computes the trace (sum of diagonal elements) of a square matrix.
	 * @return `fields[0][0] + fields[1][1] + ... + fields[N-1][N-1]`.
	 */
	[[nodiscard]] constexpr T trace() const noexcept
	  requires(Rows == Cols)
	{
		T result{};
		for (std::size_t i = 0; i < Rows; ++i) {
			result += fields[i][i];
		}
		return result;
	}

	/**
	 * @brief Computes the Frobenius norm of the matrix.
	 * @return `sqrt(sum of squares of all elements)`.
	 */
	[[nodiscard]] constexpr T frobenius_norm() const noexcept
	{
		T sum{};
		for (auto const& row : fields) {
			for (auto const& elem : row.fields) {
				sum += elem * elem;
			}
		}
		return std::sqrt(sum);
	}

	/**
	 * @brief Returns `true` if the Frobenius norm is less than `epsilon`.
	 * @param [in] epsilon Tolerance (default: `100 * numeric_limits<T>::epsilon()`).
	 * @return `true` iff `frobenius_norm() < epsilon`.
	 */
	[[nodiscard]] constexpr bool isNearZero(T epsilon = std::numeric_limits<T>::epsilon() *
	                                                    T(100)) const noexcept
	  requires std::floating_point<T>
	{
		return frobenius_norm() < epsilon;
	}

	/**
	 * @brief Estimates the condition number using the Frobenius norm.
	 * @return `frobenius_norm() * frobenius_norm(inverse(*this))`.
	 *
	 * @details This is a rough approximation; the true condition number uses the
	 *          spectral (2-)norm.
	 */
	[[nodiscard]] constexpr T conditionNumber() const noexcept
	  requires InvertibleMatrix<Mat>
	{
		// Estimate using Frobenius norm (rough approximation)
		auto inv = inverse(*this);
		return frobenius_norm() * inv.frobenius_norm();
	}

	/**
	 * @brief Computes the inverse, returning `std::nullopt` for near-singular matrices.
	 * @param [in] epsilon Singularity threshold
	 *             (default: `1000 * numeric_limits<T>::epsilon()`).
	 * @return `inverse(*this)` if `|det| >= epsilon`, otherwise `std::nullopt`.
	 */
	[[nodiscard]] constexpr std::optional<Mat> safe_inverse(
	    T epsilon = std::numeric_limits<T>::epsilon() * T(1000)) const noexcept
	  requires InvertibleMatrix<Mat>
	{
		auto det = determinant(*this);
		if (std::abs(det) < epsilon) {
			return std::nullopt;
		}
		return inverse(*this);
	}

	/**
	 * @brief Returns a flat `std::span<T, Rows * Cols>` over all elements (row-major).
	 * @return Span of all `Rows * Cols` elements (const or non-const, deduced from `this`).
	 */
	[[nodiscard]] constexpr auto flat_view(this auto& self) noexcept
	{
		return std::span{self.data()->data(), Rows * Cols};
	}

	/**
	 * @brief Extracts a square sub-matrix of size `R2 × R2` starting at `(start_row,
	 * start_col)`.
	 * @tparam R2 Size of the square sub-matrix (must satisfy `R2 <= Rows` and `R2 <=
	 * Cols`).
	 * @param [in] start_row Top-left row of the block.
	 * @param [in] start_col Top-left column of the block.
	 * @return A `Mat<R2, R2, T>` containing the extracted block.
	 */
	template <std::size_t R2>
	[[nodiscard]] constexpr auto block(std::size_t start_row,
	                                   std::size_t start_col) const noexcept
	    -> Mat<R2, R2, T>
	  requires(R2 <= Rows && R2 <= Cols)
	{
		Mat<R2, R2, T> result{};
		for (std::size_t r = 0; r < R2; ++r) {
			for (std::size_t c = 0; c < R2; ++c) {
				result[r][c] = fields[start_row + r][start_col + c];
			}
		}
		return result;
	}

	/**
	 * @brief Returns `true` if all elements satisfy the predicate.
	 * @tparam Pred Unary predicate type.
	 * @param [in] pred Predicate to apply to each element.
	 * @return `true` iff `pred(e)` is `true` for every element `e`.
	 */
	template <std::predicate<T> Pred>
	[[nodiscard]] constexpr bool all_of(Pred&& pred) const noexcept
	{
		return std::ranges::all_of(flat_view(), std::forward<Pred>(pred));
	}

	/**
	 * @brief Returns `true` if at least one element satisfies the predicate.
	 * @tparam Pred Unary predicate type.
	 * @param [in] pred Predicate to apply to each element.
	 * @return `true` iff `pred(e)` is `true` for at least one element `e`.
	 */
	template <std::predicate<T> Pred>
	[[nodiscard]] constexpr bool any_of(Pred&& pred) const noexcept
	{
		return std::ranges::any_of(flat_view(), std::forward<Pred>(pred));
	}

	/**
	 * @brief Applies a unary operation to each element and returns the resulting matrix.
	 * @tparam UnaryOp Callable type mapping `T` to `T`.
	 * @param [in] op Operation to apply element-wise.
	 * @return A new matrix where each element is `op(e)`.
	 */
	template <class UnaryOp>
	[[nodiscard]] constexpr Mat transform(UnaryOp&& op) const noexcept
	{
		Mat result{};
		std::ranges::transform(flat_view(), result.flat_view().begin(),
		                       std::forward<UnaryOp>(op));
		return result;
	}

	/**
	 * @brief Returns `false` (storage is row-major).
	 * @return `false`.
	 */
	[[nodiscard]] static constexpr bool is_column_major() noexcept { return false; }

	/**
	 * @brief Returns `true` (storage is row-major).
	 * @return `true`.
	 */
	[[nodiscard]] static constexpr bool is_row_major() noexcept { return true; }

	/**
	 * @brief Returns the size of this matrix type in bytes.
	 * @return `sizeof(Mat<Rows, Cols, T>)`.
	 */
	[[nodiscard]] static constexpr std::size_t memory_footprint() noexcept
	{
		return sizeof(Mat);
	}

	/**
	 * @brief Returns `true` if all off-diagonal elements are within `epsilon` of zero.
	 * @param [in] epsilon Tolerance (default: `numeric_limits<T>::epsilon()`).
	 * @return `true` iff the matrix is diagonal (up to `epsilon`).
	 */
	[[nodiscard]] constexpr bool isDiagonal(
	    T epsilon = std::numeric_limits<T>::epsilon()) const noexcept
	  requires SquareMatrix<Mat>
	{
		for (std::size_t r = 0; r < Rows; ++r) {
			for (std::size_t c = 0; c < Cols; ++c) {
				if (r != c && std::abs(fields[r][c]) > epsilon) {
					return false;
				}
			}
		}
		return true;
	}

	/**
	 * @brief Returns `true` if the matrix equals its transpose (within `epsilon`).
	 * @param [in] epsilon Tolerance (default: `numeric_limits<T>::epsilon()`).
	 * @return `true` iff `|M[r][c] - M[c][r]| <= epsilon` for all `r < c`.
	 */
	[[nodiscard]] constexpr bool isSymmetric(
	    T epsilon = std::numeric_limits<T>::epsilon()) const noexcept
	  requires SquareMatrix<Mat>
	{
		for (std::size_t r = 0; r < Rows; ++r) {
			for (std::size_t c = r + 1; c < Cols; ++c) {
				if (std::abs(fields[r][c] - fields[c][r]) > epsilon) {
					return false;
				}
			}
		}
		return true;
	}

	/**
	 * @brief Returns `true` if all elements below the main diagonal are within `epsilon` of
	 * zero.
	 * @param [in] epsilon Tolerance (default: `numeric_limits<T>::epsilon()`).
	 * @return `true` iff the matrix is upper triangular (up to `epsilon`).
	 */
	[[nodiscard]] constexpr bool isUpperTriangular(
	    T epsilon = std::numeric_limits<T>::epsilon()) const noexcept
	  requires SquareMatrix<Mat>
	{
		for (std::size_t r = 1; r < Rows; ++r) {
			for (std::size_t c = 0; c < r; ++c) {
				if (std::abs(fields[r][c]) > epsilon) {
					return false;
				}
			}
		}
		return true;
	}

	/**
	 * @brief Returns `true` if all elements above the main diagonal are within `epsilon` of
	 * zero.
	 * @param [in] epsilon Tolerance (default: `numeric_limits<T>::epsilon()`).
	 * @return `true` iff the matrix is lower triangular (up to `epsilon`).
	 */
	[[nodiscard]] constexpr bool isLowerTriangular(
	    T epsilon = std::numeric_limits<T>::epsilon()) const noexcept
	  requires SquareMatrix<Mat>
	{
		for (std::size_t r = 0; r < Rows - 1; ++r) {
			for (std::size_t c = r + 1; c < Cols; ++c) {
				if (std::abs(fields[r][c]) > epsilon) {
					return false;
				}
			}
		}
		return true;
	}

	/**
	 * @brief Estimates the matrix rank via Gaussian elimination.
	 * @param [in] epsilon Pivot threshold
	 *             (default: `1000 * numeric_limits<T>::epsilon()`).
	 * @return The number of linearly independent rows (up to numerical precision).
	 */
	[[nodiscard]] constexpr std::size_t rank(T epsilon = std::numeric_limits<T>::epsilon() *
	                                                     T(1000)) const noexcept
	  requires std::floating_point<T>
	{
		auto        temp = *this;
		std::size_t rank = 0;

		for (std::size_t r = 0; r < std::min(Rows, Cols); ++r) {
			// Find pivot
			std::size_t pivot_row = r;
			for (std::size_t i = r + 1; i < Rows; ++i) {
				if (std::abs(temp[i][r]) > std::abs(temp[pivot_row][r])) {
					pivot_row = i;
				}
			}

			if (std::abs(temp[pivot_row][r]) <= epsilon) continue;

			if (pivot_row != r) {
				std::ranges::swap(temp[r], temp[pivot_row]);
			}

			rank++;

			// Eliminate below
			for (std::size_t i = r + 1; i < Rows; ++i) {
				if (std::abs(temp[i][r]) <= epsilon) continue;

				T factor = temp[i][r] / temp[r][r];
				for (std::size_t j = r; j < Cols; ++j) {
					temp[i][j] -= factor * temp[r][j];
				}
			}
		}

		return rank;
	}

	/**************************************************************************************
	|                                                                                     |
	|                              Unary arithmetic operator                              |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Unary identity operator.
	 * @return A copy of this matrix, unchanged.
	 */
	constexpr Mat operator+() const noexcept { return *this; }

	/**
	 * @brief Unary negation operator.
	 * @return A matrix with every element negated.
	 */
	constexpr Mat operator-() const noexcept
	{
		Mat result;
		std::ranges::transform(fields, result.fields.begin(), std::negate{});
		return result;
	}

	/**************************************************************************************
	|                                                                                     |
	|                            Compound assignment operator                             |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Adds `rhs` element-wise to this matrix.
	 * @param [in] rhs Right-hand side matrix.
	 * @return Reference to `*this`.
	 */
	constexpr Mat& operator+=(Mat const& rhs) noexcept
	{
		std::ranges::transform(fields, rhs.fields, fields.begin(), std::plus{});
		return *this;
	}

	/**
	 * @brief Subtracts `rhs` element-wise from this matrix.
	 * @param [in] rhs Right-hand side matrix.
	 * @return Reference to `*this`.
	 */
	constexpr Mat& operator-=(Mat const& rhs) noexcept
	{
		std::ranges::transform(fields, rhs.fields, fields.begin(), std::minus{});
		return *this;
	}

	/**
	 * @brief Multiplies this square matrix by `rhs` (matrix product, square matrices only).
	 * @param [in] rhs Right-hand side matrix.
	 * @return Reference to `*this`.
	 */
	constexpr Mat& operator*=(Mat const& rhs) noexcept
	  requires(Rows == Cols)
	{
		*this = *this * rhs;
		return *this;
	}

	/**************************************************************************************
	|                                                                                     |
	|                             Scalar compound assignment operator                     |
	|                                                                                     |
	**************************************************************************************/

	/**
	 * @brief Adds scalar `rhs` to every element.
	 * @param [in] rhs Scalar value.
	 * @return Reference to `*this`.
	 */
	constexpr Mat& operator+=(T rhs) noexcept
	{
		for (auto& row : fields) row += rhs;
		return *this;
	}

	/**
	 * @brief Subtracts scalar `rhs` from every element.
	 * @param [in] rhs Scalar value.
	 * @return Reference to `*this`.
	 */
	constexpr Mat& operator-=(T rhs) noexcept
	{
		for (auto& row : fields) row -= rhs;
		return *this;
	}

	/**
	 * @brief Multiplies every element by scalar `rhs`.
	 * @param [in] rhs Scalar value.
	 * @return Reference to `*this`.
	 */
	constexpr Mat& operator*=(T rhs) noexcept
	{
		for (auto& row : fields) row *= rhs;
		return *this;
	}

	/**
	 * @brief Divides every element by scalar `rhs`.
	 * @param [in] rhs Scalar value.
	 * @return Reference to `*this`.
	 */
	constexpr Mat& operator/=(T rhs) noexcept
	{
		for (auto& row : fields) row /= rhs;
		return *this;
	}
};

/**************************************************************************************
|                                                                                     |
|                                   Type Aliases                                      |
|                                                                                     |
**************************************************************************************/

// Primary aliases
template <class T = float>
using Mat2 = Mat<2, 2, T>;
template <class T = float>
using Mat3 = Mat<3, 3, T>;
template <class T = float>
using Mat4 = Mat<4, 4, T>;

// Common type instantiations
using Mat2f = Mat2<float>;
using Mat2d = Mat2<double>;
using Mat3f = Mat3<float>;
using Mat3d = Mat3<double>;
using Mat4f = Mat4<float>;
using Mat4d = Mat4<double>;

// Legacy compatibility - can be removed eventually
using Mat2x2f = Mat2f;
using Mat2x2d = Mat2d;
using Mat3x3f = Mat3f;
using Mat3x3d = Mat3d;
using Mat4x4f = Mat4f;
using Mat4x4d = Mat4d;

/**************************************************************************************
|                                                                                     |
|                                  Binary operators                                   |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Element-wise addition of two matrices.
 * @tparam Rows Number of rows.
 * @tparam Cols Number of columns.
 * @tparam T Element type.
 * @param [in] lhs Left-hand side matrix.
 * @param [in] rhs Right-hand side matrix.
 * @return A matrix whose element `[r][c]` is `lhs[r][c] + rhs[r][c]`.
 */
template <std::size_t Rows, std::size_t Cols, class T>
[[nodiscard]] constexpr Mat<Rows, Cols, T> operator+(
    Mat<Rows, Cols, T> lhs, Mat<Rows, Cols, T> const& rhs) noexcept
{
	lhs += rhs;
	return lhs;
}

/**
 * @brief Element-wise subtraction of two matrices.
 * @tparam Rows Number of rows.
 * @tparam Cols Number of columns.
 * @tparam T Element type.
 * @param [in] lhs Left-hand side matrix.
 * @param [in] rhs Right-hand side matrix.
 * @return A matrix whose element `[r][c]` is `lhs[r][c] - rhs[r][c]`.
 */
template <std::size_t Rows, std::size_t Cols, class T>
[[nodiscard]] constexpr Mat<Rows, Cols, T> operator-(
    Mat<Rows, Cols, T> lhs, Mat<Rows, Cols, T> const& rhs) noexcept
{
	lhs -= rhs;
	return lhs;
}

/**************************************************************************************
|                                                                                     |
|                              Scalar binary operators                                |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Adds scalar `rhs` to every element of `lhs`.
 * @tparam Rows Number of rows.
 * @tparam Cols Number of columns.
 * @tparam T Element type.
 * @param [in] lhs Left-hand side matrix.
 * @param [in] rhs Scalar value.
 * @return A matrix whose element `[r][c]` is `lhs[r][c] + rhs`.
 */
template <std::size_t Rows, std::size_t Cols, class T>
[[nodiscard]] constexpr Mat<Rows, Cols, T> operator+(Mat<Rows, Cols, T> lhs,
                                                     T                  rhs) noexcept
{
	lhs += rhs;
	return lhs;
}

/**
 * @brief Subtracts scalar `rhs` from every element of `lhs`.
 * @tparam Rows Number of rows.
 * @tparam Cols Number of columns.
 * @tparam T Element type.
 * @param [in] lhs Left-hand side matrix.
 * @param [in] rhs Scalar value.
 * @return A matrix whose element `[r][c]` is `lhs[r][c] - rhs`.
 */
template <std::size_t Rows, std::size_t Cols, class T>
[[nodiscard]] constexpr Mat<Rows, Cols, T> operator-(Mat<Rows, Cols, T> lhs,
                                                     T                  rhs) noexcept
{
	lhs -= rhs;
	return lhs;
}

/**
 * @brief Multiplies every element of `lhs` by scalar `rhs`.
 * @tparam Rows Number of rows.
 * @tparam Cols Number of columns.
 * @tparam T Element type.
 * @param [in] lhs Left-hand side matrix.
 * @param [in] rhs Scalar value.
 * @return A matrix whose element `[r][c]` is `lhs[r][c] * rhs`.
 */
template <std::size_t Rows, std::size_t Cols, class T>
[[nodiscard]] constexpr Mat<Rows, Cols, T> operator*(Mat<Rows, Cols, T> lhs,
                                                     T                  rhs) noexcept
{
	lhs *= rhs;
	return lhs;
}

/**
 * @brief Divides every element of `lhs` by scalar `rhs`.
 * @tparam Rows Number of rows.
 * @tparam Cols Number of columns.
 * @tparam T Element type.
 * @param [in] lhs Left-hand side matrix.
 * @param [in] rhs Scalar value.
 * @return A matrix whose element `[r][c]` is `lhs[r][c] / rhs`.
 */
template <std::size_t Rows, std::size_t Cols, class T>
[[nodiscard]] constexpr Mat<Rows, Cols, T> operator/(Mat<Rows, Cols, T> lhs,
                                                     T                  rhs) noexcept
{
	lhs /= rhs;
	return lhs;
}

/**
 * @brief Adds scalar `lhs` to every element of `rhs`.
 * @tparam Rows Number of rows.
 * @tparam Cols Number of columns.
 * @tparam T Element type.
 * @param [in] lhs Scalar value.
 * @param [in] rhs Right-hand side matrix.
 * @return A matrix whose element `[r][c]` is `lhs + rhs[r][c]`.
 */
template <std::size_t Rows, std::size_t Cols, class T>
[[nodiscard]] constexpr Mat<Rows, Cols, T> operator+(T                  lhs,
                                                     Mat<Rows, Cols, T> rhs) noexcept
{
	rhs += lhs;
	return rhs;
}

/**
 * @brief Subtracts every element of `rhs` from scalar `lhs`.
 * @tparam Rows Number of rows.
 * @tparam Cols Number of columns.
 * @tparam T Element type.
 * @param [in] lhs Scalar value.
 * @param [in] rhs Right-hand side matrix.
 * @return A matrix whose element `[r][c]` is `lhs - rhs[r][c]`.
 */
template <std::size_t Rows, std::size_t Cols, class T>
[[nodiscard]] constexpr Mat<Rows, Cols, T> operator-(
    T lhs, Mat<Rows, Cols, T> const& rhs) noexcept
{
	Mat<Rows, Cols, T> result;
	std::ranges::transform(rhs.fields, result.fields.begin(),
	                       [lhs](auto const& row) { return lhs - row; });
	return result;
}

/**
 * @brief Multiplies scalar `lhs` by every element of `rhs`.
 * @tparam Rows Number of rows.
 * @tparam Cols Number of columns.
 * @tparam T Element type.
 * @param [in] lhs Scalar value.
 * @param [in] rhs Right-hand side matrix.
 * @return A matrix whose element `[r][c]` is `lhs * rhs[r][c]`.
 */
template <std::size_t Rows, std::size_t Cols, class T>
[[nodiscard]] constexpr Mat<Rows, Cols, T> operator*(T                  lhs,
                                                     Mat<Rows, Cols, T> rhs) noexcept
{
	rhs *= lhs;
	return rhs;
}

/**
 * @brief Divides scalar `lhs` by every element of `rhs`.
 * @tparam Rows Number of rows.
 * @tparam Cols Number of columns.
 * @tparam T Element type.
 * @param [in] lhs Scalar value.
 * @param [in] rhs Right-hand side matrix.
 * @return A matrix whose element `[r][c]` is `lhs / rhs[r][c]`.
 */
template <std::size_t Rows, std::size_t Cols, class T>
[[nodiscard]] constexpr Mat<Rows, Cols, T> operator/(
    T lhs, Mat<Rows, Cols, T> const& rhs) noexcept
{
	Mat<Rows, Cols, T> result;
	std::ranges::transform(rhs.fields, result.fields.begin(),
	                       [lhs](auto const& row) { return lhs / row; });
	return result;
}

/**************************************************************************************
|                                                                                     |
|                             Matrix × vector product                                 |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Multiplies a matrix by a column vector (`M * v`).
 * @tparam Rows Number of rows in `m`.
 * @tparam Cols Number of columns in `m` (equals the dimension of `v`).
 * @tparam T Element type.
 * @param [in] m Left-hand side matrix.
 * @param [in] v Column vector.
 * @return A `Vec<Rows, T>` where element `r` is `dot(m.row(r), v)`.
 */
template <std::size_t Rows, std::size_t Cols, class T>
[[nodiscard]] constexpr Vec<Rows, T> operator*(Mat<Rows, Cols, T> const& m,
                                               Vec<Cols, T> const&       v) noexcept
{
	Vec<Rows, T> result{};
	for (std::size_t r = 0; r < Rows; ++r) {
		result[r] = dot(m.fields[r], v);
	}
	return result;
}

/**
 * @brief Multiplies a row vector by a matrix (`v * M`).
 * @tparam Rows Number of rows in `m` (equals the dimension of `v`).
 * @tparam Cols Number of columns in `m`.
 * @tparam T Element type.
 * @param [in] v Row vector.
 * @param [in] m Right-hand side matrix.
 * @return A `Vec<Cols, T>` where element `c` is `sum(v[r] * m[r][c])`.
 */
template <std::size_t Rows, std::size_t Cols, class T>
[[nodiscard]] constexpr Vec<Cols, T> operator*(Vec<Rows, T> const&       v,
                                               Mat<Rows, Cols, T> const& m) noexcept
{
	Vec<Cols, T> result{};
	for (std::size_t c = 0; c < Cols; ++c) {
		for (std::size_t r = 0; r < Rows; ++r) {
			result[c] += v[r] * m.fields[r][c];
		}
	}
	return result;
}

/**************************************************************************************
|                                                                                     |
|                             Matrix × matrix product                                 |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Matrix × matrix product.
 * @details Computes the standard matrix product `a * b` via the triple-loop formula.
 *          The number of columns in `a` must equal the number of rows in `b` (`Shared`).
 * @tparam Rows   Number of rows in `a` (and in the result).
 * @tparam Shared Inner dimension: columns of `a` and rows of `b`.
 * @tparam Cols   Number of columns in `b` (and in the result).
 * @tparam T      Element type.
 * @param [in] a Left-hand side matrix of shape `Rows × Shared`.
 * @param [in] b Right-hand side matrix of shape `Shared × Cols`.
 * @return A `Mat<Rows, Cols, T>` where `result[r][c] = sum_k a[r][k] * b[k][c]`.
 */
// Mat<Rows, Shared, T> * Mat<Shared, Cols, T> -> Mat<Rows, Cols, T>
template <std::size_t Rows, std::size_t Shared, std::size_t Cols, class T>
[[nodiscard]] constexpr Mat<Rows, Cols, T> operator*(
    Mat<Rows, Shared, T> const& a, Mat<Shared, Cols, T> const& b) noexcept
{
	Mat<Rows, Cols, T> result{};
	for (std::size_t r = 0; r < Rows; ++r) {
		for (std::size_t c = 0; c < Cols; ++c) {
			for (std::size_t k = 0; k < Shared; ++k) {
				result.fields[r][c] += a.fields[r][k] * b.fields[k][c];
			}
		}
	}
	return result;
}

/**************************************************************************************
|                                                                                     |
|                                      Functions                                      |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Computes the determinant of a square matrix.
 * @details Closed-form implementations are provided for 2×2, 3×3, and 4×4 matrices.
 *          Requesting any other size triggers a `static_assert` at compile time.
 * @tparam M A square matrix type satisfying `SquareMatrix`.
 * @param [in] m The matrix.
 * @return The determinant of `m` as `M::value_type`.
 */
template <SquareMatrix M>
[[nodiscard]] constexpr auto determinant(M const& m) noexcept
{
	using T          = typename M::value_type;
	constexpr auto N = M::rows();

	if constexpr (2 == N) {
		return m[0][0] * m[1][1] - m[0][1] * m[1][0];
	} else if constexpr (3 == N) {
		return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
		       m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
		       m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
	} else if constexpr (4 == N) {
		// Use row-major indexing: m[row][col]
		T const A2323 = m[2][2] * m[3][3] - m[2][3] * m[3][2];
		T const A1323 = m[2][1] * m[3][3] - m[2][3] * m[3][1];
		T const A1223 = m[2][1] * m[3][2] - m[2][2] * m[3][1];
		T const A0323 = m[2][0] * m[3][3] - m[2][3] * m[3][0];
		T const A0223 = m[2][0] * m[3][2] - m[2][2] * m[3][0];
		T const A0123 = m[2][0] * m[3][1] - m[2][1] * m[3][0];
		return m[0][0] * (m[1][1] * A2323 - m[1][2] * A1323 + m[1][3] * A1223) -
		       m[0][1] * (m[1][0] * A2323 - m[1][2] * A0323 + m[1][3] * A0223) +
		       m[0][2] * (m[1][0] * A1323 - m[1][1] * A0323 + m[1][3] * A0123) -
		       m[0][3] * (m[1][0] * A1223 - m[1][1] * A0223 + m[1][2] * A0123);
	} else {
		[]<bool flag = false>() {
			static_assert(flag, "determinant only implemented for 2x2, 3x3, and 4x4 matrices");
		}();
	}
}

/**
 * @brief Computes the inverse of a square floating-point matrix.
 * @details Closed-form implementations are provided for 2×2, 3×3, and 4×4 matrices.
 *          Division by the determinant is performed without a prior singularity check;
 *          use `Mat::safe_inverse()` when the matrix may be singular.
 *          Requesting any other size triggers a `static_assert` at compile time.
 * @tparam M A square floating-point matrix satisfying `InvertibleMatrix`.
 * @param [in] m The matrix to invert.
 * @return The inverse of `m`.
 */
template <InvertibleMatrix M>
[[nodiscard]] constexpr M inverse(M const& m) noexcept
{
	using T          = typename M::value_type;
	constexpr auto N = M::rows();

	if constexpr (2 == N) {
		T const det_inv = T(1) / determinant(m);
		M       result{};
		result[0][0] = m[1][1] * det_inv;
		result[0][1] = -m[0][1] * det_inv;
		result[1][0] = -m[1][0] * det_inv;
		result[1][1] = m[0][0] * det_inv;
		return result;
	} else if constexpr (3 == N) {
		T const det_inv = T(1) / determinant(m);
		M       result{};
		result[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * det_inv;
		result[0][1] = -(m[0][1] * m[2][2] - m[0][2] * m[2][1]) * det_inv;
		result[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * det_inv;
		result[1][0] = -(m[1][0] * m[2][2] - m[1][2] * m[2][0]) * det_inv;
		result[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * det_inv;
		result[1][2] = -(m[0][0] * m[1][2] - m[0][2] * m[1][0]) * det_inv;
		result[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * det_inv;
		result[2][1] = -(m[0][0] * m[2][1] - m[0][1] * m[2][0]) * det_inv;
		result[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * det_inv;
		return result;
	} else if constexpr (4 == N) {
		T const A2323 = m[2][2] * m[3][3] - m[2][3] * m[3][2];
		T const A1323 = m[2][1] * m[3][3] - m[2][3] * m[3][1];
		T const A1223 = m[2][1] * m[3][2] - m[2][2] * m[3][1];
		T const A0323 = m[2][0] * m[3][3] - m[2][3] * m[3][0];
		T const A0223 = m[2][0] * m[3][2] - m[2][2] * m[3][0];
		T const A0123 = m[2][0] * m[3][1] - m[2][1] * m[3][0];
		T const A2313 = m[1][2] * m[3][3] - m[1][3] * m[3][2];
		T const A1313 = m[1][1] * m[3][3] - m[1][3] * m[3][1];
		T const A1213 = m[1][1] * m[3][2] - m[1][2] * m[3][1];
		T const A2312 = m[1][2] * m[2][3] - m[1][3] * m[2][2];
		T const A1312 = m[1][1] * m[2][3] - m[1][3] * m[2][1];
		T const A1212 = m[1][1] * m[2][2] - m[1][2] * m[2][1];
		T const A0313 = m[1][0] * m[3][3] - m[1][3] * m[3][0];
		T const A0213 = m[1][0] * m[3][2] - m[1][2] * m[3][0];
		T const A0312 = m[1][0] * m[2][3] - m[1][3] * m[2][0];
		T const A0212 = m[1][0] * m[2][2] - m[1][2] * m[2][0];
		T const A0113 = m[1][0] * m[3][1] - m[1][1] * m[3][0];
		T const A0112 = m[1][0] * m[2][1] - m[1][1] * m[2][0];

		T const det_inv =
		    T(1) / (m[0][0] * (m[1][1] * A2323 - m[1][2] * A1323 + m[1][3] * A1223) -
		            m[0][1] * (m[1][0] * A2323 - m[1][2] * A0323 + m[1][3] * A0223) +
		            m[0][2] * (m[1][0] * A1323 - m[1][1] * A0323 + m[1][3] * A0123) -
		            m[0][3] * (m[1][0] * A1223 - m[1][1] * A0223 + m[1][2] * A0123));

		M result{};
		result[0][0] = (m[1][1] * A2323 - m[1][2] * A1323 + m[1][3] * A1223) * det_inv;
		result[0][1] = -(m[0][1] * A2323 - m[0][2] * A1323 + m[0][3] * A1223) * det_inv;
		result[0][2] = (m[0][1] * A2313 - m[0][2] * A1313 + m[0][3] * A1213) * det_inv;
		result[0][3] = -(m[0][1] * A2312 - m[0][2] * A1312 + m[0][3] * A1212) * det_inv;
		result[1][0] = -(m[1][0] * A2323 - m[1][2] * A0323 + m[1][3] * A0223) * det_inv;
		result[1][1] = (m[0][0] * A2323 - m[0][2] * A0323 + m[0][3] * A0223) * det_inv;
		result[1][2] = -(m[0][0] * A2313 - m[0][2] * A0313 + m[0][3] * A0213) * det_inv;
		result[1][3] = (m[0][0] * A2312 - m[0][2] * A0312 + m[0][3] * A0212) * det_inv;
		result[2][0] = (m[1][0] * A1323 - m[1][1] * A0323 + m[1][3] * A0123) * det_inv;
		result[2][1] = -(m[0][0] * A1323 - m[0][1] * A0323 + m[0][3] * A0123) * det_inv;
		result[2][2] = (m[0][0] * A1313 - m[0][1] * A0313 + m[0][3] * A0113) * det_inv;
		result[2][3] = -(m[0][0] * A1312 - m[0][1] * A0312 + m[0][3] * A0112) * det_inv;
		result[3][0] = -(m[1][0] * A1223 - m[1][1] * A0223 + m[1][2] * A0123) * det_inv;
		result[3][1] = (m[0][0] * A1223 - m[0][1] * A0223 + m[0][2] * A0123) * det_inv;
		result[3][2] = -(m[0][0] * A1213 - m[0][1] * A0213 + m[0][2] * A0113) * det_inv;
		result[3][3] = (m[0][0] * A1212 - m[0][1] * A0212 + m[0][2] * A0112) * det_inv;
		return result;
	} else {
		[]<bool flag = false>() {
			static_assert(flag, "inverse only implemented for 2x2, 3x3, and 4x4 matrices");
		}();
	}
}

/**
 * @brief Returns the transpose of a matrix.
 * @details Produces a new `Mat<Cols, Rows, T>` where `result[c][r] = m[r][c]`.
 * @tparam Rows Number of rows in `m` (becomes the column count of the result).
 * @tparam Cols Number of columns in `m` (becomes the row count of the result).
 * @tparam T    Element type.
 * @param [in] m The matrix to transpose.
 * @return The transposed matrix of shape `Cols × Rows`.
 */
template <std::size_t Rows, std::size_t Cols, class T>
[[nodiscard]] constexpr Mat<Cols, Rows, T> transpose(Mat<Rows, Cols, T> const& m) noexcept
{
	Mat<Cols, Rows, T> result{};
	for (std::size_t r = 0; r < Rows; ++r) {
		for (std::size_t c = 0; c < Cols; ++c) {
			result.fields[c][r] = m.fields[r][c];
		}
	}
	return result;
}

/**
 * @brief Returns an `R × C` matrix with all elements set to zero.
 * @tparam R Number of rows.
 * @tparam C Number of columns.
 * @tparam T Element type (default: `float`).
 * @return A zero matrix of shape `R × C`.
 */
// Convenient factory functions
template <std::size_t R, std::size_t C, class T = float>
[[nodiscard]] constexpr Mat<R, C, T> zeros() noexcept
{
	return Mat<R, C, T>::zeros();
}

/**
 * @brief Returns an `N × N` identity matrix.
 * @tparam N Matrix dimension.
 * @tparam T Element type (default: `float`).
 * @return An identity matrix of shape `N × N`.
 */
template <std::size_t N, class T = float>
[[nodiscard]] constexpr Mat<N, N, T> identity() noexcept
{
	return Mat<N, N, T>::identity();
}

/**
 * @brief Returns an `R × C` matrix with all elements set to one.
 * @tparam R Number of rows.
 * @tparam C Number of columns.
 * @tparam T Element type (default: `float`).
 * @return An all-ones matrix of shape `R × C`.
 */
template <std::size_t R, std::size_t C, class T = float>
[[nodiscard]] constexpr Mat<R, C, T> ones() noexcept
{
	return Mat<R, C, T>::ones();
}

/**************************************************************************************
|                                                                                     |
|                           Symmetric 3×3 eigen decomposition                        |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Computes the eigenvalues of a real symmetric 3×3 matrix, sorted ascending.
 *
 * @details Uses the closed-form trigonometric solution (Cardano's method adapted for
 *          symmetric 3×3 matrices). Only the upper triangle of `m` is read; the
 *          lower triangle is assumed to equal the upper triangle.
 *
 * @tparam T Floating-point scalar type.
 * @param [in] m A real symmetric 3×3 matrix.
 * @return `Vec<3, T>` with eigenvalues `e[0] ≤ e[1] ≤ e[2]`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr Vec<3, T> eigenValues(Mat<3, 3, T> const& m) noexcept
{
	T const a = m[0][0], d = m[0][1], f = m[0][2];
	T const b = m[1][1], e = m[1][2];
	T const c = m[2][2];

	T const x_1 =
	    a * a + b * b + c * c - a * b - a * c - b * c + T(3) * (d * d + f * f + e * e);

	T const x_2 = -(T(2) * a - b - c) * (T(2) * b - a - c) * (T(2) * c - a - b) +
	              T(9) * ((T(2) * c - a - b) * (d * d) + (T(2) * b - a - c) * (f * f) +
	                      (T(2) * a - b - c) * (e * e)) -
	              T(54) * (d * e * f);

	T const phi =
	    T(0) < x_2
	        ? std::atan(std::sqrt(T(4) * x_1 * x_1 * x_1 - x_2 * x_2) / x_2)
	        : (T(0) > x_2 ? std::atan(std::sqrt(T(4) * x_1 * x_1 * x_1 - x_2 * x_2) / x_2) +
	                            std::numbers::pi_v<T>
	                      : std::numbers::pi_v<T> / T(2));

	return Vec<3, T>(
	    (a + b + c - T(2) * std::sqrt(x_1) * std::cos(phi / T(3))) / T(3),
	    (a + b + c +
	     T(2) * std::sqrt(x_1) * std::cos((phi + std::numbers::pi_v<T>) / T(3))) /
	        T(3),
	    (a + b + c +
	     T(2) * std::sqrt(x_1) * std::cos((phi - std::numbers::pi_v<T>) / T(3))) /
	        T(3));
}

/**
 * @brief Computes the eigenvectors of a real symmetric 3×3 matrix.
 *
 * @details Each returned vector is normalized. The ordering matches `eigen_values`,
 *          so `result[i]` is the eigenvector for `eigen_values[i]`.
 *
 * @note The computation is undefined if the covariance is degenerate (e.g. all points
 *       collinear), which may produce a zero denominator.
 *
 * @tparam T Floating-point scalar type.
 * @param [in] m            A real symmetric 3×3 matrix (upper triangle is read).
 * @return Array of three unit eigenvectors ordered to match `eigen_values`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr std::array<Vec<3, T>, 3> eigenVectors(
    Mat<3, 3, T> const& m) noexcept
{
	return eigenVectors(m, eigenValues(m));
}

/**
 * @brief Computes the eigenvectors of a real symmetric 3×3 matrix.
 *
 * @details Each returned vector is normalized. The ordering matches `eigen_values`,
 *          so `result[i]` is the eigenvector for `eigen_values[i]`.
 *
 * @note The computation is undefined if the covariance is degenerate (e.g. all points
 *       collinear), which may produce a zero denominator.
 *
 * @tparam T Floating-point scalar type.
 * @param [in] m            A real symmetric 3×3 matrix (upper triangle is read).
 * @param [in] eigen_values Pre-computed eigenvalues, e.g. from `eigenValues(m)`.
 * @return Array of three unit eigenvectors ordered to match `eigen_values`.
 */
template <std::floating_point T>
[[nodiscard]] constexpr std::array<Vec<3, T>, 3> eigenVectors(
    Mat<3, 3, T> const& m, Vec<3, T> const& eigen_values) noexcept
{
	assert(eigenValues(m) == eigen_values);

	T const d = m[0][1];
	T const f = m[0][2] == T(0) ? std::numeric_limits<T>::epsilon() : m[0][2];
	T const b = m[1][1], e = m[1][2];
	T const c = m[2][2];

	T const l_1 = eigen_values[0];
	T const l_2 = eigen_values[1];
	T const l_3 = eigen_values[2];

	T const m_1 = (d * (c - l_1) - e * f) / (f * (b - l_1) - d * e);
	T const m_2 = (d * (c - l_2) - e * f) / (f * (b - l_2) - d * e);
	T const m_3 = (d * (c - l_3) - e * f) / (f * (b - l_3) - d * e);

	return {normalize(Vec<3, T>((l_1 - c - e * m_1) / f, m_1, T(1))),
	        normalize(Vec<3, T>((l_2 - c - e * m_2) / f, m_2, T(1))),
	        normalize(Vec<3, T>((l_3 - c - e * m_3) / f, m_3, T(1)))};
}

/**************************************************************************************
|                                                                                     |
|                              Projection / view matrices                             |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Builds a right-handed 2-D orthographic projection matrix (no depth clipping).
 * @details Maps the rectangle `[left, right] × [bottom, top]` to the NDC square
 *          `[-1, 1]²`. The resulting matrix has `m[2][2] = -1` and `m[3][3] = 1`.
 * @tparam T Floating-point element type.
 * @param [in] left   Left clipping plane.
 * @param [in] right  Right clipping plane.
 * @param [in] bottom Bottom clipping plane.
 * @param [in] top    Top clipping plane.
 * @return A `Mat<4, 4, T>` representing the orthographic projection.
 */
template <std::floating_point T>
[[nodiscard]] Mat<4, 4, T> orthogonal(T left, T right, T bottom, T top)
{
	Mat<4, 4, T> m{};
	m[0][0] = T(2) / (right - left);
	m[1][1] = T(2) / (top - bottom);
	m[2][2] = -T(1);
	m[0][3] = -(right + left) / (right - left);
	m[1][3] = -(top + bottom) / (top - bottom);
	m[3][3] = T(1);
	return m;
}

/**
 * @brief Builds an orthographic projection matrix (OpenCV convention).
 * @details Constructs a right-handed orthographic matrix (+X Right, +Y Down, +Z Forward)
 *          with a `[0, 1]` depth range.
 * @tparam T           Floating-point element type.
 * @param [in] left   Left clipping plane.
 * @param [in] right  Right clipping plane.
 * @param [in] bottom Bottom clipping plane.
 * @param [in] top    Top clipping plane.
 * @param [in] zNear  Near clipping plane.
 * @param [in] zFar   Far clipping plane.
 * @return A `Mat<4, 4, T>` representing the orthographic projection.
 */
template <std::floating_point T>
[[nodiscard]] Mat<4, 4, T> orthogonal(T left, T right, T bottom, T top, T zNear, T zFar)
{
	Mat<4, 4, T> m{};
	m[3][3] = T(1);

	m[0][0] = T(2) / (right - left);
	m[1][1] = T(2) / (top - bottom);
	m[2][2] = T(1) / (zFar - zNear);
	m[0][3] = -(right + left) / (right - left);
	m[1][3] = -(top + bottom) / (top - bottom);
	m[2][3] = -zNear / (zFar - zNear);

	return m;
}

/**
 * @brief Builds a perspective projection matrix (OpenCV convention).
 * @details Constructs a right-handed perspective matrix (+X Right, +Y Down, +Z Forward)
 *          with a `[0, 1]` depth range.
 * @tparam T           Floating-point element type.
 * @param [in] fovy   Vertical field of view in radians.
 * @param [in] aspect Aspect ratio (viewport width / viewport height).
 * @param [in] near   Near clipping distance (must be positive).
 * @param [in] far    Far clipping distance (must be greater than `near`).
 * @return A `Mat<4, 4, T>` representing the perspective projection.
 */
template <std::floating_point T>
[[nodiscard]] Mat<4, 4, T> perspective(T fovy, T aspect, T near, T far)
{
	Mat<4, 4, T> m{};

	T const tan_half_fovy = std::tan(fovy / T(2));

	m[0][0] = T(1) / (aspect * tan_half_fovy);
	m[1][1] = T(1) / tan_half_fovy;
	m[2][2] = far / (far - near);
	m[3][2] = T(1);
	m[2][3] = -(far * near) / (far - near);

	return m;
}

/**
 * @brief Builds a perspective projection matrix (OpenCV convention).
 * @details Constructs a right-handed perspective matrix (+X Right, +Y Down, +Z Forward)
 *          from camera intrinsics (focal lengths, principal point, and resolution)
 *          with a `[0, 1]` depth range.
 * @tparam T           Floating-point element type.
 * @param [in] fx     Horizontal focal length.
 * @param [in] fy     Vertical focal length.
 * @param [in] cx     Horizontal principal point (offset from center).
 * @param [in] cy     Vertical principal point (offset from center).
 * @param [in] width  Viewport width.
 * @param [in] height Viewport height.
 * @param [in] near   Near clipping distance.
 * @param [in] far    Far clipping distance.
 * @return A `Mat<4, 4, T>` representing the perspective projection.
 */
template <std::floating_point T>
[[nodiscard]] Mat<4, 4, T> perspective(T fx, T fy, T cx, T cy, T width, T height, T near,
                                       T far)
{
	Mat<4, 4, T> m{};

	m[0][0] = T(2) * fx / width;
	m[1][1] = T(2) * fy / height;
	m[0][2] = (T(2) * cx / width) - T(1);
	m[1][2] = (T(2) * cy / height) - T(1);
	m[2][2] = far / (far - near);
	m[3][2] = T(1);
	m[2][3] = -(far * near) / (far - near);

	return m;
}

/**
 * @brief Builds a view matrix (OpenCV convention).
 * @details Computes a right-handed look-at view transform (+X Right, +Y Down, +Z
 * Forward): places the camera at `eye` and looks toward `target`.
 * @tparam T           Floating-point element type.
 * @param [in] eye    Camera position in world space.
 * @param [in] target Point in world space the camera looks at.
 * @param [in] up     World-up direction (need not be orthogonal to the view direction).
 * @return A `Mat<4, 4, T>` view matrix.
 */
template <std::floating_point T>
[[nodiscard]] Mat<4, 4, T> lookAt(Vec<3, T> const& eye, Vec<3, T> const& target,
                                  Vec<3, T> const& up)
{
	Mat<4, 4, T> m{};

	Vec<3, T> const f(normalize(target - eye));
	Vec<3, T> const s(normalize(cross(f, up)));
	Vec<3, T> const u(cross(s, f));

	m[0][0] = s[0];
	m[0][1] = s[1];
	m[0][2] = s[2];
	m[1][0] = -u[0];
	m[1][1] = -u[1];
	m[1][2] = -u[2];
	m[2][0] = f[0];
	m[2][1] = f[1];
	m[2][2] = f[2];
	m[0][3] = -dot(s, eye);
	m[1][3] = dot(u, eye);
	m[2][3] = -dot(f, eye);
	m[3][3] = T(1);

	return m;
}

/**
 * @brief Builds a right-handed perspective projection matrix with an infinite far plane.
 * @details Useful for rendering techniques that require an unbounded depth range (e.g.,
 *          shadow volumes, skyboxes). The far plane is pushed to infinity so depth
 *          buffer precision is concentrated near the camera.
 * @tparam T           Floating-point element type.
 * @param [in] fovy   Vertical field of view in radians.
 * @param [in] aspect Aspect ratio (viewport width / viewport height).
 * @param [in] near   Near clipping distance (must be positive).
 * @return A `Mat<4, 4, T>` representing the infinite perspective projection.
 */
template <std::floating_point T>
[[nodiscard]] Mat<4, 4, T> infinitePerspective(T fovy, T aspect, T near)
{
	Mat<4, 4, T> m{};
	T const      tan_half_fovy = std::tan(fovy / T(2));

	m[0][0] = T(1) / (aspect * tan_half_fovy);
	m[1][1] = T(1) / tan_half_fovy;

	m[2][2] = T(1);
	m[3][2] = T(1);
	m[2][3] = -near;

	return m;
}

/**
 * @brief Applies an axis-angle rotation to a 4×4 matrix.
 * @details Computes `m * R` where `R` is the 3×3 rotation matrix for the given
 *          axis and angle. The last row of `m` is preserved unchanged.
 * @tparam T Floating-point element type.
 * @param [in] m     The base 4×4 transformation matrix.
 * @param [in] angle Rotation angle in radians.
 * @param [in] v     Rotation axis (need not be normalised).
 * @return A new `Mat<4, 4, T>` with the rotation applied to the upper-left 3×3 block.
 */
template <std::floating_point T>
[[nodiscard]] Mat<4, 4, T> rotate(Mat<4, 4, T> const& m, T angle, Vec<3, T> const& v)
{
	T const c = std::cos(angle);
	T const s = std::sin(angle);

	Vec<3, T> const axis(normalize(v));
	Vec<3, T> const temp((T(1) - c) * axis);

	// Rotation matrix R in row-major: rot[row][col] = R_{row,col}
	Mat<3, 3, T> rot{};
	rot[0][0] = c + temp[0] * axis[0];
	rot[0][1] = temp[1] * axis[0] - s * axis[2];
	rot[0][2] = temp[2] * axis[0] + s * axis[1];
	rot[1][0] = temp[0] * axis[1] + s * axis[2];
	rot[1][1] = c + temp[1] * axis[1];
	rot[1][2] = temp[2] * axis[1] - s * axis[0];
	rot[2][0] = temp[0] * axis[2] - s * axis[1];
	rot[2][1] = temp[1] * axis[2] + s * axis[0];
	rot[2][2] = c + temp[2] * axis[2];

	// Compute M * R for upper-left 3x3, preserving last row and column
	Mat<4, 4, T> result{};
	for (std::size_t i = 0; i < 3; ++i) {
		for (std::size_t j = 0; j < 3; ++j) {
			for (std::size_t k = 0; k < 3; ++k) {
				result[i][j] += m[i][k] * rot[k][j];
			}
		}
		result[i][3] = m[i][3];
	}
	result[3] = m[3];
	return result;
}

/**************************************************************************************
|                                                                                     |
|                                 Structured bindings                                 |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Structured-binding accessor — lvalue reference to row `I`.
 * @tparam I    Row index (must satisfy `I < Rows`).
 * @tparam Rows Number of rows in the matrix.
 * @tparam Cols Number of columns in the matrix.
 * @tparam T    Element type.
 * @param [in] m The matrix.
 * @return A mutable reference to row `I` as `Vec<Cols, T>&`.
 */
template <std::size_t I, std::size_t Rows, std::size_t Cols, class T>
  requires(I < Rows)
[[nodiscard]] constexpr Vec<Cols, T>& get(Mat<Rows, Cols, T>& m) noexcept
{
	return m.fields[I];
}

/**
 * @brief Structured-binding accessor — const lvalue reference to row `I`.
 * @tparam I    Row index (must satisfy `I < Rows`).
 * @tparam Rows Number of rows in the matrix.
 * @tparam Cols Number of columns in the matrix.
 * @tparam T    Element type.
 * @param [in] m The matrix.
 * @return A const reference to row `I` as `Vec<Cols, T> const&`.
 */
template <std::size_t I, std::size_t Rows, std::size_t Cols, class T>
  requires(I < Rows)
[[nodiscard]] constexpr Vec<Cols, T> const& get(Mat<Rows, Cols, T> const& m) noexcept
{
	return m.fields[I];
}

/**
 * @brief Structured-binding accessor — rvalue reference to row `I`.
 * @tparam I    Row index (must satisfy `I < Rows`).
 * @tparam Rows Number of rows in the matrix.
 * @tparam Cols Number of columns in the matrix.
 * @tparam T    Element type.
 * @param [in] m The matrix (rvalue).
 * @return Row `I` as `Vec<Cols, T>&&`.
 */
template <std::size_t I, std::size_t Rows, std::size_t Cols, class T>
  requires(I < Rows)
[[nodiscard]] constexpr Vec<Cols, T>&& get(Mat<Rows, Cols, T>&& m) noexcept
{
	return std::move(m.fields[I]);
}

/**************************************************************************************
|                                                                                     |
|                                       Print                                         |
|                                                                                     |
**************************************************************************************/

/**
 * @brief Writes a matrix to an output stream, one row per line.
 * @details Each row is formatted using `Vec`'s `operator<<`, separated by newlines.
 * @tparam Rows Number of rows.
 * @tparam Cols Number of columns.
 * @tparam T    Element type.
 * @param [in] out Output stream.
 * @param [in] m   Matrix to print.
 * @return Reference to `out`.
 */
template <std::size_t Rows, std::size_t Cols, class T>
std::ostream& operator<<(std::ostream& out, Mat<Rows, Cols, T> const& m)
{
	for (std::size_t r = 0; r < Rows; ++r) {
		if (r > 0) out << '\n';
		out << m.fields[r];
	}
	return out;
}

}  // namespace ufo

/**
 * @}
 */

template <std::size_t Rows, std::size_t Cols, class T>
struct std::tuple_size<ufo::Mat<Rows, Cols, T>>
    : std::integral_constant<std::size_t, Rows> {
};

template <std::size_t I, std::size_t Rows, std::size_t Cols, class T>
struct std::tuple_element<I, ufo::Mat<Rows, Cols, T>> {
	using type = ufo::Vec<Cols, T>;
};

template <std::size_t Rows, std::size_t Cols, std::formattable<char> T>
struct std::formatter<ufo::Mat<Rows, Cols, T>> {
	constexpr auto parse(std::format_parse_context& ctx) { return ctx.begin(); }

	auto format(ufo::Mat<Rows, Cols, T> const& m, std::format_context& ctx) const
	{
		auto out = ctx.out();
		for (std::size_t r = 0; r < Rows; ++r) {
			if (r > 0) out = std::format_to(out, "\n");
			out = std::format_to(out, "{}", m.fields[r]);
		}
		return out;
	}
};

#endif  // UFO_NUMERIC_MAT_HPP