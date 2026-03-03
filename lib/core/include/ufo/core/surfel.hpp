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

#ifndef UFO_CORE_SURFEL_HPP
#define UFO_CORE_SURFEL_HPP

// UFO
#include <ufo/math/mat.hpp>
#include <ufo/math/vec.hpp>

// STL
#include <algorithm>
#include <array>
#include <concepts>
#include <cstdint>
#include <format>
#include <ostream>
#include <ranges>

namespace ufo
{
/**
 * @ingroup core
 * @{
 */

/**
 * @brief Represents an incremental surface element (surfel) that tracks the sufficient
 * statistics of a 3D point set for online surface estimation.
 *
 * @details
 * A `Surfel` maintains three quantities that together allow the mean, full 3×3 covariance
 * matrix, eigenvalues, eigenvectors, surface normal, and planarity of an arbitrary set of
 * 3D points to be computed at any time, without storing the individual points:
 * - `sum_`         — the sum of all point positions (a `Vec<3, float>`).
 * - `sum_squares_` — the upper triangle of the **scatter matrix** S, stored as six
 * `float`s in the order (Sxx, Sxy, Sxz, Syy, Syz, Szz). S accumulates the sum of squared
 * deviations from the running mean; the sample covariance is S / (n − 1).
 * - `num_points_`  — the number of contributing points.
 *
 * Points and entire surfels can be added or removed incrementally. The addition algorithm
 * uses a numerically stable parallel / online update rule (analogous to Welford's
 * algorithm for variance) so that the scatter matrix remains accurate as the point count
 * grows. Batch addition uses a two-pass algorithm for improved numerical conditioning.
 * All intermediate arithmetic is performed in double precision; results are stored as
 * float.
 *
 * Intended for use as a per-point or per-node attribute in surface reconstruction,
 * mapping, or point cloud processing.
 *
 * **Derived quantities**
 * | Method              | Description                                                  |
 * |---------------------|--------------------------------------------------------------|
 * | `mean()`            | Centroid of the point set.                                   |
 * | `covariance()`      | 3×3 sample covariance matrix.                                |
 * | `normal()`          | Unit surface normal = eigenvector of the smallest eigenvalue.|
 * | `planarity()`       | Planarity measure in [0, 1]; 1 means perfectly planar.       |
 */
class Surfel
{
 public:
	//
	// Constructors
	//

	/**
	 * @brief Default-constructs an empty surfel with no points.
	 */
	constexpr Surfel() = default;

	/**
	 * @brief Constructs a surfel directly from its raw sufficient statistics.
	 *
	 * @param [in] sum         Sum of all point positions.
	 * @param [in] sum_squares Upper triangle of the scatter matrix (Sxx, Sxy, Sxz, Syy,
	 * Syz, Szz).
	 * @param [in] num_points  Number of points represented by this surfel.
	 *
	 * @details
	 * Intended for deserialization or copying internal state.
	 */
	constexpr Surfel(Vec<3, float> sum, std::array<float, 6> sum_squares,
	                 std::uint32_t num_points)
	    : sum_squares_(sum_squares), sum_(sum), num_points_(num_points)
	{
	}

	/**
	 * @brief Constructs a surfel from a single position and optional scatter matrix.
	 * @param [in] position The point position.
	 * @param [in] scatter Optional scatter matrix.
	 */
	constexpr Surfel(Vec<3, float> position, Mat<3, 3, float> scatter = {})
	    : sum_squares_{scatter[0][0], scatter[0][1], scatter[0][2],
	                   scatter[1][1], scatter[1][2], scatter[2][2]}
	    , sum_(position)
	    , num_points_(1)
	{
	}

	/**
	 * @brief Constructs a surfel by adding all points in `[first, last)`.
	 *
	 * @tparam InputIt `std::input_iterator` whose value type is convertible to `Vec3d`.
	 * @param [in] first The beginning of the input range.
	 * @param [in] last The end of the input range.
	 */
	template <std::input_iterator InputIt>
	  requires std::constructible_from<Vec<3, float>, std::iter_value_t<InputIt>>
	constexpr Surfel(InputIt first, InputIt last)
	{
		add(first, last);
	}

	/**
	 * @brief Constructs a surfel by adding all points in `points`.
	 *
	 * @tparam Points `std::ranges::input_range` whose elements are convertible to `Vec3d`.
	 * @param [in] points The range of points to add.
	 */
	template <std::ranges::input_range Points>
	  requires std::constructible_from<Vec<3, float>, std::ranges::range_value_t<Points>>
	constexpr Surfel(Points const& points) : Surfel(std::cbegin(points), std::cend(points))
	{
	}

	/**
	 * @brief Constructs a surfel from a brace-enclosed list of `Vec3f` points.
	 * @param [in] points The list of points to add.
	 */
	constexpr Surfel(std::initializer_list<Vec<3, float>> points)
	    : Surfel(begin(points), end(points))
	{
	}

	/**
	 * @brief Equality comparison on all three stored statistics.
	 * @return True if all statistics are equal, false otherwise.
	 */
	[[nodiscard]] constexpr bool operator==(Surfel const&) const noexcept = default;

	//
	// Empty
	//

	/**
	 * @brief Returns true if no points have been added.
	 * @return True if empty.
	 */
	[[nodiscard]] constexpr bool empty() const noexcept { return 0 == num_points_; }

	//
	// Add
	//

	/**
	 * @brief Merges another surfel into this one using a numerically stable
	 * parallel update rule.
	 *
	 * @param [in] rhs The surfel to merge in.
	 * @return Reference to `*this` after merging.
	 */
	constexpr Surfel& operator+=(Surfel const& rhs) noexcept
	{
		add(rhs);
		return *this;
	}

	/**
	 * @brief Adds a single 3D point to this surfel using an online (Welford-like) update
	 * rule.
	 *
	 * @param [in] rhs The point to add.
	 * @return Reference to `*this` after adding the point.
	 */
	constexpr Surfel& operator+=(Vec<3, float> rhs) noexcept
	{
		add(rhs);
		return *this;
	}

	/**
	 * @brief Returns a new surfel that is the merge of `lhs` and `rhs`.
	 *
	 * @param [in] lhs The left-hand side surfel.
	 * @param [in] rhs The right-hand side surfel.
	 * @return Surfel The merged surfel.
	 */
	[[nodiscard]] friend Surfel operator+(Surfel lhs, Surfel const& rhs) noexcept
	{
		return lhs += rhs;
	}

	/**
	 * @brief Returns a new surfel that is the result of adding point `rhs` to `lhs`.
	 *
	 * @param [in] lhs The left-hand side surfel.
	 * @param [in] rhs The point to add.
	 * @return Surfel The resulting surfel.
	 */
	[[nodiscard]] friend Surfel operator+(Surfel lhs, Vec<3, float> rhs) noexcept
	{
		return lhs += rhs;
	}

	/**
	 * @brief Merges another surfel into this one using a numerically stable
	 * parallel update rule.
	 *
	 * @param surfel The surfel to merge in.
	 *
	 * @details
	 * If this surfel is empty the other surfel's statistics are copied directly.
	 * Otherwise the scatter matrix is updated with the cross-term correction:
	 * @code{.cpp}
	 *   S += S_other + (n * n_other / (n + n_other)) * outer(mu - mu_other)
	 * @endcode
	 * where `mu` and `mu_other` are the current means. This is the parallel
	 * variant of Welford's algorithm.
	 */
	constexpr void add(Surfel const& surfel) noexcept
	{
		if (empty()) {
			*this = surfel;
			return;
		}
		add(toDouble(surfel.sum_squares_), cast<double>(surfel.sum_), surfel.num_points_);
	}

	/**
	 * @brief Adds a single 3D point using an online (Welford-like) update rule.
	 *
	 * @param point The point to incorporate.
	 *
	 * @details
	 * If this surfel is empty the point initializes the sum and the scatter
	 * matrix remains zero. Otherwise the scatter matrix is updated with:
	 * @code{.cpp}
	 *   S += (n / (n + 1)) * outer(sum/n - point)
	 * @endcode
	 */
	constexpr void add(Vec<3, float> point) noexcept
	{
		add(std::array<double, 6>{}, cast<double>(point), 1);
	}

	/**
	 * @brief Adds all points in `[first, last)` using a batch algorithm.
	 *
	 * @tparam InputIt `std::input_iterator` whose value type is convertible to `Vec3d`.
	 * @param [in] first The beginning of the input range.
	 * @param [in] last The end of the input range.
	 *
	 * @details
	 * Accumulates raw second moments in a single pass, then applies a
	 * centering correction before merging with the parallel update rule.
	 */
	template <std::input_iterator InputIt>
	  requires std::constructible_from<Vec<3, float>, std::iter_value_t<InputIt>>
	constexpr void add(InputIt first, InputIt last)
	{
		if (first == last) {
			return;
		}
		auto [ss, s, n] = batchScatter(first, last);
		add(ss, s, n);
	}

	/**
	 * @brief Adds all points in `points` using the batch algorithm.
	 *
	 * @tparam Points `std::ranges::input_range` whose elements are convertible to `Vec3d`.
	 * @param [in] points The input range of points.
	 */
	template <std::ranges::input_range Points>
	  requires std::constructible_from<Vec<3, float>, std::ranges::range_value_t<Points>>
	constexpr void add(Points const& points)
	{
		add(begin(points), end(points));
	}

	/**
	 * @brief Adds points from a brace-enclosed initializer list.
	 *
	 * @param [in] points The initializer list of points to add.
	 */
	constexpr void add(std::initializer_list<Vec<3, float>> points) noexcept
	{
		add(begin(points), end(points));
	}

	//
	// Remove
	//

	/**
	 * @brief Removes another surfel's contribution from this one using a numerically stable
	 * parallel update rule.
	 *
	 * @param [in] rhs The surfel to remove.
	 * @return Reference to `*this` after removal.
	 */
	constexpr Surfel& operator-=(Surfel const& rhs) noexcept
	{
		remove(rhs);
		return *this;
	}

	/**
	 * @brief Removes a single 3D point from this surfel and returns `*this`.
	 *
	 * @param [in] rhs The point to remove.
	 * @return Reference to `*this` after removal.
	 */
	constexpr Surfel& operator-=(Vec<3, float> rhs) noexcept
	{
		remove(rhs);
		return *this;
	}

	/**
	 * @brief Returns a new surfel that is the result of removing `rhs` from `lhs`.
	 *
	 * @param [in] lhs The left-hand side surfel.
	 * @param [in] rhs The surfel to remove.
	 * @return A new surfel with `rhs` removed from `lhs`.
	 */
	[[nodiscard]] friend Surfel operator-(Surfel lhs, Surfel const& rhs) noexcept
	{
		return lhs -= rhs;
	}

	/**
	 * @brief Returns a new surfel that is the result of removing point `rhs` from `lhs`.
	 *
	 * @param [in] lhs The left-hand side surfel.
	 * @param [in] rhs The point to remove.
	 * @return A new surfel with point `rhs` removed from `lhs`.
	 */
	[[nodiscard]] friend Surfel operator-(Surfel lhs, Vec<3, float> rhs) noexcept
	{
		return lhs -= rhs;
	}

	/**
	 * @brief Removes a previously-added surfel's contribution from this one.
	 *
	 * @param [in] surfel The surfel to subtract.
	 *
	 * @details
	 * If `surfel` contains at least as many points as this surfel, the surfel is
	 * cleared entirely. Otherwise the scatter matrix is updated by reversing the
	 * parallel merge formula:
	 * @code{.cpp}
	 *   S -= S_other + (n_other * n_result / (n_other + n_result)) * outer(mu_result -
	 * mu_other)
	 * @endcode
	 */
	constexpr void remove(Surfel const& surfel) noexcept
	{
		remove(toDouble(surfel.sum_squares_), cast<double>(surfel.sum_), surfel.num_points_);
	}

	/**
	 * @brief Removes a single previously-added 3D point.
	 *
	 * @param [in] point The point to remove.
	 *
	 * @details
	 * Clears the surfel if it is empty or contains exactly one point. Otherwise
	 * reverses the online update rule used in `add(Vec3f)`:
	 * @code{.cpp}
	 *   S -= (n / (n - 1)) * outer(sum/n - point)
	 * @endcode
	 */
	constexpr void remove(Vec<3, float> point) noexcept
	{
		remove(std::array<double, 6>{}, cast<double>(point), 1);
	}

	/**
	 * @brief Removes all points in `[first, last)` using a batch algorithm.
	 *
	 * @tparam InputIt `std::input_iterator` whose value type is convertible to `Vec3d`.
	 * @param [in] first The beginning of the input range.
	 * @param [in] last The end of the input range.
	 *
	 * @details
	 * Computes the batch's scatter statistics then reverses the parallel merge.
	 */
	template <std::input_iterator InputIt>
	  requires std::constructible_from<Vec<3, float>, std::iter_value_t<InputIt>>
	constexpr void remove(InputIt first, InputIt last)
	{
		if (first == last) {
			return;
		}
		auto [ss, s, n] = batchScatter(first, last);
		remove(ss, s, n);
	}

	/**
	 * @brief Removes all points in `points` using the batch algorithm.
	 *
	 * @tparam Points `std::ranges::input_range` whose elements are convertible to `Vec3d`.
	 * @param [in] points The input range of points.
	 */
	template <std::ranges::input_range Points>
	  requires std::constructible_from<Vec<3, float>, std::ranges::range_value_t<Points>>
	constexpr void remove(Points const& points)
	{
		remove(begin(points), end(points));
	}

	/**
	 * @brief Removes all points in a brace-enclosed initializer list.
	 *
	 * @param [in] points The input range of points.
	 */
	constexpr void remove(std::initializer_list<Vec<3, float>> points) noexcept
	{
		remove(begin(points), end(points));
	}

	//
	// Clear
	//

	/**
	 * @brief Clears the surfel to an empty state (no points, zero statistics).
	 *
	 * @details
	 * After calling this method, `empty()` will return true and all statistics will be
	 * reset to zero.
	 */
	constexpr void clear() noexcept
	{
		sum_squares_ = {};
		sum_         = {};
		num_points_  = {};
	}

	//
	// Get mean
	//

	/**
	 * @brief Returns the centroid (mean position) of all accumulated points.
	 *
	 * @return The mean position of the points in the surfel.
	 *
	 * @pre `!empty()` - behavior is undefined if the surfel contains no points.
	 */
	[[nodiscard]] constexpr Vec<3, double> mean() const noexcept
	{
		return cast<double>(sum_) / static_cast<double>(num_points_);
	}

	//
	// Get covariance
	//

	/**
	 * @brief Returns the 3×3 sample covariance matrix.
	 *
	 * @return The sample covariance matrix of the accumulated points.
	 *
	 * @details
	 * Computed as `sum_squares_ / (n − 1)` where `n = numPoints()`. The matrix is
	 * symmetric; both upper and lower triangles are filled.
	 *
	 * @pre `numPoints() >= 2`.
	 */
	[[nodiscard]] constexpr Mat<3, 3, double> covariance() const noexcept
	{
		double const n = static_cast<double>(num_points_ - 1);
		auto const   s = [&](std::size_t i) noexcept {
      return static_cast<double>(sum_squares_[i]) / n;
		};
		return Mat<3, 3, double>{s(0), s(1), s(2), s(1), s(3), s(4), s(2), s(4), s(5)};
	}

	//
	// Get normal
	//

	/**
	 * @brief Returns the surface normal as the eigenvector of the covariance
	 * matrix corresponding to the smallest eigenvalue.
	 *
	 * @return Unit vector normal to the local surface.
	 *
	 * @pre `numPoints() >= 2`.
	 */
	[[nodiscard]] constexpr Vec3d normal() const noexcept
	{
		return eigenVectors(covariance())[0];
	}

	//
	// Get planarity
	//

	/**
	 * @brief Returns a planarity measure in [0, 1].
	 *
	 * @return Planarity measure in [0, 1], where 1 indicates a perfectly planar arrangement
	 * and 0 indicates an isotropic distribution.
	 *
	 * @details
	 * Computed as `2 * (e1 - e0) / (e0 + e1 + e2)` where `e0 <= e1 <= e2` are
	 * the sorted eigenvalues of the covariance matrix. A value of 1 indicates a
	 * perfectly planar arrangement; 0 indicates an isotropic distribution.
	 *
	 * @pre `numPoints() >= 2`.
	 */
	[[nodiscard]] constexpr double planarity() const noexcept
	{
		auto const e = eigenValues(covariance());
		return 2.0 * (e[1] - e[0]) / (e[0] + e[1] + e[2]);
	}

	//
	// Get num points
	//

	/**
	 * @brief Returns the number of points currently represented by this surfel.
	 *
	 * @return The number of points in the surfel.
	 */
	[[nodiscard]] constexpr std::uint32_t numPoints() const noexcept { return num_points_; }

	//
	// Get sum
	//

	/**
	 * @brief Returns the raw sum of all point positions.
	 *
	 * @return The sum of all point positions.
	 *
	 * @details
	 * This is the unnormalized sum of all point positions. The mean can be obtained by
	 * dividing this sum by `numPoints()`, or by calling the `mean()` method directly.
	 */
	[[nodiscard]] constexpr Vec<3, float> sum() const noexcept { return sum_; }

	//
	// Get sum squares
	//

	/**
	 * @brief Returns the raw scatter matrix upper triangle (Sxx, Sxy, Sxz, Syy,
	 * Syz, Szz).
	 *
	 * @return The upper triangle of the scatter matrix.
	 *
	 * @details
	 * Dividing by `(numPoints() − 1)` yields the sample covariance upper triangle.
	 */
	[[nodiscard]] constexpr std::array<float, 6> sumSquares() const noexcept
	{
		return sum_squares_;
	}

 protected:
	//
	// Helpers
	//

	/**
	 * @brief Converts an array of six `float`s to `double`s for intermediate calculations.
	 *
	 * @param [in] arr The input array of six `float`s representing the upper triangle of
	 * the scatter matrix.
	 * @return An array of six `double`s converted from the input `float`s.
	 */
	[[nodiscard]] static constexpr std::array<double, 6> toDouble(
	    std::array<float, 6> const& arr) noexcept
	{
		std::array<double, 6> result;
		std::ranges::transform(arr, result.begin(),
		                       [](float v) { return static_cast<double>(v); });
		return result;
	}

	/**
	 * @brief Converts an array of six `double`s to `float`s for storage.
	 *
	 * @param [in] src The input array of six `double`s.
	 * @param [out] dst The output array of six `float`s.
	 */
	static constexpr void toFloat(std::array<double, 6> const& src,
	                              std::array<float, 6>&        dst) noexcept
	{
		std::ranges::transform(src, dst.begin(),
		                       [](double v) { return static_cast<float>(v); });
	}

	/**
	 * @brief Helper to add the outer product correction term in the parallel update.
	 *
	 * @param [in,out] ss The scatter matrix upper triangle to update.
	 * @param [in] v The vector used in the outer product correction.
	 * @param [in] scale The scaling factor for the correction term.
	 */
	static constexpr void addOuter(std::array<double, 6>& ss, Vec<3, double> const& v,
	                               double scale) noexcept
	{
		ss[0] += v[0] * v[0] * scale;
		ss[1] += v[0] * v[1] * scale;
		ss[2] += v[0] * v[2] * scale;
		ss[3] += v[1] * v[1] * scale;
		ss[4] += v[1] * v[2] * scale;
		ss[5] += v[2] * v[2] * scale;
	}

	/**
	 * @brief Computes scatter statistics `(ss, sum, n)` for a batch of points.
	 *
	 * @tparam InputIt `std::input_iterator` whose value type is convertible to `Vec3d`.
	 * @param [in] first The beginning of the input range.
	 * @param [in] last The end of the input range.
	 * @return A tuple containing:
	 * - `ss`: An array of six `double`s representing the upper triangle of the scatter
	 * matrix.
	 * - `sum`: The sum of all point positions as a `Vec<3, double>`.
	 * - `n`: The number of points in the batch as a `std::uint32_t`.
	 *
	 * @details
	 * Accumulates raw second moments in a single pass, then subtracts the
	 * centering correction `s[i]*s[j]/n` to produce the scatter matrix.
	 */
	template <std::input_iterator InputIt>
	  requires std::constructible_from<Vec<3, double>, std::iter_value_t<InputIt>>
	[[nodiscard]] static constexpr std::tuple<std::array<double, 6>, Vec<3, double>,
	                                          std::uint32_t>
	batchScatter(InputIt first, InputIt last) noexcept
	{
		std::array<double, 6> ss{};
		Vec<3, double>        s{};
		std::uint32_t         n{};

		for (; first != last; ++first) {
			Vec<3, double> const p(*first);
			ss[0] += p[0] * p[0];
			ss[1] += p[0] * p[1];
			ss[2] += p[0] * p[2];
			ss[3] += p[1] * p[1];
			ss[4] += p[1] * p[2];
			ss[5] += p[2] * p[2];
			s += p;
			++n;
		}

		double const nd = static_cast<double>(n);
		ss[0] -= s[0] * s[0] / nd;
		ss[1] -= s[0] * s[1] / nd;
		ss[2] -= s[0] * s[2] / nd;
		ss[3] -= s[1] * s[1] / nd;
		ss[4] -= s[1] * s[2] / nd;
		ss[5] -= s[2] * s[2] / nd;

		return {ss, s, n};
	}

	//
	// Private add/remove
	//

	/**
	 * @brief Helper to add a batch of points represented by their scatter statistics.
	 *
	 * @param [in] ss The scatter matrix upper triangle of the batch to add.
	 * @param [in] sum The sum of all point positions in the batch.
	 * @param [in] num_points The number of points in the batch.
	 */
	constexpr void add(std::array<double, 6> ss, Vec<3, double> const& sum,
	                   std::uint32_t num_points)
	{
		if (empty()) {
			toFloat(ss, sum_squares_);
			sum_        = cast<float>(sum);
			num_points_ = num_points;
			return;
		}

		double const         n   = static_cast<double>(num_points_);
		double const         n_o = static_cast<double>(num_points);
		Vec<3, double> const s(sum_);

		std::ranges::transform(ss, sum_squares_, ss.begin(),
		                       [](double a, float b) { return a + static_cast<double>(b); });
		addOuter(ss, s * n_o - sum * n, 1.0 / (n * n_o * (n + n_o)));

		toFloat(ss, sum_squares_);
		sum_ = cast<float>(s + sum);
		num_points_ += num_points;
	}

	/**
	 * @brief Helper to remove a batch of points represented by their scatter statistics.
	 *
	 * @param [in] ss The scatter matrix upper triangle of the batch to remove.
	 * @param [in] sum The sum of all point positions in the batch.
	 * @param [in] num_points The number of points in the batch.
	 */
	constexpr void remove(std::array<double, 6> ss, Vec<3, double> const& sum,
	                      std::uint32_t num_points)
	{
		if (num_points >= num_points_) {
			clear();
			return;
		}

		double const         n_a   = static_cast<double>(num_points_ - num_points);
		double const         n_b   = static_cast<double>(num_points);
		Vec<3, double> const sum_a = cast<double>(sum_) - sum;

		std::ranges::transform(sum_squares_, ss, ss.begin(),
		                       [](float a, double b) { return static_cast<double>(a) - b; });
		addOuter(ss, sum_a * n_b - sum * n_a, -1.0 / (n_a * n_b * (n_a + n_b)));

		toFloat(ss, sum_squares_);
		sum_ = cast<float>(sum_a);
		num_points_ -= num_points;
	}

	//
	// Data members
	//

	/**
	 * @brief Upper triangle of the scatter matrix S, stored as six `float`s in the order
	 * (Sxx, Sxy, Sxz, Syy, Syz, Szz). S accumulates the sum of squared deviations from the
	 * running mean; the sample covariance is S / (n − 1).
	 */
	std::array<float, 6> sum_squares_{};

	/**
	 * @brief The sum of all point positions (a `Vec<3, float>`). This is the unnormalized
	 * sum; the mean can be obtained by dividing this sum by `numPoints()`, or by calling
	 * the `mean()` method directly.
	 */
	Vec<3, float> sum_{};

	/**
	 * @brief The number of contributing points. This is used to compute the mean and
	 * covariance from the raw sums. If this is zero, the surfel is considered empty and all
	 * statistics are zero. If this is one, the mean is equal to the single point and the
	 * covariance is undefined (but treated as zero). For two or more points, the mean and
	 * covariance are computed from the sums as described in the method documentation.
	 */
	std::uint32_t num_points_{};
};

/**
 * @}
 */
}  // namespace ufo

template <>
struct std::formatter<ufo::Surfel> {
	constexpr auto parse(std::format_parse_context& ctx) const { return ctx.begin(); }

	auto format(ufo::Surfel const& s, std::format_context& ctx) const
	{
		if (s.empty()) {
			return std::format_to(ctx.out(), "Surfel{{empty}}");
		}
		auto const m = s.mean();
		return std::format_to(ctx.out(), "Surfel{{n={}, mean=({}, {}, {})}}", s.numPoints(),
		                      m[0], m[1], m[2]);
	}
};

namespace ufo
{
/**
 * @ingroup core
 * @brief Writes a human-readable summary of the surfel to `out`.
 *
 * @param [out] out Output stream.
 * @param [in] s Surfel to print.
 * @return Reference to the output stream.
 *
 * @details
 * Empty surfel: `"Surfel{empty}"`.
 * Non-empty:    `"Surfel{n=N, mean=(x, y, z)}"`.
 */
inline std::ostream& operator<<(std::ostream& out, Surfel const& s)
{
	return out << std::format("{}", s);
}
}  // namespace ufo
#endif  // UFO_CORE_SURFEL_HPP
