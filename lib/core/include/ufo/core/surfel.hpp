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
	 * Intended for deserialization or copying internal state.
	 *
	 * @param sum         Sum of all point positions.
	 * @param sum_squares Upper triangle of the scatter matrix (Sxx, Sxy, Sxz, Syy, Syz,
	 *                    Szz).
	 * @param num_points  Number of points represented by this surfel.
	 */
	constexpr Surfel(Vec<3, float> sum, std::array<float, 6> sum_squares,
	                 std::uint32_t num_points)
	    : sum_squares_(sum_squares), sum_(sum), num_points_(num_points)
	{
	}

	/**
	 * @brief Constructs a surfel from a single position and optional scatter matrix.
	 * @param position The point position.
	 * @param scatter Optional scatter matrix.
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
	 */
	template <std::input_iterator InputIt>
	constexpr Surfel(InputIt first, InputIt last)
	{
		add(first, last);
	}

	/**
	 * @brief Constructs a surfel by adding all points in @p points.
	 *
	 * @tparam Range `std::ranges::input_range` whose elements are convertible to `Vec3d`.
	 */
	template <std::ranges::input_range Range>
	  requires(!std::same_as<std::remove_cvref_t<Range>, Surfel>)
	constexpr Surfel(Range const& points) : Surfel(std::cbegin(points), std::cend(points))
	{
	}

	//! @brief Constructs a surfel from a brace-enclosed list of `Vec3f` points.
	constexpr Surfel(std::initializer_list<Vec3f> points)
	    : Surfel(begin(points), end(points))
	{
	}

	//! @brief Equality comparison on all three stored statistics.
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

	//! @brief Merges @p rhs into this surfel and returns `*this`.
	constexpr Surfel& operator+=(Surfel const& rhs) noexcept
	{
		add(rhs);
		return *this;
	}

	//! @brief Adds point @p rhs to this surfel and returns `*this`.
	constexpr Surfel& operator+=(Vec<3, float> rhs) noexcept
	{
		add(rhs);
		return *this;
	}

	//! @brief Returns a new surfel that is the merge of @p lhs and @p rhs.
	[[nodiscard]] friend Surfel operator+(Surfel lhs, Surfel const& rhs) noexcept
	{
		return lhs += rhs;
	}

	//! @brief Returns a new surfel that is @p lhs with point @p rhs added.
	[[nodiscard]] friend Surfel operator+(Surfel lhs, Vec<3, float> rhs) noexcept
	{
		return lhs += rhs;
	}

	/**
	 * @brief Merges another surfel into this one using a numerically stable
	 * parallel update rule.
	 *
	 * If this surfel is empty the other surfel's statistics are copied directly.
	 * Otherwise the scatter matrix is updated with the cross-term correction:
	 * @code
	 *   S += S_other + (n * n_other / (n + n_other)) * outer(mu - mu_other)
	 * @endcode
	 * where @p mu and @p mu_other are the current means. This is the parallel
	 * variant of Welford's algorithm.
	 *
	 * @param surfel The surfel to merge in.
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
	 * If this surfel is empty the point initializes the sum and the scatter
	 * matrix remains zero. Otherwise the scatter matrix is updated with:
	 * @code
	 *   S += (n / (n + 1)) * outer(sum/n - point)
	 * @endcode
	 *
	 * @param point The point to incorporate.
	 */
	constexpr void add(Vec<3, float> point) noexcept
	{
		add(std::array<double, 6>{}, cast<double>(point), 1);
	}

	/**
	 * @brief Adds all points in `[first, last)` using a batch algorithm.
	 *
	 * Accumulates raw second moments in a single pass, then applies a
	 * centering correction before merging with the parallel update rule.
	 *
	 * @tparam InputIt `std::input_iterator` whose value type is convertible to `Vec3d`.
	 */
	template <std::input_iterator InputIt>
	constexpr void add(InputIt first, InputIt last)
	{
		if (first == last) {
			return;
		}
		auto [ss, s, n] = batchScatter(first, last);
		add(ss, s, n);
	}

	/**
	 * @brief Adds all points in @p points using the batch algorithm.
	 *
	 * @tparam Range `std::ranges::input_range` whose elements are convertible to `Vec3d`.
	 */
	template <std::ranges::input_range Range>
	  requires(!std::same_as<std::remove_cvref_t<Range>, Surfel>)
	constexpr void add(Range const& points)
	{
		add(begin(points), end(points));
	}

	//! @brief Adds points from a brace-enclosed initializer list.
	constexpr void add(std::initializer_list<Vec<3, float>> points) noexcept
	{
		add(begin(points), end(points));
	}

	//
	// Remove
	//

	//! @brief Removes @p rhs's contribution from this surfel and returns `*this`.
	constexpr Surfel& operator-=(Surfel const& rhs) noexcept
	{
		remove(rhs);
		return *this;
	}

	//! @brief Removes point @p rhs from this surfel and returns `*this`.
	constexpr Surfel& operator-=(Vec<3, float> rhs) noexcept
	{
		remove(rhs);
		return *this;
	}

	//! @brief Returns a new surfel with @p rhs's contribution removed from @p lhs.
	[[nodiscard]] friend Surfel operator-(Surfel lhs, Surfel const& rhs) noexcept
	{
		return lhs -= rhs;
	}

	//! @brief Returns a new surfel with point @p rhs removed from @p lhs.
	[[nodiscard]] friend Surfel operator-(Surfel lhs, Vec<3, float> rhs) noexcept
	{
		return lhs -= rhs;
	}

	/**
	 * @brief Removes a previously-added surfel's contribution from this one.
	 *
	 * If @p surfel contains at least as many points as this surfel, the surfel is
	 * cleared entirely. Otherwise the scatter matrix is updated by reversing the
	 * parallel merge formula:
	 * @code
	 *   S -= S_other + (n_other * n_result / (n_other + n_result)) * outer(mu_result -
	 * mu_other)
	 * @endcode
	 *
	 * @param surfel The surfel to subtract.
	 */
	constexpr void remove(Surfel const& surfel) noexcept
	{
		remove(toDouble(surfel.sum_squares_), cast<double>(surfel.sum_), surfel.num_points_);
	}

	/**
	 * @brief Removes a single previously-added 3D point.
	 *
	 * Clears the surfel if it is empty or contains exactly one point. Otherwise
	 * reverses the online update rule used in `add(Vec3f)`:
	 * @code
	 *   S -= (n / (n - 1)) * outer(sum/n - point)
	 * @endcode
	 *
	 * @param point The point to remove.
	 */
	constexpr void remove(Vec<3, float> point) noexcept
	{
		remove(std::array<double, 6>{}, cast<double>(point), 1);
	}

	/**
	 * @brief Removes all points in `[first, last)` using a batch algorithm.
	 *
	 * Computes the batch's scatter statistics then reverses the parallel merge.
	 *
	 * @tparam InputIt `std::input_iterator` whose value type is convertible to `Vec3d`.
	 */
	template <std::input_iterator InputIt>
	constexpr void remove(InputIt first, InputIt last)
	{
		if (first == last) {
			return;
		}
		auto [ss, s, n] = batchScatter(first, last);
		remove(ss, s, n);
	}

	/**
	 * @brief Removes all points in @p points using the batch algorithm.
	 *
	 * @tparam Range `std::ranges::input_range` whose elements are convertible to `Vec3d`.
	 */
	template <std::ranges::input_range Range>
	  requires(!std::same_as<std::remove_cvref_t<Range>, Surfel>)
	constexpr void remove(Range const& points)
	{
		remove(begin(points), end(points));
	}

	//! @brief Removes points from a brace-enclosed initializer list.
	constexpr void remove(std::initializer_list<Vec<3, float>> points) noexcept
	{
		remove(begin(points), end(points));
	}

	//
	// Clear
	//

	//! @brief Resets the surfel to an empty state (no points, zero statistics).
	constexpr void clear() noexcept
	{
		num_points_  = {};
		sum_         = {};
		sum_squares_ = {};
	}

	//
	// Get mean
	//

	/**
	 * @brief Returns the centroid (mean position) of all accumulated points.
	 *
	 * @pre `!empty()` — behavior is undefined if the surfel contains no points.
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

	//! @brief Returns the number of 3D points currently represented by this surfel.
	[[nodiscard]] constexpr std::uint32_t numPoints() const noexcept { return num_points_; }

	//
	// Get sum
	//

	/**
	 * @brief Returns the raw sum of all point positions.
	 *
	 * Divide by `numPoints()` to obtain the mean, or use `mean()` directly.
	 */
	[[nodiscard]] constexpr Vec<3, float> sum() const noexcept { return sum_; }

	//
	// Get sum squares
	//

	/**
	 * @brief Returns the raw scatter matrix upper triangle (Sxx, Sxy, Sxz, Syy,
	 * Syz, Szz).
	 *
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

	//! @brief Converts a `float[6]` scatter array to `double[6]`.
	[[nodiscard]] static constexpr std::array<double, 6> toDouble(
	    std::array<float, 6> const& arr) noexcept
	{
		std::array<double, 6> result;
		std::ranges::transform(arr, result.begin(),
		                       [](float v) { return static_cast<double>(v); });
		return result;
	}

	//! @brief Stores a `double[6]` scatter array back as `float[6]`.
	static constexpr void toFloat(std::array<double, 6> const& src,
	                              std::array<float, 6>&        dst) noexcept
	{
		std::ranges::transform(src, dst.begin(),
		                       [](double v) { return static_cast<float>(v); });
	}

	//! @brief Applies `ss += scale * outer(v, v)` to the upper triangle of @p ss.
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
	 * Accumulates raw second moments in a single pass, then subtracts the
	 * centering correction `s[i]*s[j]/n` to produce the scatter matrix.
	 */
	template <std::input_iterator InputIt>
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

	//! Upper triangle of the scatter matrix (Sxx, Sxy, Sxz, Syy, Syz, Szz).
	std::array<float, 6> sum_squares_{};
	//! Sum of all accumulated point positions.
	Vec<3, float> sum_{};
	//! Number of points accumulated so far.
	std::uint32_t num_points_{};
};
}  // namespace ufo

/**
 * @brief `std::format` / `std::formatter` specialization for `ufo::Surfel`.
 *
 * Empty surfel: `"Surfel{empty}"`.
 * Non-empty:    `"Surfel{n=N, mean=(x, y, z)}"`.
 * No format specifier is accepted; the format string must be empty (`{}`).
 *
 * @tparam T Scalar type.
 */
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
 * @brief Writes a human-readable summary of the surfel to @p out.
 *
 * @param out Output stream.
 * @param s Surfel to print.
 * @return Reference to the output stream.
 *
 * Empty surfel: `"Surfel{empty}"`.
 * Non-empty:    `"Surfel{n=N, mean=(x, y, z)}"`.
 */
inline std::ostream& operator<<(std::ostream& out, Surfel const& s)
{
	return out << std::format("{}", s);
}
}  // namespace ufo
#endif  // UFO_CORE_SURFEL_HPP
