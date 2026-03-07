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

#ifndef UFO_CORE_RANGE_HPP
#define UFO_CORE_RANGE_HPP

// STL
#include <format>
#include <ostream>
#include <type_traits>
#include <utility>

namespace ufo
{
/**
 * @ingroup core
 * @{
 */

/**
 * @brief Represents a closed interval [lower, upper] of a scalar type.
 *
 * @tparam T Scalar type (e.g., int, float, double).
 *
 * @details
 * Provides interval operations, ordering, and containment checks.
 */
template <typename T>
struct Range {
	/**
	 * @brief Scalar type of the interval bounds.
	 */
	using value_type = T;

	/**
	 * @brief Lower bound of the interval (inclusive).
	 */
	value_type lower;

	/**
	 * @brief Upper bound of the interval (inclusive).
	 */
	value_type upper;

	/**
	 * @brief Comparator for interval ordering in associative containers.
	 *
	 * @details
	 * Implements interval ordering: [a, b] < [c, d] iff b < c.
	 * Supports transparent (heterogeneous) lookup.
	 */
	// struct Comparator {
	// 	using is_transparent = std::true_type;

	// 	/**
	// 	 * @brief Interval ordering for associative containers.
	// 	 *
	// 	 * @param lhs Left operand (Range or pair).
	// 	 * @param rhs Right operand (Range or pair).
	// 	 * @return True if lhs.upper < rhs.lower.
	// 	 */
	// 	template <typename L, typename R>
	// 	bool operator()(L const& lhs, R const& rhs) const
	// 	{
	// 		return lhs.upper < rhs.lower;
	// 	}
	// };

	struct Comparator {
		using range_type     = std::pair<T, T>;
		using is_transparent = std::true_type;

		[[nodiscard]] constexpr bool operator()(Range lhs, Range rhs) const noexcept
		{
			// return lhs.upper < rhs.lower;
			return lhs.lower < rhs.lower && lhs.upper < rhs.upper;
		}
		[[nodiscard]] constexpr bool operator()(Range lhs, range_type rhs) const noexcept
		{
			// return lhs.upper < rhs.first;
			return lhs.lower < rhs.first && lhs.upper < rhs.second;
		}
		[[nodiscard]] constexpr bool operator()(range_type lhs, Range rhs) const noexcept
		{
			// return lhs.second < rhs.lower;
			return lhs.first < rhs.lower && lhs.second < rhs.upper;
		}
	};

	/**
	 * @brief Checks if a value is contained within the interval.
	 *
	 * @param [in] value Value to check.
	 * @return True if value is in [lower, upper].
	 */
	[[nodiscard]] constexpr bool contains(value_type value) const noexcept
	{
		return lower <= value && value <= upper;
	}

	/**
	 * @brief Checks if another range is fully contained within this interval.
	 *
	 * @param [in] other Range to check.
	 * @return True if other is in [lower, upper].
	 */
	[[nodiscard]] constexpr bool contains(Range const& other) const noexcept
	{
		return lower <= other.lower && other.upper <= upper;
	}

	/**
	 * @brief Checks if this interval overlaps with another range.
	 *
	 * @param [in] other Range to check.
	 * @return True if the intervals overlap (i.e., they share any common values).
	 */
	[[nodiscard]] constexpr bool overlaps(Range const& other) const noexcept
	{
		return lower <= other.upper && other.lower <= upper;
	}

	/**
	 * @brief Equality comparison.
	 *
	 * @param [in] lhs Left operand.
	 * @param [in] rhs Right operand.
	 * @return True if both bounds are equal.
	 */
	[[nodiscard]] friend constexpr bool operator==(Range const& lhs,
	                                               Range const& rhs) noexcept = default;

	/**
	 * @brief Interval ordering: `[a, b] < [c, d]` iff `b < c`.
	 *
	 * @param [in] lhs Left operand.
	 * @param [in] rhs Right operand.
	 * @return True if this interval is entirely before rhs.
	 */
	[[nodiscard]] friend constexpr bool operator<(Range const& lhs,
	                                              Range const& rhs) noexcept
	{
		return lhs.upper < rhs.lower;
	}

	/**
	 * @brief Interval ordering: `[a, b] <= [c, d]` iff `b <= c`.
	 *
	 * @param [in] lhs Left operand.
	 * @param [in] rhs Right operand.
	 * @return True if this interval is before or touching rhs.
	 */
	[[nodiscard]] friend constexpr bool operator<=(Range const& lhs,
	                                               Range const& rhs) noexcept
	{
		return lhs.upper <= rhs.lower;
	}

	/**
	 * @brief Interval ordering: `[a, b] > [c, d]` iff `a > d`.
	 *
	 * @param [in] lhs Left operand.
	 * @param [in] rhs Right operand.
	 * @return True if this interval is entirely after rhs.
	 */
	[[nodiscard]] friend constexpr bool operator>(Range const& lhs,
	                                              Range const& rhs) noexcept
	{
		return lhs.lower > rhs.upper;
	}

	/**
	 * @brief Interval ordering: `[a, b] >= [c, d]` iff `a >= d`.
	 *
	 * @param [in] lhs Left operand.
	 * @param [in] rhs Right operand.
	 * @return True if this interval is after or touching rhs.
	 */
	[[nodiscard]] friend constexpr bool operator>=(Range const& lhs,
	                                               Range const& rhs) noexcept
	{
		return lhs.lower >= rhs.upper;
	}

	/**
	 * @brief Equality comparison with scalar: `[a, b] == t` iff `a == t && b == t`.
	 *
	 * @param [in] lhs Range operand.
	 * @param [in] rhs Scalar operand.
	 * @return True if both bounds equal rhs.
	 */
	[[nodiscard]] friend constexpr bool operator==(Range const& lhs,
	                                               value_type   rhs) noexcept
	{
		return lhs.lower == rhs && lhs.upper == rhs;
	}

	/**
	 * @brief Range-scalar ordering: `[a, b] < t` iff `b < t`.
	 *
	 * @param [in] lhs Range operand.
	 * @param [in] rhs Scalar operand.
	 * @return True if upper < rhs.
	 */
	[[nodiscard]] friend constexpr bool operator<(Range const& lhs, value_type rhs) noexcept
	{
		return lhs.upper < rhs;
	}

	/**
	 * @brief Range-scalar ordering: `[a, b] <= t` iff `b <= t`.
	 *
	 * @param [in] lhs Range operand.
	 * @param [in] rhs Scalar operand.
	 * @return True if upper <= rhs.
	 */
	[[nodiscard]] friend constexpr bool operator<=(Range const& lhs,
	                                               value_type   rhs) noexcept
	{
		return lhs.upper <= rhs;
	}

	/**
	 * @brief Range-scalar ordering: `[a, b] > t` iff `a > t`.
	 *
	 * @param [in] lhs Range operand.
	 * @param [in] rhs Scalar operand.
	 * @return True if lower > rhs.
	 */
	[[nodiscard]] friend constexpr bool operator>(Range const& lhs, value_type rhs) noexcept
	{
		return lhs.lower > rhs;
	}

	/**
	 * @brief Range-scalar ordering: `[a, b] >= t` iff `a >= t`.
	 *
	 * @param [in] lhs Range operand.
	 * @param [in] rhs Scalar operand.
	 * @return True if lower >= rhs.
	 */
	[[nodiscard]] friend constexpr bool operator>=(Range const& lhs,
	                                               value_type   rhs) noexcept
	{
		return lhs.lower >= rhs;
	}

	/**
	 * @brief Equality comparison with scalar: `t == [a, b]` iff `t == a && t == b`.
	 *
	 * @param [in] lhs Scalar operand.
	 * @param [in] rhs Range operand.
	 * @return True if both bounds equal lhs.
	 */
	[[nodiscard]] friend constexpr bool operator==(value_type   lhs,
	                                               Range const& rhs) noexcept
	{
		return rhs == lhs;
	}

	/**
	 * @brief Scalar-range ordering: `t < [a, b]` iff `t < a`.
	 *
	 * @param [in] lhs Scalar operand.
	 * @param [in] rhs Range operand.
	 * @return True if lhs < rhs.lower.
	 */
	[[nodiscard]] friend constexpr bool operator<(value_type lhs, Range const& rhs) noexcept
	{
		return rhs > lhs;
	}

	/**
	 * @brief Scalar-range ordering: `t <= [a, b]` iff `t <= a`.
	 *
	 * @param [in] lhs Scalar operand.
	 * @param [in] rhs Range operand.
	 * @return True if lhs <= rhs.lower.
	 */
	[[nodiscard]] friend constexpr bool operator<=(value_type   lhs,
	                                               Range const& rhs) noexcept
	{
		return rhs >= lhs;
	}

	/**
	 * @brief Scalar-range ordering: `t > [a, b]` iff `t > b`.
	 *
	 * @param [in] lhs Scalar operand.
	 * @param [in] rhs Range operand.
	 * @return True if lhs > rhs.upper.
	 */
	[[nodiscard]] friend constexpr bool operator>(value_type lhs, Range const& rhs) noexcept
	{
		return rhs < lhs;
	}

	/**
	 * @brief Scalar-range ordering: `t >= [a, b]` iff `t >= b`.
	 *
	 * @param [in] lhs Scalar operand.
	 * @param [in] rhs Range operand.
	 * @return True if lhs >= rhs.upper.
	 */
	[[nodiscard]] friend constexpr bool operator>=(value_type   lhs,
	                                               Range const& rhs) noexcept
	{
		return rhs <= lhs;
	}
};

/**
 * @}
 */
}  // namespace ufo

template <class T>
struct std::formatter<ufo::Range<T>> {
	constexpr auto parse(std::format_parse_context& ctx) const { return ctx.begin(); }

	auto format(ufo::Range<T> const& r, std::format_context& ctx) const
	{
		if (r.lower == r.upper) {
			return std::format_to(ctx.out(), "[{}]", +r.lower);
		} else if constexpr (std::is_floating_point_v<T>) {
			return std::format_to(ctx.out(), "[{},{}]", +r.lower, +r.upper);
		} else {
			return std::format_to(ctx.out(), "[{}..{}]", +r.lower, +r.upper);
		}
	}
};

namespace ufo
{
/**
 * @ingroup core
 * @brief Writes the range to `p` out in human-readable format.
 *
 * @tparam T Scalar type.
 * @param [in,out] out Output stream.
 * @param [in] r Range to print.
 * @return Reference to the output stream.
 *
 * @details
 * Degenerate: [a, a] -> "[a]"
 * Range:      [a, b] -> "[a..b]"
 * Float:      [a, b] -> "[a,b]"
 */
template <class T>
std::ostream& operator<<(std::ostream& out, Range<T> const& r)
{
	return out << std::format("{}", r);
}
}  // namespace ufo

#endif  // UFO_CORE_RANGE_HPP
