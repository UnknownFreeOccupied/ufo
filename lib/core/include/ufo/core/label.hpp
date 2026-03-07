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

#ifndef UFO_CORE_LABEL_HPP
#define UFO_CORE_LABEL_HPP

// STL
#include <cstdint>
#include <format>
#include <ostream>

namespace ufo
{
/**
 * @ingroup core
 * @{
 */

/**
 * @brief Represents a semantic class label as a single unsigned integer value.
 *
 * @details
 * `Label` is a lightweight, trivially-copyable value type with no overhead over a
 * bare `uint32_t`. It is implicitly convertible to and from `uint32_t`, fully ordered via
 * `<=>`, and supports `std::ostream` streaming and `std::format`.
 *
 * Intended for use as a per-point or per-node attribute in a semantic segmentation
 * pipeline, where it can be stored as a separate SoA channel alongside position or
 * intensity data. The integer backing allows up to ~4 billion distinct categories (e.g.,
 * "car", "wall", "ground").
 *
 * `Label` is the category component of `Semantic`; pair it with `Confidence` when you
 * also need a per-label confidence score.
 */
struct Label {
	/**
	 * @brief Underlying integer type.
	 */
	using value_type = std::uint32_t;

	/**
	 * @brief The raw class label identifier.
	 */
	value_type label{};

	/**
	 * @brief Implicitly converts to the underlying integer type.
	 * @return The label value as a uint32_t.
	 *
	 * @details
	 * Allows `Label` to be used wherever a `uint32_t` is expected.
	 */
	[[nodiscard]] constexpr operator value_type() const noexcept { return label; }

	/**
	 * @brief Three-way comparison (total order on the underlying integer).
	 *
	 * @param [in] lhs Left operand.
	 * @param [in] rhs Right operand.
	 * @return Comparison result.
	 */
	[[nodiscard]] friend constexpr auto operator<=>(Label lhs,
	                                                Label rhs) noexcept = default;

	/**
	 * @brief Equality comparison.
	 *
	 * @param [in] lhs Left operand.
	 * @param [in] rhs Right operand.
	 * @return True if both label values are equal.
	 */
	[[nodiscard]] friend constexpr bool operator==(Label lhs, Label rhs) noexcept = default;
};

/**
 * @}
 */

/**
 * @ingroup core
 * @brief Writes the raw label integer to `out`.
 *
 * @param [in,out] out Output stream.
 * @param [in] l Label to print.
 * @return Reference to the output stream.
 */
inline std::ostream& operator<<(std::ostream& out, Label l) { return out << l.label; }
}  // namespace ufo

template <>
struct std::formatter<ufo::Label> : std::formatter<ufo::Label::value_type> {
	auto format(ufo::Label l, std::format_context& ctx) const
	{
		return std::formatter<ufo::Label::value_type>::format(l.label, ctx);
	}
};
#endif  // UFO_CORE_LABEL_HPP
