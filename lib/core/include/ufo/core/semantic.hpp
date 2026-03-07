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

#ifndef UFO_CORE_SEMANTIC_HPP
#define UFO_CORE_SEMANTIC_HPP

// UFO
#include <ufo/core/confidence.hpp>
#include <ufo/core/label.hpp>

// STL
#include <concepts>
#include <format>
#include <ostream>

namespace ufo
{
/**
 * @ingroup core
 * @{
 */

/**
 * @brief Represents a semantic annotation as a pair of class label and confidence score.
 *
 * @details
 * `Semantic` combines a discrete category identifier (`Label`) with a continuous
 * confidence measure (`Confidence`), making it suitable for probabilistic semantic
 * segmentation of point clouds and map nodes. It is a lightweight, trivially-copyable
 * aggregate, fully ordered via `<=>`, and supports `std::ostream` streaming (as "label:
 * confidence") and `std::format`.
 *
 * Intended for use as a per-point or per-node attribute in semantic mapping, where both
 * the class and its associated certainty are required.
 *
 * @see Label, Confidence
 */
struct Semantic {
	/**
	 * @brief The discrete semantic class identifier.
	 */
	Label label{};

	/**
	 * @brief The associated confidence or weight score.
	 */
	Confidence confidence{};

	/**
	 * @brief Three-way comparison (lexicographic on label then value).
	 *
	 * @param [in] lhs Left operand.
	 * @param [in] rhs Right operand.
	 * @return Comparison result.
	 */
	[[nodiscard]] friend constexpr auto operator<=>(Semantic lhs,
	                                                Semantic rhs) noexcept = default;

	/**
	 * @brief Equality comparison.
	 *
	 * @param [in] lhs Left operand.
	 * @param [in] rhs Right operand.
	 * @return True if both label and confidence are equal.
	 */
	[[nodiscard]] friend constexpr bool operator==(Semantic lhs,
	                                               Semantic rhs) noexcept = default;
};

/**
 * @}
 */

/**
 * @ingroup core
 * @brief Writes the semantic as "label: confidence" to `out`.
 *
 * @param [in,out] out Output stream.
 * @param [in] s Semantic to print.
 * @return Reference to the output stream.
 */
inline std::ostream& operator<<(std::ostream& out, Semantic s)
{
	return out << s.label << ": " << s.confidence;
}
}  // namespace ufo

template <>
struct std::formatter<ufo::Semantic> {
	constexpr auto parse(std::format_parse_context& ctx) const { return ctx.begin(); }

	auto format(ufo::Semantic s, std::format_context& ctx) const
	{
		return std::format_to(ctx.out(), "{}: {}", s.label, s.confidence);
	}
};
#endif  // UFO_CORE_SEMANTIC_HPP
