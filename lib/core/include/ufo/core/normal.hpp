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

#ifndef UFO_CORE_NORMAL_HPP
#define UFO_CORE_NORMAL_HPP

// UFO
#include <ufo/math/vec.hpp>

// STL
#include <concepts>
#include <cstddef>
#include <format>
#include <ostream>

namespace ufo
{
/**
 * @brief Represents an N-dimensional unit surface normal vector as a fixed-size array.
 *
 * @details
 * `Normal<N, T>` is a trivially-copyable aggregate that holds a surface normal direction.
 * The normal is not automatically normalized on construction — callers are responsible
 * for ensuring unit length when semantics require it.
 *
 * Intended for use as a per-point attribute in a `Cloud` / `PointCloud` SoA channel,
 * alongside position data and other attributes.
 *
 * @tparam Dim Dimensionality of the normal vector (typically 2 or 3).
 * @tparam T   Scalar type for each component (defaults to `float`).
 *
 * Convenience aliases:
 * - `Normal2f` / `Normal3f` / `Normal4f` — single-precision
 * - `Normal2d` / `Normal3d` / `Normal4d` — double-precision
 */
template <std::size_t Dim = 3, std::floating_point T = float>
struct Normal : Vec<Dim, T> {
	/**
	 * @brief Default-constructs with indeterminate component values.
	 */
	constexpr Normal() noexcept = default;

	/**
	 * @brief Constructs from a `Vec<Dim, T>`.
	 *
	 * @param normal The vector to initialize the normal from.
	 */
	constexpr Normal(Vec<Dim, T> const& normal) noexcept : Vec<Dim, T>(normal) {}

	/**
	 * @brief Constructs from exactly @p Dim scalar components.
	 *
	 * @param args Component values in order (x, y, [z, ...]).
	 */
	template <std::convertible_to<T>... Args>
	  requires(sizeof...(Args) == Dim)
	constexpr Normal(Args... args) noexcept : Vec<Dim, T>{static_cast<T>(args)...}
	{
	}
};

/**
 * @brief 2D single-precision surface normal.
 */
using Normal2f = Normal<2>;
/**
 * @brief 3D single-precision surface normal.
 */
using Normal3f = Normal<3>;
/**
 * @brief 4D single-precision surface normal.
 */
using Normal4f = Normal<4>;
/**
 * @brief 2D double-precision surface normal.
 */
using Normal2d = Normal<2, double>;
/**
 * @brief 3D double-precision surface normal.
 */
using Normal3d = Normal<3, double>;
/**
 * @brief 4D double-precision surface normal.
 */
using Normal4d = Normal<4, double>;

}  // namespace ufo

/**
 * @brief `std::format` / `std::formatter` specialization for `ufo::Normal<Dim, T>`.
 *
 * Delegates to `std::formatter<ufo::Vec<Dim, T>>`. See that specialization for the output
 * format. No format specifier is accepted; the format string must be empty (`{}`).
 *
 * @tparam Dim Dimensionality of the normal vector.
 * @tparam T   Scalar type for each component.
 */
template <std::size_t Dim, std::formattable<char> T>
struct std::formatter<ufo::Normal<Dim, T>> : std::formatter<ufo::Vec<Dim, T>> {
	auto format(ufo::Normal<Dim, T> const& n, std::format_context& ctx) const
	{
		// Explicitly qualify to avoid unqualified lookup resolving back to this
		// overload via the Normal(Vec const&) converting constructor.
		return std::formatter<ufo::Vec<Dim, T>>::format(n, ctx);
	}
};

namespace ufo
{
/**
 * @brief Writes a human-readable representation of @p n to @p os.
 *
 * @param os Output stream.
 * @param n Normal to print.
 * @return Reference to the output stream.
 */
template <std::size_t Dim, class T>
std::ostream& operator<<(std::ostream& os, Normal<Dim, T> const& n)
{
	return os << std::format("{}", n);
}
}  // namespace ufo

#endif  // UFO_CORE_NORMAL_HPP
