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

#ifndef UFO_UTILITY_SCALAR_TYPE_HPP
#define UFO_UTILITY_SCALAR_TYPE_HPP

// STL
#include <concepts>
#include <cstdint>
#include <format>
#include <ostream>
#include <string_view>
#include <utility>

namespace ufo
{
/**
 * @brief Tag identifying the primitive scalar type of a data channel.
 *
 * @details
 * Used at runtime to describe the underlying C++ type without requiring
 * template parameters.  `UNKNOWN` is the sentinel for unrecognised types.
 */
enum class ScalarType : std::uint8_t {
	INT8    = 0,  ///< Signed 8-bit integer   (`std::int8_t`).
	UINT8   = 1,  ///< Unsigned 8-bit integer  (`std::uint8_t`).
	INT16   = 2,  ///< Signed 16-bit integer  (`std::int16_t`).
	UINT16  = 3,  ///< Unsigned 16-bit integer (`std::uint16_t`).
	INT32   = 4,  ///< Signed 32-bit integer  (`std::int32_t`).
	UINT32  = 5,  ///< Unsigned 32-bit integer (`std::uint32_t`).
	INT64   = 6,  ///< Signed 64-bit integer  (`std::int64_t`).
	UINT64  = 7,  ///< Unsigned 64-bit integer (`std::uint64_t`).
	FLOAT32 = 8,  ///< 32-bit IEEE 754 floating-point (`float`).
	FLOAT64 = 9,  ///< 64-bit IEEE 754 floating-point (`double`).
	UNKNOWN = 10  ///< Unrecognised or unsupported type.
};

/**
 * @brief Returns a human-readable string for a `ScalarType` value.
 *
 * @param type The scalar type to convert.
 * @return A null-terminated `std::string_view` with a stable lifetime.
 */
[[nodiscard]] constexpr std::string_view toString(ScalarType type) noexcept
{
	using enum ScalarType;
	switch (type) {
		case INT8: return "INT8";
		case UINT8: return "UINT8";
		case INT16: return "INT16";
		case UINT16: return "UINT16";
		case INT32: return "INT32";
		case UINT32: return "UINT32";
		case INT64: return "INT64";
		case UINT64: return "UINT64";
		case FLOAT32: return "FLOAT32";
		case FLOAT64: return "FLOAT64";
		case UNKNOWN: return "UNKNOWN";
	}
	std::unreachable();
}

/**
 * @brief Stream-insertion operator for `ScalarType`.
 */
inline std::ostream& operator<<(std::ostream& os, ScalarType type)
{
	return os << toString(type);
}

namespace detail
{
/**
 * @brief Compile-time mapping from type `T` to the corresponding `ScalarType`.
 *
 * @details
 * Returns `ScalarType::UNKNOWN` for any type that is not explicitly recognised.
 */
template <class T>
consteval ScalarType scalarTypeOf() noexcept
{
	using enum ScalarType;
	if constexpr (std::same_as<T, std::int8_t>) {
		return INT8;
	} else if constexpr (std::same_as<T, std::uint8_t>) {
		return UINT8;
	} else if constexpr (std::same_as<T, std::int16_t>) {
		return INT16;
	} else if constexpr (std::same_as<T, std::uint16_t>) {
		return UINT16;
	} else if constexpr (std::same_as<T, std::int32_t>) {
		return INT32;
	} else if constexpr (std::same_as<T, std::uint32_t>) {
		return UINT32;
	} else if constexpr (std::same_as<T, std::int64_t>) {
		return INT64;
	} else if constexpr (std::same_as<T, std::uint64_t>) {
		return UINT64;
	} else if constexpr (std::same_as<T, float>) {
		return FLOAT32;
	} else if constexpr (std::same_as<T, double>) {
		return FLOAT64;
	} else {
		return UNKNOWN;
	}
}
}  // namespace detail

/**
 * @brief The `ScalarType` enumerator for `T`, or `ScalarType::UNKNOWN` if
 * `T` is not a recognised scalar type.
 */
template <class T>
constexpr inline ScalarType scalar_type_v = detail::scalarTypeOf<T>();

}  // namespace ufo

template <>
struct std::formatter<ufo::ScalarType> : std::formatter<std::string_view> {
	auto format(ufo::ScalarType type, std::format_context& ctx) const
	{
		return std::formatter<std::string_view>::format(ufo::toString(type), ctx);
	}
};

#endif  // UFO_UTILITY_SCALAR_TYPE_HPP
