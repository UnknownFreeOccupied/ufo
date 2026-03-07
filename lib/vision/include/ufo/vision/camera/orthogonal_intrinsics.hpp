/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the
 * Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of
 * Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
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

#ifndef UFO_VISION_CAMERA_ORTHOGONAL_INTRINSICS_HPP
#define UFO_VISION_CAMERA_ORTHOGONAL_INTRINSICS_HPP

// STL
#include <cmath>
#include <cstddef>
#include <format>
#include <ostream>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

/**
 * @brief Stores the intrinsic parameters of an orthogonal camera.
 */
struct OrthogonalIntrinsics {
	/**
	 * @brief Image height in pixels.
	 */
	std::size_t rows{};
	/**
	 * @brief Image width in pixels.
	 */
	std::size_t cols{};
	/**
	 * @brief Viewport width in world-space units (e.g., meters).
	 */
	float width{};
	/**
	 * @brief Viewport height in world-space units (e.g., meters).
	 */
	float height{};
	/**
	 * @brief Zoom factor.
	 * @details Scales the projection matrix along the x and y axes. Default is 1.0.
	 */
	float zoom = 1.0f;

	/**
	 * @brief Computes the aspect ratio (cols / rows).
	 * @return The aspect ratio.
	 */
	[[nodiscard]] constexpr float aspect() const noexcept
	{
		return static_cast<float>(cols) / static_cast<float>(rows);
	}

	/**
	 * @brief Memberwise equality comparison.
	 */
	[[nodiscard]] friend bool operator==(OrthogonalIntrinsics const&,
	                                     OrthogonalIntrinsics const&) noexcept = default;
};

/**
 * @brief Writes a human-readable summary of `intrinsics` to `os`.
 */
inline std::ostream& operator<<(std::ostream& os, OrthogonalIntrinsics const& intrinsics)
{
	os << "Rows: " << intrinsics.rows << ", Cols: " << intrinsics.cols
	   << "\nWidth: " << intrinsics.width << ", Height: " << intrinsics.height
	   << "\nZoom: " << intrinsics.zoom;
	return os;
}

/**
 * @}
 */
}  // namespace ufo

template <>
struct std::formatter<ufo::OrthogonalIntrinsics> {
	constexpr auto parse(std::format_parse_context& ctx) const { return ctx.begin(); }

	auto format(ufo::OrthogonalIntrinsics const& intrinsics, std::format_context& ctx) const
	{
		return std::format_to(
		    ctx.out(), "Rows: {}, Cols: {}\nWidth: {}, Height: {}\nZoom: {}", intrinsics.rows,
		    intrinsics.cols, intrinsics.width, intrinsics.height, intrinsics.zoom);
	}
};

#endif  // UFO_VISION_CAMERA_ORTHOGONAL_INTRINSICS_HPP
