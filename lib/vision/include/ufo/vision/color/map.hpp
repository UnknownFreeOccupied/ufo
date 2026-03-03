/**
 * UFOMap: An Efficient Probabilistic 3D Mapping Framework That Embraces the Unknown
 *
 * @author Daniel Duberg (dduberg@kth.se)
 * @see https://github.com/UnknownFreeOccupied/ufomap
 * @version 1.0
 * @date 2022-05-13
 *
 * @copyright Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, Daniel Duberg, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
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
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef UFO_VISION_COLOR_MAP_HPP
#define UFO_VISION_COLOR_MAP_HPP

// UFO
#include <ufo/vision/color/concepts.hpp>
#include <ufo/vision/color/map/black_body.hpp>
#include <ufo/vision/color/map/extended_kindlmann.hpp>
#include <ufo/vision/color/map/inferno.hpp>
#include <ufo/vision/color/map/kindlmann.hpp>

// STL
#include <algorithm>
#include <array>

namespace ufo
{
/**
 * @ingroup vision
 * @{
 */

/**
 * @brief Maps a value to a color using a color map.
 * @tparam C The color type.
 * @tparam N The size of the color map.
 * @param [in] map The color map.
 * @param [in] value The value to map.
 * @param [in] min_value The minimum value of the range.
 * @param [in] max_value The maximum value of the range.
 * @return The mapped color.
 */
template <Color C, std::size_t N>
[[nodiscard]] constexpr C colorMap(std::array<C, N> const& map, float value,
                                   float min_value, float max_value)
{
	float v = (value - min_value) / (max_value - min_value);
	v       = std::clamp(v, 0.0f, 1.0f);
	return map[static_cast<std::size_t>(v * (N - 1))];
}

/**
 * @}
 */
}  // namespace ufo

#endif  // UFO_VISION_COLOR_MAP_HPP