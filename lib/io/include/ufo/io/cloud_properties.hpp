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

#ifndef UFO_IO_CLOUD_PROPERTIES_HPP
#define UFO_IO_CLOUD_PROPERTIES_HPP

namespace ufo
{
/**
 * @brief Flags describing which optional per-point channels are present in a cloud
 * file.
 *
 * `CloudProperties` is passed to I/O routines (read / write) to declare or query
 * which optional data channels accompany the XYZ position data. Each flag maps to
 * a distinct SoA channel in `Cloud` / `PointCloud`.
 *
 * ### Channels
 * | Field       | Type       | Description                                      |
 * |-------------|------------|--------------------------------------------------|
 * | `color`     | `Color`    | RGB colour per point (red, green, blue).         |
 * | `alpha`     | `Color`    | Alpha (opacity) component of the colour channel. |
 * | `intensity` | `Intensity`| Scalar LiDAR return intensity.                   |
 * | `normal`    | `Normal`   | Surface normal vector.                           |
 *
 * @note `alpha` is only meaningful when `color` is also `true`; the two flags
 *       are read and written together as an RGBA colour channel.
 */
struct CloudProperties {
	//! `true` if the cloud carries per-point RGB colour data.
	bool color;
	//! `true` if the colour channel includes an alpha (opacity) component.
	bool alpha;
	//! `true` if the cloud carries per-point scalar LiDAR return intensity.
	bool intensity;
	//! `true` if the cloud carries per-point surface normal vectors.
	bool normal;
};
}  // namespace ufo

#endif  // UFO_IO_CLOUD_PROPERTIES_HPP