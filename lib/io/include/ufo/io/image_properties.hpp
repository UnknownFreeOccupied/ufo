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

#ifndef UFO_IO_IMAGE_PROPERTIES_HPP
#define UFO_IO_IMAGE_PROPERTIES_HPP

// STL
#include <cstdint>

namespace ufo
{
/**
 * @brief Metadata describing the pixel layout of an image file.
 *
 * `ImageProperties` is returned by image I/O routines to convey the dimensions
 * and channel configuration of the decoded image, letting callers allocate an
 * appropriately sized `Image<T>` before reading pixel data.
 *
 * ### Fields
 * | Field       | Description                                                          |
 * |-------------|----------------------------------------------------------------------|
 * | `width`     | Pixel width of the image (number of columns).                        |
 * | `height`    | Pixel height of the image (number of rows).                          |
 * | `bit_depth` | Bits per channel sample (e.g., 8 for standard sRGB, 16 for HDR).    |
 * | `alpha`     | `true` if the image has a dedicated alpha (transparency) channel.    |
 * | `grayscale` | `true` if the image is single-channel (luminance only, no colour).   |
 */
struct ImageProperties {
	//! Pixel width (number of columns) of the image.
	std::uint32_t width;
	//! Pixel height (number of rows) of the image.
	std::uint32_t height;
	//! Bits per channel sample (e.g., 8 for uint8, 16 for uint16).
	std::uint32_t bit_depth;
	//! `true` if the image contains an alpha (transparency) channel.
	bool alpha;
	//! `true` if the image is single-channel grayscale (no RGB colour data).
	bool grayscale;
};
}  // namespace ufo

#endif  // UFO_IO_IMAGE_PROPERTIES_HPP