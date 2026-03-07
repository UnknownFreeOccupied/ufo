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

#ifndef UFO_IO_PNG_HPP
#define UFO_IO_PNG_HPP

// UFO
#include <ufo/io/file_handler.hpp>
#include <ufo/io/image_properties.hpp>
#include <ufo/vision/color.hpp>
#include <ufo/vision/image.hpp>

// STL
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <print>
#include <type_traits>

namespace ufo
{
namespace detail
{
/**
 * @brief Decodes a PNG file into a pre-allocated pixel buffer.
 *
 * Reads from an already-opened C file handle and decodes the image in one
 * pass using libspng. The target format (number of channels, bit depth) is
 * selected by the caller via the SPNG format constant `fmt`.
 *
 * @param fp      Open `FILE*` positioned at the start of a PNG stream.
 * @param image   Destination buffer; must be at least `length` bytes.
 * @param length  Expected byte size of the decoded image.
 * @param fmt     SPNG pixel format constant (e.g. `SPNG_FMT_RGBA8`).
 *
 * @return `true` on success, `false` if any decoding step fails (with a
 *         diagnostic printed to `stderr`).
 */
bool readPNG(std::FILE* fp, void* image, std::size_t length, int fmt);

/**
 * @brief Encodes a pixel buffer as a PNG file.
 *
 * Writes to an already-opened C file handle using libspng. The IHDR metadata
 * (dimensions, bit depth, colour type) is taken from the remaining parameters
 * so the caller selects the exact PNG colour mode.
 *
 * @param fp          Open `FILE*` to write the PNG stream into.
 * @param image       Source pixel data; must be at least `length` bytes.
 * @param length      Byte size of the source pixel data.
 * @param width       Image width in pixels.
 * @param height      Image height in pixels.
 * @param color_type  SPNG colour type constant (e.g. `SPNG_COLOR_TYPE_TRUECOLOR`).
 * @param bit_depth   Bits per channel sample (8 or 16).
 *
 * @return `true` on success, `false` if encoding fails (with a diagnostic
 *         printed to `stderr`).
 */
bool writePNG(std::FILE* fp, void const* image, std::size_t length, std::uint32_t width,
              std::uint32_t height, int color_type, int bit_depth);
}  // namespace detail

/**
 * @brief Reads the metadata of a PNG file without decoding pixel data.
 *
 * Opens `file` and parses only the IHDR chunk to extract image dimensions,
 * bit depth, and channel configuration.
 *
 * @param file  Path to the PNG file.
 *
 * @return An `ImageProperties` struct describing the image layout.
 *
 * @throws std::runtime_error  If the file cannot be opened or the IHDR chunk
 *         cannot be parsed.
 */
[[nodiscard]] ImageProperties imagePropertiesPNG(std::filesystem::path const& file);

/**
 * @brief Reads a PNG file into an `Image` of the specified colour type.
 *
 * Detects the PNG's internal layout (grayscale / RGB, 8- / 16-bit, with or
 * without alpha) and decodes it into a temporary image of the closest matching
 * native colour type, then converts each pixel to `Color<CT, T, Alpha, Weight>`.
 *
 * Pixels with a zero alpha component have their colour channels zeroed out
 * before the conversion step (when the source format carries alpha).
 *
 * @tparam CT      Colour type of the destination image (e.g. `ColorType::RGB`).
 * @tparam T       Scalar channel type of the destination image (e.g. `std::uint8_t`).
 * @tparam Alpha   Whether the destination image has an alpha channel.
 * @tparam Weight  Whether the destination image has a weight channel.
 *
 * @param file   Path to the PNG file to read.
 * @param image  Output image; resized and filled with decoded pixel data.
 *
 * @return `true` on success, `false` if the file cannot be opened, the PNG
 *         format is unrecognised, or decoding fails.
 */
template <ColorType CT, class T, bool Alpha, bool Weight>
bool readPNG(std::filesystem::path const& file, Image<Color<CT, T, Alpha, Weight>>& image)
{
	FileHandler fp(file, "rb");

	if (!fp) {
		std::println(stderr, "[UFO | Read PNG] Failed to open file: {}", file.string());
		return false;
	}

	auto fun = [&image](std::FILE* fp, auto& tmp, int fmt) {
		using C = typename std::remove_cvref_t<decltype(tmp)>::value_type;
		if (detail::readPNG(fp, tmp.data(), sizeof(C) * tmp.size(), fmt)) {
			if constexpr (has_alpha_v<C>) {
				for (auto& p : tmp) {
					if (0 == alpha(p)) {
						if constexpr (ColorType::GRAY == color_type_v<C>) {
							p.gray = 0;
						} else {
							p.red   = 0;
							p.green = 0;
							p.blue  = 0;
						}
					}
				}
			}
			image = convert<Color<CT, T, Alpha, Weight>>(tmp);
			return true;
		}
		return false;
	};

	auto const prop = imagePropertiesPNG(file);
	if (prop.alpha && prop.grayscale && 8 >= prop.bit_depth) {
		int                                                      fmt = 16;  // SPNG_FMT_GA8
		Image<Color<ColorType::GRAY, std::uint8_t, true, false>> tmp(prop.height, prop.width);
		return fun(fp.get(), tmp, fmt);
	} else if (prop.grayscale && 16 == prop.bit_depth) {
		int                                                       fmt = 32;  // SPNG_FMT_GA16
		Image<Color<ColorType::GRAY, std::uint16_t, true, false>> tmp(prop.height,
		                                                              prop.width);
		return fun(fp.get(), tmp, fmt);
	} else if (prop.alpha && !prop.grayscale && 8 == prop.bit_depth) {
		int                                                     fmt = 1;  // SPNG_FMT_RGBA8
		Image<Color<ColorType::RGB, std::uint8_t, true, false>> tmp(prop.height, prop.width);
		return fun(fp.get(), tmp, fmt);
	} else if (!prop.grayscale && 16 == prop.bit_depth) {
		int                                                      fmt = 2;  // SPNG_FMT_RGBA16
		Image<Color<ColorType::RGB, std::uint16_t, true, false>> tmp(prop.height, prop.width);
		return fun(fp.get(), tmp, fmt);
	} else if (!prop.alpha && prop.grayscale && 8 >= prop.bit_depth) {
		int                                                       fmt = 64;  // SPNG_FMT_G8
		Image<Color<ColorType::GRAY, std::uint8_t, false, false>> tmp(prop.height,
		                                                              prop.width);
		return fun(fp.get(), tmp, fmt);
	} else if (!prop.alpha && !prop.grayscale && 8 == prop.bit_depth) {
		int                                                      fmt = 4;  // SPNG_FMT_RGB8
		Image<Color<ColorType::RGB, std::uint8_t, false, false>> tmp(prop.height, prop.width);
		return fun(fp.get(), tmp, fmt);
	} else {
		std::println(stderr, "[UFO | Read PNG] Unknown PNG type");
		return false;
	}
}

/**
 * @brief Writes an `Image` to a PNG file.
 *
 * Selects the appropriate SPNG colour type and bit depth from the compile-time
 * template parameters. If the image's colour type or scalar type differs from
 * the canonical PNG representation, pixels are converted before writing.
 *
 * @tparam CT      Colour type of the source image.
 * @tparam T       Scalar channel type of the source image.
 * @tparam Alpha   Whether the source image has an alpha channel.
 * @tparam Weight  Whether the source image has a weight channel (stripped on output).
 *
 * @param file   Destination path; the file is created or truncated.
 * @param image  Source image to encode.
 *
 * @return `true` on success, `false` if the file cannot be created or encoding
 *         fails.
 */
template <ColorType CT, class T, bool Alpha, bool Weight>
bool writePNG(std::filesystem::path const&              file,
              Image<Color<CT, T, Alpha, Weight>> const& image)
{
	FileHandler fp(file, "wb");

	if (!fp) {
		std::println(stderr, "[UFO | Write PNG] Failed to create file: {}", file.string());
		return false;
	}

	std::uint32_t const width     = image.cols();
	std::uint32_t const height    = image.rows();
	int const           bit_depth = sizeof(std::uint8_t) < sizeof(T) ? 16 : 8;

	// SPNG colour type and channel count derived entirely from template parameters.
	constexpr int color_type = (ColorType::GRAY == CT) ? (Alpha ? 4 : 0)   // GA or G
	                                                   : (Alpha ? 6 : 2);  // RGBA or RGB
	constexpr int channels   = (ColorType::GRAY == CT) ? (Alpha ? 2 : 1) : (Alpha ? 4 : 3);

	std::size_t const length =
	    static_cast<std::size_t>(width) * height * (bit_depth / 8) * channels;

	// Canonical output types: strip Weight, normalise scalar width.
	constexpr ColorType CT2 = (ColorType::GRAY == CT) ? ColorType::GRAY : ColorType::RGB;
	using T2 =
	    std::conditional_t<sizeof(std::uint8_t) < sizeof(T), std::uint16_t, std::uint8_t>;

	if constexpr (CT == CT2 && std::is_same_v<T, T2> && !Weight) {
		return detail::writePNG(fp.get(), image.data(), length, width, height, color_type,
		                        bit_depth);
	} else {
		Image<Color<CT2, T2, Alpha, false>> data(image.rows(), image.cols());
		std::ranges::transform(image, data.begin(), [](auto x) {
			return convert<Color<CT2, T2, Alpha, false>>(x);
		});
		return detail::writePNG(fp.get(), data.data(), length, width, height, color_type,
		                        bit_depth);
	}
}
}  // namespace ufo

#endif  // UFO_IO_PNG_HPP
