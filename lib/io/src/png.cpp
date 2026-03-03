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

// UFO
#include <ufo/io/png.hpp>

// STL
#include <format>
#include <memory>
#include <print>
#include <stdexcept>

// PNG
#include <spng.h>

namespace ufo
{
namespace detail
{
bool readPNG(std::FILE* fp, void* image, std::size_t length, int fmt)
{
	std::unique_ptr<spng_ctx, decltype(&spng_ctx_free)> ctx{spng_ctx_new(0), spng_ctx_free};
	if (!ctx) {
		std::println(stderr, "[UFO | Read PNG] Failed to create PNG context");
		return false;
	}

	/* Ignore and do not calculate chunk CRC's */
	spng_set_crc_action(ctx.get(), SPNG_CRC_USE, SPNG_CRC_USE);

	/* Set memory usage limits for storing standard and unknown chunks,
	   this is important when reading untrusted files! */
	constexpr std::size_t limit = 64uz * 1024 * 1024;
	spng_set_chunk_limits(ctx.get(), limit, limit);

	/* Set source PNG */
	spng_set_png_file(ctx.get(), fp);

	spng_ihdr ihdr;
	if (int const ret = spng_get_ihdr(ctx.get(), &ihdr); ret) {
		std::println(stderr, "[UFO | Read PNG] Get PNG IHDR error: {}", spng_strerror(ret));
		return false;
	}

	/* Calculate output image size */
	std::size_t image_size{};
	if (int const ret = spng_decoded_image_size(ctx.get(), fmt, &image_size); ret) {
		std::println(stderr, "[UFO | Read PNG] Get PNG image size error: {}",
		             spng_strerror(ret));
		return false;
	}

	if (image_size != length) {
		std::println(stderr, "[UFO | Read PNG] Incorrect image size, expected {} was {}",
		             length, image_size);
		return false;
	}

	int flags = 0;
	if (SPNG_FMT_GA8 == fmt || SPNG_FMT_GA16 == fmt || SPNG_FMT_RGBA8 == fmt ||
	    SPNG_FMT_RGBA16 == fmt) {
		flags |= SPNG_DECODE_TRNS;
	}

	/* Decode the image in one go */
	if (int const ret = spng_decode_image(ctx.get(), image, image_size, fmt, flags); ret) {
		std::println(stderr, "[UFO | Read PNG] Decode PNG image error: {}",
		             spng_strerror(ret));
		return false;
	}

	return true;
}

bool writePNG(std::FILE* fp, void const* image, std::size_t length, std::uint32_t width,
              std::uint32_t height, int color_type, int bit_depth)
{
	std::unique_ptr<spng_ctx, decltype(&spng_ctx_free)> ctx{spng_ctx_new(SPNG_CTX_ENCODER),
	                                                        spng_ctx_free};
	if (!ctx) {
		std::println(stderr, "[UFO | Write PNG] Failed to create PNG context");
		return false;
	}

	spng_set_png_file(ctx.get(), fp);

	spng_ihdr ihdr{};
	ihdr.width      = width;
	ihdr.height     = height;
	ihdr.bit_depth  = bit_depth;
	ihdr.color_type = color_type;

	spng_set_ihdr(ctx.get(), &ihdr);

	/* SPNG_FMT_PNG matches the format in ihdr; SPNG_ENCODE_FINALIZE writes the EOF marker
	 */
	if (int const ret =
	        spng_encode_image(ctx.get(), image, length, SPNG_FMT_PNG, SPNG_ENCODE_FINALIZE);
	    ret) {
		std::println(stderr, "[UFO | Write PNG] Encode PNG image error: {}",
		             spng_strerror(ret));
		return false;
	}

	return true;
}
}  // namespace detail

ImageProperties imagePropertiesPNG(std::filesystem::path const& file)
{
	FileHandler fp(file, "rb");

	if (!fp) {
		throw std::runtime_error(std::format(
		    "[UFO | Image Properties PNG] Failed to open file: {}", file.string()));
	}

	std::unique_ptr<spng_ctx, decltype(&spng_ctx_free)> ctx{spng_ctx_new(0), spng_ctx_free};
	if (!ctx) {
		throw std::runtime_error("[UFO | Image Properties PNG] Failed to create PNG context");
	}

	/* Ignore and do not calculate chunk CRC's */
	spng_set_crc_action(ctx.get(), SPNG_CRC_USE, SPNG_CRC_USE);

	/* Set memory usage limits for storing standard and unknown chunks,
	   this is important when reading untrusted files! */
	constexpr std::size_t limit = 64uz * 1024 * 1024;
	spng_set_chunk_limits(ctx.get(), limit, limit);

	/* Set source PNG */
	spng_set_png_file(ctx.get(), fp.get());

	spng_ihdr ihdr;
	if (int const ret = spng_get_ihdr(ctx.get(), &ihdr); ret) {
		throw std::runtime_error(std::format(
		    "[UFO | Image Properties PNG] Get PNG IHDR error: {}", spng_strerror(ret)));
	}

	return {
	    .width     = ihdr.width,
	    .height    = ihdr.height,
	    .bit_depth = ihdr.bit_depth,
	    .alpha     = SPNG_COLOR_TYPE_TRUECOLOR_ALPHA == ihdr.color_type ||
	             SPNG_COLOR_TYPE_GRAYSCALE_ALPHA == ihdr.color_type,
	    .grayscale = SPNG_COLOR_TYPE_GRAYSCALE == ihdr.color_type ||
	                 SPNG_COLOR_TYPE_GRAYSCALE_ALPHA == ihdr.color_type,
	};
}
}  // namespace ufo
