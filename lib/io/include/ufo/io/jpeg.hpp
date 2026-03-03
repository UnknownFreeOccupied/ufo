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

#ifndef UFO_IO_JPEG_HPP
#define UFO_IO_JPEG_HPP

// UFO
#include <ufo/io/file_handler.hpp>
#include <ufo/io/image_properties.hpp>
#include <ufo/vision/color.hpp>
#include <ufo/vision/image.hpp>

// STL
#include <cstdio>
#include <filesystem>
#include <print>

namespace ufo
{
namespace detail
{
bool readJPEG(FileHandler fp, std::uint8_t* image);

bool writeJPEG(FileHandler& fp, std::uint8_t const* image, std::uint32_t width,
               std::uint32_t height, int num_channels, int quality);
}  // namespace detail

[[nodiscard]] ImageProperties imagePropertiesJPEG(std::filesystem::path const& file);

template <ColorType CT, class T, bool Alpha, bool Weight>
bool readJPEG(std::filesystem::path const&        file,
              Image<Color<CT, T, Alpha, Weight>>& image)
{
	FileHandler fp(file.c_str(), "rb");

	if (!fp) {
		std::println(stderr, "[UFO | Read JPEG] Failed to open file: {}", file.string());
		return false;
	}

	auto fun = [&image](FileHandler& fp, auto& tmp) {
		using C = typename std::remove_cvref_t<decltype(tmp)>::value_type;
		if (detail::readJPEG(fp, reinterpret_cast<std::uint8_t*>(tmp.data()))) {
			image = convert<Color<CT, T, Alpha, Weight>>(tmp);
			return true;
		}
		return false;
	};

	auto prop = imagePropertiesJPEG(file);
	if (prop.grayscale) {
		Image<Color<ColorType::GRAY, std::uint8_t, false, false>> tmp(prop.height,
		                                                              prop.width);
		return fun(fp, tmp);
	} else if (!prop.grayscale) {
		Image<Color<ColorType::RGB, std::uint8_t, false, false>> tmp(prop.height, prop.width);
		return fun(fp, tmp);
	} else {
		std::println(stderr, "[UFO | Read JPEG] Unknown JPEG type");
		return false;
	}
}

// Compression quality (0..100; 5-95 is most useful range,\n"
template <ColorType CT, class T, bool Alpha, bool Weight>
bool writeJPEG(std::filesystem::path const&              file,
               Image<Color<CT, T, Alpha, Weight>> const& image, int quality = 90)
{
	FileHandler fp(file.c_str(), "wb");

	if (!fp) {
		std::println(stderr, "[UFO | Write JPEG] Failed to create file: {}", file.string());
		return false;
	}

	std::uint32_t width        = image.cols();
	std::uint32_t height       = image.rows();
	int           num_channels = ColorType::GRAY == CT ? 1 : 3;

	constexpr ColorType const CT2 =
	    ColorType::GRAY == CT ? ColorType::GRAY : ColorType::RGB;

	if constexpr (CT == CT2 && std::is_same_v<T, std::uint8_t> && !Alpha && !Weight) {
		return detail::writeJPEG(fp, reinterpret_cast<std::uint8_t const*>(image.data()),
		                         width, height, num_channels, quality);
	} else {
		auto tmp = convert<Color<CT2, std::uint8_t, false, false>>(image);

		return detail::writeJPEG(fp, reinterpret_cast<std::uint8_t const*>(image.data()),
		                         width, height, num_channels, quality);
	}
}
}  // namespace ufo

#endif  // UFO_IO_JPEG_HPP