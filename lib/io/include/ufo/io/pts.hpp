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

#ifndef UFO_CLOUD_IO_PTS_HPP
#define UFO_CLOUD_IO_PTS_HPP

// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/core/intensity.hpp>
#include <ufo/io/cloud_properties.hpp>
#include <ufo/io/file_handler.hpp>
#include <ufo/numeric/vec.hpp>
#include <ufo/vision/color.hpp>

// STL
#include <cstdio>
#include <filesystem>
#include <print>

namespace ufo
{
[[nodiscard]] CloudProperties cloudPropertiesPTS(std::filesystem::path const& file);

template <std::size_t Dim, class T, class... Ts>
bool readPTS(std::filesystem::path const& file, PointCloud<Dim, T, Ts...>& pc)
{
	FileHandler fp(file.c_str(), "rb");

	if (!fp) {
		std::println(stderr, "[UFO | Read PTS] Failed to open file: {}", file.string());
		return false;
	}

	char        line[1024];
	std::size_t size{};
	if (nullptr != std::fgets(line, sizeof line, fp.get())) {
		std::sscanf(line, "%zu", &size);
	}

	if (0 == size) {
		std::println(stderr, "[UFO | Read PTS] Unable to read header");
		return false;
	}

	pc.clear();
	pc.resize(size);

	std::size_t           i{};
	std::array<double, 7> fields;
	do {
		auto num_fields =
		    std::sscanf(line, "%lf %lf %lf %lf %lf %lf %lf", &fields[0], &fields[1],
		                &fields[2], &fields[3], &fields[4], &fields[5], &fields[6]);

		if (3 != num_fields && 4 != num_fields && 6 != num_fields && 7 != num_fields) {
			std::println(stderr, "[UFO | Read PTS] Unknown format of line: {}", line);
			return false;
		}

		view<Vec<Dim, T>>(pc)[i] =
		    convert<Vec<Dim, T>>(Vec3d{fields[0], fields[1], fields[2]});

		if constexpr ((is_intensity_v<Ts> || ...)) {
			if (4 == num_fields || 7 == num_fields) {
				view<Intensity>(pc)[i] = fields[3];
			}
		}

		if constexpr ((is_color_v<Ts> || ...)) {
			if (6 == num_fields || 7 == num_fields) {
				view<first_color_t<Ts...>>(pc)[i] = convert<first_color_t<Ts...>>(
				    SmallRGB{static_cast<std::uint8_t>(fields[num_fields - 3]),
				             static_cast<std::uint8_t>(fields[num_fields - 2]),
				             static_cast<std::uint8_t>(fields[num_fields - 1])});
			}
		}

		++i;
	} while (nullptr != std::fgets(line, sizeof line, fp.get()));

	return true;
}

template <std::size_t Dim, class T, class... Ts>
bool writePTS(std::filesystem::path const& file, PointCloud<Dim, T, Ts...> const& pc)
{
	FileHandler fp(file.c_str(), "wb");

	if (!fp) {
		std::println(stderr, "[UFO | Write PTS] Failed to create file: {}", file.string());
		return false;
	}

	std::size_t size = pc.size();
	std::println(fp.get(), "{}", size);

	auto points = view<Vec<Dim, T>>(pc);
	for (std::size_t i{}; size > i; ++i) {
		auto p = convert<Vec<3, T>>(points[i]);
		if constexpr (std::is_same_v<T, float>) {
			std::print(fp.get(), "{:.6} {:.6} {:.6}", p.x, p.y, p.z);
		} else if constexpr (std::is_floating_point_v<T>) {
			std::print(fp.get(), "{:.10} {:.10} {:.10}", p.x, p.y, p.z);
		} else {
			std::print(fp.get(), "{} {} {}", p.x, p.y, p.z);
		}
		if constexpr ((is_intensity_v<Ts> || ...)) {
			auto intensity = view<Intensity>(pc)[i];
			std::print(fp.get(), "{:.6}", intensity.intensity);
		}
		if constexpr ((is_color_v<Ts> || ...)) {
			auto c = convert<SmallRGB>(view<first_color_t<Ts...>>(pc)[i]);
			std::println(fp.get(), " {} {} {}", static_cast<int>(c.red),
			             static_cast<int>(c.green), static_cast<int>(c.blue));
		} else {
			std::print(fp.get(), "\n");
		}
	}

	return true;
}
}  // namespace ufo

#endif  // UFO_CLOUD_IO_PTS_HPP