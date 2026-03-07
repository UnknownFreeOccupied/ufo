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

#ifndef UFO_CLOUD_IO_XYZRGB_HPP
#define UFO_CLOUD_IO_XYZRGB_HPP

// UFO
#include <ufo/cloud/point_cloud.hpp>
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
[[nodiscard]] CloudProperties cloudPropertiesXYZRGB(std::filesystem::path const& file);

template <std::size_t Dim, class T, class... Ts>
bool readXYZRGB(std::filesystem::path const& file, PointCloud<Dim, T, Ts...>& pc)
{
	FileHandler fp(file.c_str(), "rb");

	if (!fp) {
		std::println(stderr, "[UFO | Read XYZRGB] Failed to open file: {}", file.string());
		return false;
	}

	pc.clear();

	for (char line[1024]; nullptr != std::fgets(line, sizeof line, fp.get());) {
		Vec3d   p;
		FineRGB c;
		if (6 == std::sscanf(line, "%lf %lf %lf %f %f %f", &p.x, &p.y, &p.z, &c.red, &c.green,
		                     &c.blue)) {
			if constexpr (std::disjunction_v<is_color<Ts>...>) {
				pc.push_back(convert<Vec<Dim, T>>(p), convert<first_color_t<Ts...>>(c));
			} else {
				pc.push_back(convert<Vec<Dim, T>>(p));
			}
		}
	}

	return true;
}

template <std::size_t Dim, class T, class... Ts>
bool writeXYZRGB(std::filesystem::path const& file, PointCloud<Dim, T, Ts...> const& pc)
{
	FileHandler fp(file.c_str(), "wb");

	if (!fp) {
		std::println(stderr, "[UFO | Write XYZRGB] Failed to create file: {}", file.string());
		return false;
	}

	std::size_t const size   = pc.size();
	auto              points = view<Vec<Dim, T>>(pc);
	auto              colors = view<first_color_t<Ts...>>(pc);
	for (std::size_t i{}; size > i; ++i) {
		auto p = convert<Vec<3, T>>(points[i]);
		auto c = convert<FineRGB>(colors[i]);
		if constexpr (std::is_same_v<T, float>) {
			std::print(fp.get(), "{:.6} {:.6} {:.6}", p.x, p.y, p.z);
		} else if constexpr (std::is_floating_point_v<T>) {
			std::print(fp.get(), "{:.10} {:.10} {:.10}", p.x, p.y, p.z);
		} else {
			std::print(fp.get(), "{} {} {}", p.x, p.y, p.z);
		}
		if constexpr ((is_color_v<Ts> || ...)) {
			std::println(fp.get(), " {:.6} {:.6} {:.6}", c.red, c.green, c.blue);
		} else {
			std::println(fp.get(), " 0 0 0");
		}
	}

	return true;
}
}  // namespace ufo

#endif  // UFO_CLOUD_IO_XYZRGB_HPP