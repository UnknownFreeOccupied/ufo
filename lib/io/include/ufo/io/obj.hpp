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

#ifndef UFO_CLOUD_IO_OBJ_HPP
#define UFO_CLOUD_IO_OBJ_HPP

// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/core/normal.hpp>
#include <ufo/io/cloud_properties.hpp>
#include <ufo/io/file_handler.hpp>
#include <ufo/math/vec.hpp>
#include <ufo/vision/color.hpp>

// STL
#include <cstdio>
#include <filesystem>
#include <print>

namespace ufo
{
[[nodiscard]] CloudProperties cloudPropertiesOBJ(std::filesystem::path const& file);

template <std::size_t Dim, class T, class... Ts>
bool readOBJ(std::filesystem::path const& file, PointCloud<Dim, T, Ts...>& pc)
{
	FileHandler fp(file.c_str(), "rb");

	if (!fp) {
		std::println(stderr, "[UFO | Read OBJ] Failed to open file: {}", file.string());
		return false;
	}

	std::size_t num_points{};
	std::size_t num_normals{};
	for (char line[1024]; nullptr != std::fgets(line, sizeof line, fp.get());) {
		Vec3d   p;
		FineRGB c;
		Normal  n;
		if (auto num_fields = std::sscanf(line, "v %lf %lf %lf %f %f %f", &p.x, &p.y, &p.z,
		                                  &c.red, &c.green, &c.blue);
		    3 <= num_fields) {
			++num_points;

			if (4 == num_fields) {
				// TODO: Correct?
				p /= static_cast<double>(c.red);
			}

			if constexpr ((is_color_v<Ts> || ...)) {
				if (6 == num_fields) {
					if (pc.size() < num_points) {
						pc.push_back(convert<Vec<Dim, T>>(p), convert<first_color_t<Ts...>>(c));
					} else {
						view<Vec<Dim, T>>(pc)[num_points - 1] = convert<Vec<Dim, T>>(p);
						view<first_color_t<Ts...>>(pc)[num_points - 1] =
						    convert<first_color_t<Ts...>>(c);
					}
				}
			}

			if (3 == num_fields || 4 == num_fields) {
				if (pc.size() < num_points) {
					pc.push_back(convert<Vec<Dim, T>>(p));
				} else {
					view<Vec<Dim, T>>(pc)[num_points - 1] = convert<Vec<Dim, T>>(p);
				}
			} else {
				std::println(stderr, "[UFO | Read OBJ] Unknown format of line: {}", line);
				return false;
			}
		} else if constexpr ((is_normal_v<Ts> || ...)) {
			if (auto num_fields = std::sscanf(line, "vn %f %f %f", &n.x, &n.y, &n.z);
			    3 == num_fields) {
				++num_normals;
				if (pc.size() < num_normals) {
					pc.push_back(n);
				} else {
					view<Normal>(pc)[num_normals - 1] = n;
				}
			}
		}
	}

	return true;
}

template <std::size_t Dim, class T, class... Ts>
bool writeOBJ(std::filesystem::path const& file, PointCloud<Dim, T, Ts...> const& pc)
{
	FileHandler fp(file.c_str(), "wb");

	if (!fp) {
		std::println(stderr, "[UFO | Write OBJ] Failed to create file: {}", file.string());
		return false;
	}

	std::println(fp.get(), "Created by UFO");

	std::size_t size   = pc.size();
	auto        points = view<Vec<Dim, T>>(pc);
	for (std::size_t i{}; size > i; ++i) {
		auto p = convert<Vec<3, T>>(points[i]);
		if constexpr (std::is_same_v<T, float>) {
			std::print(fp.get(), "v {:.6} {:.6} {:.6}", p.x, p.y, p.z);
		} else if constexpr (std::is_floating_point_v<T>) {
			std::print(fp.get(), "v {:.10} {:.10} {:.10}", p.x, p.y, p.z);
		} else {
			std::print(fp.get(), "v {} {} {}", p.x, p.y, p.z);
		}
		if constexpr ((is_color_v<Ts> || ...)) {
			auto c = convert<FineRGB>(view<first_color_t<Ts...>>(pc)[i]);
			std::println(fp.get(), " {:.6} {:.6} {:.6}", c.red, c.green, c.blue);
		} else {
			std::print(fp.get(), "\n");
		}
		if constexpr ((is_normal_v<Ts> || ...)) {
			auto n = view<Normal>(pc)[i];
			std::println(fp.get(), "vn {:.6} {:.6} {:.6}", n.x, n.y, n.z);
		}
	}

	return true;
}
}  // namespace ufo

#endif  // UFO_CLOUD_IO_OBJ_HPP