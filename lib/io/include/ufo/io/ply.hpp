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

#ifndef UFO_CLOUD_IO_PLY_HPP
#define UFO_CLOUD_IO_PLY_HPP

// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/core/intensity.hpp>
#include <ufo/core/normal.hpp>
#include <ufo/io/cloud_properties.hpp>
#include <ufo/io/file_handler.hpp>
#include <ufo/numeric/vec.hpp>
#include <ufo/vision/color.hpp>

// STL
#include <cstdio>
#include <filesystem>
#include <span>
#include <vector>

namespace ufo
{
namespace detail
{
bool readPLY(std::filesystem::path const& file, std::vector<Vec3f>& points,
             std::vector<SmallRGB>* colors, std::vector<SmallRGBA>* colors_with_alpha,
             std::vector<Normal>* normals, std::vector<Intensity>* intensities);

bool writePLY(std::filesystem::path const& file, std::span<Vec3f const> points,
              std::span<SmallRGB const>  colors,
              std::span<SmallRGBA const> colors_with_alpha,
              std::span<Normal const> normals, std::span<Intensity const> intensities,
              bool ascii);
}  // namespace detail

[[nodiscard]] CloudProperties cloudPropertiesPLY(std::filesystem::path const& file);

template <std::size_t Dim, class T, class... Ts>
bool readPLY(std::filesystem::path const& file, PointCloud<Dim, T, Ts...>& pc)
{
	std::vector<Vec3f>     points;
	std::vector<SmallRGB>  colors;
	std::vector<SmallRGBA> colors_with_alpha;
	std::vector<Normal>    normals;
	std::vector<Intensity> intensities;

	std::vector<SmallRGB>*  p_colors            = nullptr;
	std::vector<SmallRGBA>* p_colors_with_alpha = nullptr;
	std::vector<Normal>*    p_normals           = nullptr;
	std::vector<Intensity>* p_intensities       = nullptr;

	if constexpr ((is_color_v<Ts> || ...)) {
		if constexpr (has_alpha_v<first_color_t<Ts...>>) {
			p_colors_with_alpha = &colors_with_alpha;
		} else {
			p_colors = &colors;
		}
	}
	if constexpr ((is_normal_v<Ts> || ...)) {
		p_normals = &normals;
	}
	if constexpr ((is_intensity_v<Ts> || ...)) {
		p_intensities = &intensities;
	}

	if (!detail::readPLY(file, points, p_colors, p_colors_with_alpha, p_normals,
	                     p_intensities)) {
		return false;
	}

	pc.clear();
	pc.resize(points.size());

	for (std::size_t i{}; points.size() > i; ++i) {
		view<Vec<Dim, T>>(pc)[i] = convert<Vec<Dim, T>>(points[i]);
		if constexpr ((is_color_v<Ts> || ...)) {
			if constexpr (has_alpha_v<first_color_t<Ts...>>) {
				view<first_color_t<Ts...>>(pc)[i] =
				    convert<first_color_t<Ts...>>(colors_with_alpha[i]);
			} else {
				view<first_color_t<Ts...>>(pc)[i] = convert<first_color_t<Ts...>>(colors[i]);
			}
		}
		if constexpr ((is_normal_v<Ts> || ...)) {
			view<Normal>(pc)[i] = normals[i];
		}
		if constexpr ((is_intensity_v<Ts> || ...)) {
			view<Intensity>(pc)[i] = intensities[i];
		}
	}

	return true;
}

template <std::size_t Dim, class T, class... Ts>
bool writePLY(std::filesystem::path const& file, PointCloud<Dim, T, Ts...> const& pc,
              bool ascii = false)
{
	std::vector<Vec3f>     points;
	std::vector<SmallRGB>  colors;
	std::vector<SmallRGBA> colors_with_alpha;

	for (std::size_t i{}; pc.size() > i; ++i) {
		if constexpr (!std::is_same_v<Vec3f, Vec<Dim, T>>) {
			points.push_back(convert<Vec3f>(view<Vec<Dim, T>>(pc)[i]));
		}

		if constexpr ((is_color_v<Ts> || ...)) {
			if constexpr (has_alpha_v<first_color_t<Ts...>>) {
				if constexpr (!std::is_same_v<SmallRGBA, first_color_t<Ts...>>) {
					colors_with_alpha.push_back(
					    convert<SmallRGBA>(view<first_color_t<Ts...>>(pc)[i]));
				}
			} else {
				if constexpr (!std::is_same_v<SmallRGB, first_color_t<Ts...>>) {
					colors.push_back(convert<SmallRGB>(view<first_color_t<Ts...>>(pc)[i]));
				}
			}
		}
	}

	std::span<Vec3f const>     points_span;
	std::span<SmallRGB const>  colors_span;
	std::span<SmallRGBA const> colors_with_alpha_span;
	std::span<Normal const>    normals_span;
	std::span<Intensity const> intensities_span;

	if constexpr (std::is_same_v<Vec3f, Vec<Dim, T>>) {
		points_span = view<Vec3f>(pc);
	} else {
		points_span = std::span<Vec3f>(points);
	}

	if constexpr ((is_color_v<Ts> || ...)) {
		if constexpr (std::is_same_v<SmallRGBA, first_color_t<Ts...>>) {
			colors_with_alpha_span = view<SmallRGBA>(pc);
			colors_span            = std::span<SmallRGB>(colors);
		} else if constexpr (std::is_same_v<SmallRGB, first_color_t<Ts...>>) {
			colors_span            = view<SmallRGB>(pc);
			colors_with_alpha_span = std::span<SmallRGBA>(colors_with_alpha);
		} else {
			colors_span            = std::span<SmallRGB>(colors);
			colors_with_alpha_span = std::span<SmallRGBA>(colors_with_alpha);
		}
	}

	if constexpr ((is_normal_v<Ts> || ...)) {
		normals_span = view<Normal>(pc);
	}

	if constexpr ((is_intensity_v<Ts> || ...)) {
		intensities_span = view<Intensity>(pc);
	}

	return detail::writePLY(file, points_span, colors_span, colors_with_alpha_span,
	                        normals_span, intensities_span, ascii);
}
}  // namespace ufo

#endif  // UFO_CLOUD_IO_PLY_HPP