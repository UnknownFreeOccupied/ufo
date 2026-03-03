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

#ifndef UFO_IO_QTP_HPP
#define UFO_IO_QTP_HPP

// UFO
#include <ufo/io/file_handler.hpp>
#include <ufo/io/image_properties.hpp>
#include <ufo/morton/morton.hpp>
#include <ufo/vision/color.hpp>
#include <ufo/vision/image.hpp>

// TODO: Remove
#include <ufo/io/jpeg.hpp>
#include <ufo/io/png.hpp>

// STL
#include <algorithm>
#include <bit>
#include <cstdio>
#include <filesystem>
#include <print>

namespace ufo
{
namespace detail
{
template <ColorType CT, bool Alpha>
struct QTPNode {
	using value_type = ColorFine<CT, Alpha, false>;

	std::array<std::unique_ptr<QTPNode>, 4> children{};
	value_type                              data{};

	[[nodiscard]] static std::uint32_t code(std::uint32_t x, std::uint32_t y)
	{
		return Morton<2>::encode32(x, y);
	}

	[[nodiscard]] static std::uint32_t numLevelsRequired(std::uint32_t x, std::uint32_t y)
	{
		// TODO: Correct?
		return (std::bit_width(code(x, y)) / 2) + 1;
	}

	void add(std::uint32_t code, value_type const& color, std::uint32_t level)
	{
		// data += color;

		if (0 == level) {
			data = color;
			return;
		}

		std::uint32_t node_index = (code >> ((level - 1) * 2)) & 0b11;

		if (!children[node_index]) {
			children[node_index] = std::make_unique<QTPNode>();
		}

		children[node_index]->add(code, color, level - 1);
	}

	[[nodiscard]] value_type operator()(std::uint32_t code, std::uint32_t level) const
	{
		if (0 == level) {
			return data;
		}

		std::uint32_t node_index = (code >> ((level - 1) * 2)) & 0b11;

		if (!children[node_index]) {
			return data;
		} else {
			return (*children[node_index])(code, level - 1);
		}
	}

	[[nodiscard]] bool isLeaf() const
	{
		return std::all_of(children.begin(), children.end(),
		                   [](auto const& child) { return nullptr == child; });
	}

	[[nodiscard]] bool isParent() const { return !isLeaf(); }

	[[nodiscard]] bool hasAllChildren() const
	{
		return std::all_of(children.begin(), children.end(),
		                   [](auto const& child) { return nullptr != child; });
	}

	void addChildData(std::vector<value_type>& data) const
	{
		if (isLeaf()) {
			data.push_back(this->data);
			return;
		}

		if (10000 < data.size()) {
			return;
		}

		for (auto const& child : children) {
			if (child) {
				child->addChildData(data);
			}
		}
	}

	[[nodiscard]] bool compressible(float threshold_sq)
	{
		// std::vector<value_type> child_colors;
		// addChildData(child_colors);

		// for (std::size_t i{}; child_colors.size() > i; ++i) {
		// 	for (std::size_t j{i + 1}; child_colors.size() > j; ++j) {
		// 		float de = deltaEOkSquared(child_colors[i], child_colors[j]);
		// 		// std::println("Delta E squared: {}", de);
		// 		if (threshold_sq < de) {
		// 			return false;
		// 		}
		// 	}
		// }

		// return true;

		if (!hasAllChildren()) {
			return false;
		}

		std::array<FineLab, 4> colors{};
		for (std::size_t i{}; 4 > i; ++i) {
			colors[i] = children[i]->data;
		}

		FineLab mean = average(colors);

		for (std::size_t i{}; colors.size() > i; ++i) {
			if (threshold_sq < deltaEEuclideanSquared(mean, colors[i])) {
				return false;
			}
		}

		data = mean;

		// for (std::size_t i{}; children.size() > i; ++i) {
		// 	for (std::size_t j{i + 1}; children.size() > j; ++j) {
		// 		if (threshold_sq < deltaEEuclideanSquared(children[i]->data, children[j]->data))
		// { 			return false;
		// 		}
		// 	}
		// }

		return true;
	}

	bool compress(float threshold_sq)
	{
		// if (isLeaf()) {
		// 	return;
		// }

		// // static std::size_t node_count = 0;
		// // ++node_count;
		// // std::println("Visiting node count: {}", node_count);

		// if (compressible(threshold_sq)) {
		// 	static std::size_t count = 0;
		// 	++count;
		// 	std::println("Compressing node count: {}", count);
		// 	for (auto& child : children) {
		// 		child.reset();
		// 	}
		// } else {
		// 	for (auto& child : children) {
		// 		if (child) {
		// 			child->compress(threshold_sq);
		// 		}
		// 	}
		// }

		if (isLeaf()) {
			return true;
		}

		// static std::size_t node_count = 0;
		// ++node_count;
		// std::println("Visiting node count: {}", node_count);

		bool all_children_compressed = true;
		for (auto& child : children) {
			if (child) {
				bool comp               = child->compress(threshold_sq);
				all_children_compressed = all_children_compressed && comp;
			}
		}

		if (all_children_compressed && compressible(threshold_sq)) {
			static std::size_t count = 0;
			++count;
			std::println("Compressing node count: {}", count);
			for (auto& child : children) {
				child.reset();
			}
			return true;
		}

		return false;
	}

	[[nodiscard]] std::uint32_t numLeaves() const
	{
		// if (isLeaf()) {
		// 	return 1;
		// }

		std::uint32_t count = 0;
		for (auto const& child : children) {
			if (child) {
				count += child->numLeaves();
			}
		}
		return 0 == count ? 1 : count;
	}
};
}  // namespace detail

[[nodiscard]] ImageProperties imagePropertiesQTP(std::filesystem::path const& file);

template <ColorType CT, class T, bool Alpha, bool Weight>
bool readQTP(std::filesystem::path const& file, Image<Color<CT, T, Alpha, Weight>>& image)
{
	FileHandler fp(file.c_str(), "rb");

	if (!fp) {
		std::println(stderr, "[UFO | Read QTP] Failed to open file: {}", file.string());
		return false;
	}

	// TODO: Implement

	return false;
}

// Compression quality (0..100; 5-95 is most useful range,\n"
template <ColorType CT, class T, bool Alpha, bool Weight>
bool writeQTP(std::filesystem::path const&              file,
              Image<Color<CT, T, Alpha, Weight>> const& image, int quality = 90)
{
	FileHandler fp(file.c_str(), "wb");

	if (!fp) {
		std::println(stderr, "[UFO | Write QTP] Failed to create file: {}", file.string());
		return false;
	}

	std::uint32_t width        = image.cols();
	std::uint32_t height       = image.rows();
	int           num_channels = ColorType::GRAY == CT ? 1 : 3;

	constexpr ColorType const CT2 =
	    ColorType::GRAY == CT ? ColorType::GRAY : ColorType::LAB;

	detail::QTPNode<CT2, Alpha> root;

	std::uint32_t num_levels = root.numLevelsRequired(height - 1, width - 1);

	if constexpr (ColorType::GRAY == CT) {
		// TODO: Implement
	} else {
		for (std::size_t i{}; height > i; ++i) {
			for (std::size_t j{}; width > j; ++j) {
				auto const& color     = image.at(i, j);
				auto        lab_color = convert<FineLab>(color);
				root.add(root.code(i, j), lab_color, num_levels);
			}
		}

		float threshold = 0.0025;

		root.compress(threshold * threshold);

		std::println("Number of leaves: {}\n", root.numLeaves());
		std::println("Original number of pixels: {}\n", width * height);

		Image<SmallRGB> new_image(height, width);
		for (std::size_t i{}; height > i; ++i) {
			for (std::size_t j{}; width > j; ++j) {
				new_image.at(i, j) = convert<SmallRGB>(root(root.code(i, j), num_levels));
			}
		}

		writePNG(file.string() + ".png", new_image);
		writeJPEG(file.string() + ".jpg", new_image);

		// TODO: Implement
	}

	return true;
}
}  // namespace ufo

#endif  // UFO_IO_QTP_HPP