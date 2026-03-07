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

#ifndef UFO_CLOUD_IO_PCD_HPP
#define UFO_CLOUD_IO_PCD_HPP

// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/core/intensity.hpp>
#include <ufo/core/normal.hpp>
#include <ufo/io/cloud_properties.hpp>
#include <ufo/io/file_handler.hpp>
#include <ufo/numeric/transform3.hpp>
#include <ufo/numeric/vec.hpp>
#include <ufo/vision/color.hpp>

// STL
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <memory>
#include <print>
#include <sstream>

namespace ufo
{
namespace detail
{
void pcdSplit(std::vector<std::string>& result, std::string const& in,
              char const* const delimiters);
}

[[nodiscard]] CloudProperties cloudPropertiesPCD(std::filesystem::path const& file);

template <std::size_t Dim, class T, class... Ts>
bool readPCD(std::filesystem::path const& file, PointCloud<Dim, T, Ts...>& pc,
             ufo::Transform3f* viewpoint = nullptr)
{
	FileHandler fp(file.c_str(), "rb");

	if (!fp) {
		std::println(stderr, "[UFO | Read PCD] Failed to open file: {}", file.string());
		return false;
	}

	// Read header

	std::string version;

	std::vector<std::string> fields;
	std::vector<int>         field_size;
	std::vector<char>        field_type;
	std::vector<int>         field_count;
	std::vector<int>         field_offset;

	int width  = -1;
	int height = -1;
	int points = -1;

	int data_type = -1;

	std::vector<std::string> st;
	for (char* buf = fp.readline(); nullptr != buf; buf = fp.readline()) {
		std::string line(buf);

		if (line.empty()) {
			continue;
		}

		detail::pcdSplit(st, line, "\t\r ");

		std::stringstream sstream(line);
		sstream.imbue(std::locale::classic());

		std::string line_type;
		sstream >> line_type;

		if ("#" == line_type.substr(0, 1)) {
			continue;
		}

		if ("VERSION" == line_type.substr(0, 7)) {
			version = st.at(1);
		} else if (("FIELDS" == line_type.substr(0, 6)) ||
		           ("COLUMNS" == line_type.substr(0, 7))) {
			int specified_channel_count = static_cast<int>(st.size() - 1);

			fields.resize(specified_channel_count);
			for (int i = 0; i < specified_channel_count; ++i) {
				fields[i] = st.at(i + 1);
			}
		} else if ("SIZE" == line_type.substr(0, 4)) {
			if (fields.empty()) {
				std::println(stderr,
				             "[UFO | Read PCD] FIELDS must be specified before SIZE in header");
				return false;
			}

			int specified_channel_count = static_cast<int>(st.size() - 1);

			field_size.resize(specified_channel_count);

			int offset = 0;
			for (int i = 0; i < specified_channel_count; ++i) {
				sstream >> field_size[i];
			}
		} else if ("TYPE" == line_type.substr(0, 4)) {
			if (fields.empty() || field_size.empty()) {
				std::println(stderr,
				             "[UFO | Read PCD] FIELDS and SIZE must be specified before "
				             "TYPE in header");
				return false;
			}

			int specified_channel_count = static_cast<int>(st.size() - 1);

			field_type.resize(specified_channel_count);

			for (int i{}; specified_channel_count > i; ++i) {
				field_type[i] = st.at(i + 1).c_str()[0];
			}
		} else if ("COUNT" == line_type.substr(0, 5)) {
			if (fields.empty() || field_size.empty() || field_type.empty()) {
				std::println(stderr,
				             "[UFO | Read PCD] FIELDS, SIZE, and TYPE must be specified before "
				             "COUNT in header");
				return false;
			}

			int specified_channel_count = static_cast<int>(st.size() - 1);

			field_count.resize(specified_channel_count);

			int offset = 0;
			for (int i{}; specified_channel_count > i; ++i) {
				field_offset[i] = offset;
				sstream >> field_count[i];
				offset += field_count[i] * field_size[i];
			}
		} else if ("WIDTH" == line_type.substr(0, 5)) {
			sstream >> width;
			if (sstream.fail()) {
				std::println(stderr, "[UFO | Read PCD] Invalid WIDTH value specified");
				return false;
			}
		} else if ("HEIGHT" == line_type.substr(0, 6)) {
			sstream >> height;
			if (sstream.fail()) {
				std::println(stderr, "[UFO | Read PCD] Invalid HEIGHT value specified");
				return false;
			}
		} else if ("VIEWPOINT" == line_type.substr(0, 9)) {
			if (nullptr == viewpoint) {
				continue;
			}

			if (8 > st.size()) {
				std::println(stderr,
				             "[UFO | Read PCD] Not enough number of elements in VIEWPOINT. Need "
				             "7 values (tx ty tz qw qx qy qz)");
				return false;
			}

			sstream >> viewpoint->translation.x >> viewpoint->translation.y >>
			    viewpoint->translation.z;
			Quatf q;
			sstream >> q.w >> q.x >> q.y >> q.z;
			viewpoint->rotation = static_cast<Mat3x3f>(q);
		} else if ("POINTS" == line_type.substr(0, 6)) {
			if (field_count.empty()) {
				std::println(
				    stderr, "[UFO | Read PCD] Number of POINTS specified before COUNT in header");
				return false;
			}
			sstream >> points;
		} else if ("DATA" == line_type.substr(0, 4)) {
			if (st.at(1).substr(0, 17) == "binary_compressed") {
				data_type = 2;
			} else if (st.at(1).substr(0, 6) == "binary") {
				data_type = 1;
			} else if (st.at(1).substr(0, 5) == "ascii") {
				data_type = 0;
			} else {
				std::println(stderr, "[UFO | Read PCD] Unknown DATA format: {}", line);
				return false;
			}
			break;
		}
	}

	if (fields.empty() || fields.size() != field_size.size() ||
	    fields.size() != field_type.size() || fields.size() != field_count.size() ||
	    fields.size() != field_offset.size()) {
		std::println(stderr,
		             "[UFO | Read PCD] FIELDS, SIZE, TYPE, and COUNT must be specified in "
		             "header and have the same number of elements");
		return false;
	}

	if (0 > width || 0 > height || 0 > points) {
		std::println(stderr,
		             "[UFO | Read PCD] WIDTH, HEIGHT, and POINTS must be specified in header "
		             "and be positive integers");
		return false;
	}

	if (0 > data_type) {
		std::println(stderr,
		             "[UFO | Read PCD] DATA must be specified in header and has to be either "
		             "ascii, binary, or binary_compressed");
		return false;
	}

	// Read data

	if (0 == data_type) {
		// ASCII

		// TODO: Implement
	} else if (1 == data_type) {
		// Binary

		// TODO: Implement
	} else if (2 == data_type) {
		// Binary compressed
		std::println(stderr,
		             "[UFO | Read PCD] Does not support binary compressed PCD files yet");
		return false;
	} else {
		std::println(stderr, "[UFO | Read PCD] Unknown DATA type");
		return false;
	}

	return false;
}

template <std::size_t Dim, class T, class... Ts>
bool writePCD(std::filesystem::path const& file, PointCloud<Dim, T, Ts...> const& pc,
              bool ascii = false, ufo::Transform3f viewpoint = ufo::Transform3f())
{
	FileHandler fp(file.c_str(), "wb");

	if (!fp) {
		std::println(stderr, "[UFO | Write PCD] Failed to open file: {}", file.string());
		return false;
	}

	std::string version = ".7";

	std::vector<std::string> fields;
	std::vector<int>         field_size;
	std::vector<char>        field_type;
	std::vector<int>         field_count;

	fields.push_back("x");
	field_size.push_back(4);
	field_type.push_back('F');
	field_count.push_back(1);
	fields.push_back("y");
	field_size.push_back(4);
	field_type.push_back('F');
	field_count.push_back(1);
	fields.push_back("z");
	field_size.push_back(4);
	field_type.push_back('F');
	field_count.push_back(1);

	if constexpr ((is_color_v<Ts> || ...)) {
		fields.push_back(has_alpha_v<first_color_t<Ts...>> ? "rgba" : "rgb");
		field_size.push_back(4);
		field_type.push_back('U');
		field_count.push_back(1);
	}

	if constexpr ((is_intensity_v<Ts> || ...)) {
		fields.push_back("intensity");
		field_size.push_back(4);
		field_type.push_back('F');
		field_count.push_back(1);
	}

	if constexpr ((is_normal_v<Ts> || ...)) {
		fields.push_back("normal_x");
		field_size.push_back(4);
		field_type.push_back('F');
		field_count.push_back(1);
		fields.push_back("normal_y");
		field_size.push_back(4);
		field_type.push_back('F');
		field_count.push_back(1);
		fields.push_back("normal_z");
		field_size.push_back(4);
		field_type.push_back('F');
		field_count.push_back(1);
	}

	int width  = pc.size();
	int height = 1;
	int points = width * height;

	std::println(fp.get(), "# .PCD v{} - Point Cloud Data file format", version);
	std::println(fp.get(), "VERSION {}", version);

	std::print(fp.get(), "FIELDS");
	for (auto const& field : fields) {
		std::print(fp.get(), " {}", field);
	}
	std::print(fp.get(), "\n");

	std::print(fp.get(), "SIZE");
	for (auto const& size : field_size) {
		std::print(fp.get(), " {}", size);
	}
	std::print(fp.get(), "\n");

	std::print(fp.get(), "TYPE");
	for (auto const& type : field_type) {
		std::print(fp.get(), " {}", type);
	}
	std::print(fp.get(), "\n");

	std::print(fp.get(), "COUNT");
	for (auto const& count : field_count) {
		std::print(fp.get(), " {}", count);
	}
	std::print(fp.get(), "\n");

	std::println(fp.get(), "WIDTH {}", width);
	std::println(fp.get(), "HEIGHT {}", height);

	Vec3f t(viewpoint.translation);
	Quatf q(viewpoint.rotation);
	std::println(fp.get(), "VIEWPOINT {:.6} {:.6} {:.6} {:.6} {:.6} {:.6} {:.6}", t.x, t.y,
	             t.z, q.w, q.x, q.y, q.z);

	std::println(fp.get(), "POINTS {}", points);

	if (ascii) {
		std::println(fp.get(), "DATA ascii");

		for (std::size_t i{}; points > i; ++i) {
			Vec3f p = convert<Vec3f>(view<Vec<Dim, T>>(pc)[i]);
			std::print(fp.get(), "{:.6} {:.6} {:.6}", p.x, p.y, p.z);

			if constexpr ((is_color_v<Ts> || ...)) {
				std::array<std::uint8_t, 4> rgba;
				if constexpr (has_alpha_v<first_color_t<Ts...>>) {
					SmallRGBA c = convert<SmallRGBA>(view<first_color_t<Ts...>>(pc)[i]);
					rgba        = {c.blue, c.green, c.red, c.alpha};
				} else {
					SmallRGB c = convert<SmallRGB>(view<first_color_t<Ts...>>(pc)[i]);
					rgba       = {c.blue, c.green, c.red, 0};
				}
				std::uint32_t data;
				std::memcpy(&data, rgba.data(), 4);
				std::print(fp.get(), " {}", data);
			}

			if constexpr ((is_intensity_v<Ts> || ...)) {
				float intensity = static_cast<float>(view<Intensity>(pc)[i].intensity);
				std::print(fp.get(), " {:.6}", intensity);
			}

			if constexpr ((is_normal_v<Ts> || ...)) {
				Normal n = view<Normal>(pc)[i];
				std::print(fp.get(), " {:.6} {:.6} {:.6}", n.x, n.y, n.z);
			}

			std::println(fp.get(), "");
		}

		return true;
	} else {
		std::println(fp.get(), "DATA binary");

		std::unique_ptr<float[]> data(new float[fields.size()]);
		for (std::size_t i{}; points > i; ++i) {
			std::size_t j{};

			Vec3f p   = convert<Vec3f>(view<Vec<Dim, T>>(pc)[i]);
			data[j++] = p.x;
			data[j++] = p.y;
			data[j++] = p.z;

			if constexpr ((is_color_v<Ts> || ...)) {
				std::array<std::uint8_t, 4> rgba;
				if constexpr (has_alpha_v<first_color_t<Ts...>>) {
					SmallRGBA c = convert<SmallRGBA>(view<first_color_t<Ts...>>(pc)[i]);
					rgba        = {c.blue, c.green, c.red, c.alpha};
				} else {
					SmallRGB c = convert<SmallRGB>(view<first_color_t<Ts...>>(pc)[i]);
					rgba       = {c.blue, c.green, c.red, 0};
				}
				std::memcpy(&data[j++], rgba.data(), 4);
			}

			if constexpr ((is_intensity_v<Ts> || ...)) {
				data[j++] = static_cast<float>(view<Intensity>(pc)[i]);
			}

			if constexpr ((is_normal_v<Ts> || ...)) {
				Normal n  = view<Normal>(pc)[i];
				data[j++] = n.x;
				data[j++] = n.y;
				data[j++] = n.z;
			}

			std::fwrite(data.get(), sizeof(float), fields.size(), fp.get());
		}
	}

	return true;
}
}  // namespace ufo

#endif  // UFO_CLOUD_IO_PCD_HPP