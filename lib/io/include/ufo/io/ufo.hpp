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

#ifndef UFO_CLOUD_IO_UFO_HPP
#define UFO_CLOUD_IO_UFO_HPP

// UFO
#include <ufo/cloud/cloud.hpp>
#include <ufo/execution/execution.hpp>
#include <ufo/io/cloud_properties.hpp>
#include <ufo/numeric/transform.hpp>
#include <ufo/numeric/vec.hpp>
#include <ufo/vision/color.hpp>

// STL
#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <ios>
#include <print>
#include <string>

namespace ufo
{
[[nodiscard]] CloudProperties cloudPropertiesUFO(std::filesystem::path const& file);

struct CloudUFOOptions {
	Transform3f pose;
	float       timestamp = 0.0f;
	bool        verbose   = false;
};

namespace detail
{
template <std::size_t Dim, class T>
bool readUFOData(std::istream& in, SoAView<Vec<Dim, T>> data, std::string const& type,
                 std::uint64_t size)
{
	if ("point" != type) {
		return false;
	} else if ((data.size() * sizeof(Vec3f)) != size) {
		// TODO: Error
		return false;
	}

	if constexpr (std::is_same_v<Vec3f, Vec<Dim, T>>) {
		in.read(reinterpret_cast<char*>(data.data()), data.size_bytes());
	} else {
		Vec3f tmp;
		for (auto& c : data) {
			in.read(reinterpret_cast<char*>(&tmp), sizeof(tmp));
			c = static_cast<Vec<Dim, T>>(tmp);
		}
	}

	return true;
}

template <ColorType CT, class T, bool Alpha, bool Weight>
bool readUFOData(std::istream& in, SoAView<Color<CT, T, Alpha, Weight>> data,
                 std::string const& type, std::uint64_t size)
{
	if ("color" != type) {
		return false;
	} else if ((data.size() * sizeof(SmallRGBA)) != size) {
		// TODO: Error
		return false;
	}

	if constexpr (std::is_same_v<SmallRGBA, Color<CT, T, Alpha, Weight>>) {
		in.read(reinterpret_cast<char*>(data.data()), data.size_bytes());
	} else {
		SmallRGBA tmp;
		for (auto& c : data) {
			in.read(reinterpret_cast<char*>(&tmp), sizeof(tmp));
			convert(tmp, c);
		}
	}

	return true;
}

template <class T>
bool readUFOData([[maybe_unused]] std::istream& in, [[maybe_unused]] SoAView<T> data,
                 [[maybe_unused]] std::string const& type,
                 [[maybe_unused]] std::uint64_t      size)
{
	return false;
}

template <std::size_t Dim, class T>
void writeUFOData(std::ostream& out, SoAView<Vec<Dim, T> const> data)
{
	out << "point: " << (data.size() * sizeof(Vec3f)) << '\n';

	if constexpr (std::is_same_v<Vec3f, Vec<Dim, T>>) {
		out.write(reinterpret_cast<char const*>(data.data()), data.size_bytes());
	} else {
		for (auto const& p : data) {
			Vec3f tmp(p);
			out.write(reinterpret_cast<char const*>(&tmp), sizeof(tmp));
		}
	}
}

template <ColorType CT, class T, bool Alpha, bool Weight>
void writeUFOData(std::ostream& out, SoAView<Color<CT, T, Alpha, Weight> const> data)
{
	out << "color: " << (data.size() * sizeof(SmallRGBA)) << '\n';

	if constexpr (std::is_same_v<SmallRGBA, Color<CT, T, Alpha, Weight>>) {
		out.write(reinterpret_cast<char const*>(data.data()), data.size_bytes());
	} else {
		SmallRGBA tmp;
		for (auto const& c : data) {
			convert(c, tmp);
			out.write(reinterpret_cast<char const*>(&tmp), sizeof(tmp));
		}
	}
}

template <class T>
void writeUFOData([[maybe_unused]] std::ostream&    out,
                  [[maybe_unused]] SoAView<T const> data)
{
}
}  // namespace detail

template <class... Ts>
inline bool readUFO(std::istream& in, Cloud<Ts...>& cloud)
{
	std::string line;

	// Header

	std::getline(in, line);
	if ("# UFO cloud file" != line) {
		return false;
	}

	while (std::getline(in, line) && in.good()) {
		if (line.starts_with('#')) {
			continue;  // Skip comments
		}

		std::string type  = line.substr(0, line.find(':'));
		std::string value = line.substr(line.find(':') + 1);

		if ("version" == type) {
			if (" 1.0" != value) {
				return false;
			}
		} else if ("width" == type) {
			std::size_t v = std::stoull(value);
			cloud.resize(v);
		} else if ("height" == type) {
			// TODO: Implement
		} else if ("pose" == type) {
			// TODO: Implement
		} else if ("timestamp" == type) {
			// TODO: Implement
		} else if ("data" == type) {
			break;  // End of header
		}
	}

	// Data

	std::string   type;
	std::uint64_t size;
	while (std::getline(in, line) && in.good()) {
		auto pos = line.find(':');

		type = line.substr(0, pos);

		auto [ptr, ec] =
		    std::from_chars(line.data() + pos + 2, line.data() + line.size(), size);

		if (ec == std::errc()) {
			// Good
		} else if (ec == std::errc::invalid_argument) {
			// TODO: Error
			continue;
		} else if (ec == std::errc::result_out_of_range) {
			// TODO: Error
			continue;
		}

		if ((detail::readUFOData(in, view<Ts>(cloud), type, size) || ...)) {
			continue;
		} else {
			in.seekg(size, std::ios::cur);
		}
	}

	return true;
}

template <class... Ts>
bool readUFO(std::filesystem::path const& file, Cloud<Ts...>& cloud)
{
	std::ifstream ifs;
	ifs.exceptions(std::ifstream::failbit | std::ifstream::badbit);
	ifs.imbue(std::locale());
	ifs.open(file, std::ios::in | std::ios::binary);
	return readUFO(ifs, cloud);
}

template <class... Ts>
bool writeUFO(std::ostream& out, Cloud<Ts...> const& cloud,
              CloudUFOOptions const& options = CloudUFOOptions())
{
	out << "# UFO cloud file\n";

	out << "# Generated by UFO\n";

	out << "version: 1.0\n";
	out << "width: " << cloud.size() << '\n';
	// TODO: Update
	out << "height: 1\n";
	out << "# pose: x y z qw qx qy qz\n";
	auto t = options.pose.translation;
	auto q = static_cast<Quatf>(options.pose.rotation);
	out << "pose: " << t.x << ' ' << t.y << ' ' << t.z << ' ' << q.w << ' ' << q.x << ' '
	    << q.y << ' ' << q.z << '\n';
	// TODO: Update
	out << "# timestamp: seconds nanoseconds\n";
	out << "timestamp: " << options.timestamp << '\n';
	out << "data:\n";

	(detail::writeUFOData(out, view<Ts>(cloud)), ...);

	return true;
}

template <class... Ts>
bool writeUFO(std::filesystem::path const& file, Cloud<Ts...> const& cloud,
              CloudUFOOptions const& options = CloudUFOOptions())
{
	std::ofstream ofs;
	ofs.exceptions(std::ofstream::failbit | std::ofstream::badbit);
	ofs.imbue(std::locale());
	ofs.open(file, std::ios::out | std::ios::binary);
	return writeUFO(ofs, cloud, options);
}
}  // namespace ufo

#endif  // UFO_CLOUD_IO_UFO_HPP