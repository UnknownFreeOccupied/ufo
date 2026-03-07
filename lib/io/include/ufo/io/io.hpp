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

#ifndef UFO_IO_HPP
#define UFO_IO_HPP

// UFO
#include <ufo/cloud/point_cloud.hpp>
#include <ufo/io/cloud_properties.hpp>
#include <ufo/io/file_type.hpp>
#include <ufo/io/image_properties.hpp>
#include <ufo/io/jpeg.hpp>
#include <ufo/io/obj.hpp>
#include <ufo/io/pcd.hpp>
#include <ufo/io/ply.hpp>
#include <ufo/io/png.hpp>
#include <ufo/io/pts.hpp>
#include <ufo/io/qtp.hpp>
#include <ufo/io/ufo.hpp>
#include <ufo/io/xyz.hpp>
#include <ufo/io/xyzi.hpp>
#include <ufo/io/xyzn.hpp>
#include <ufo/io/xyzrgb.hpp>
#include <ufo/vision/color.hpp>
#include <ufo/vision/image.hpp>

namespace ufo
{
// Cloud

[[nodiscard]] CloudProperties cloudProperties(std::filesystem::path const& file);

template <std::size_t Dim, class T, class... Ts>
bool read(std::filesystem::path const& file, PointCloud<Dim, T, Ts...>& pc)
{
	switch (fileType(file)) {
		case FileType::OBJ: return readOBJ(file, pc);
		case FileType::PCD: return readPCD(file, pc);
		case FileType::PLY: return readPLY(file, pc);
		case FileType::PTS: return readPTS(file, pc);
		case FileType::UFO: return readUFO(file, pc);
		case FileType::XYZ: return readXYZ(file, pc);
		case FileType::XYZI: return readXYZI(file, pc);
		case FileType::XYZN: return readXYZN(file, pc);
		case FileType::XYZRGB: return readXYZRGB(file, pc);
		case FileType::JPEG: [[fallthrough]];
		case FileType::PNG: [[fallthrough]];
		case FileType::QTP: [[fallthrough]];
		case FileType::UNKNOWN:
			std::println(stderr, "[UFO | Read] Unknown point cloud file type: {}",
			             file.c_str());
			return false;
	}
	return false;
}

template <std::size_t Dim, class T, class... Ts>
bool write(std::filesystem::path const& file, PointCloud<Dim, T, Ts...> const& pc)
{
	switch (fileType(file)) {
		case FileType::OBJ: return writeOBJ(file, pc);
		case FileType::PCD: return writePCD(file, pc);
		case FileType::PLY: return writePLY(file, pc);
		case FileType::PTS: return writePTS(file, pc);
		case FileType::UFO: return writeUFO(file, pc);
		case FileType::XYZ: return writeXYZ(file, pc);
		case FileType::XYZI: return writeXYZI(file, pc);
		case FileType::XYZN: return writeXYZN(file, pc);
		case FileType::XYZRGB: return writeXYZRGB(file, pc);
		case FileType::JPEG: [[fallthrough]];
		case FileType::PNG: [[fallthrough]];
		case FileType::QTP: [[fallthrough]];
		case FileType::UNKNOWN:
			std::println(stderr, "[UFO | Write] Unknown point cloud file type: {}",
			             file.c_str());
			return false;
	}
	return false;
}

// Image

[[nodiscard]] ImageProperties imageProperties(std::filesystem::path const& file);

template <class T>
bool read(std::filesystem::path const& file, Image<T>& image)
{
	switch (fileType(file)) {
		case FileType::JPEG: return readJPEG(file, image);
		case FileType::PNG: return readPNG(file, image);
		case FileType::QTP: return readQTP(file, image);
		case FileType::OBJ: [[fallthrough]];
		case FileType::PCD: [[fallthrough]];
		case FileType::PLY: [[fallthrough]];
		case FileType::PTS: [[fallthrough]];
		case FileType::UFO: [[fallthrough]];
		case FileType::XYZ: [[fallthrough]];
		case FileType::XYZI: [[fallthrough]];
		case FileType::XYZN: [[fallthrough]];
		case FileType::XYZRGB: [[fallthrough]];
		case FileType::UNKNOWN:
			std::println(stderr, "[UFO | Read] Unknown image file type: {}", file.c_str());
			return false;
	}
	return false;
}

template <class T>
bool write(std::filesystem::path const& file, Image<T> const& image)
{
	switch (fileType(file)) {
		case FileType::JPEG: return writeJPEG(file, image);
		case FileType::PNG: return writePNG(file, image);
		case FileType::QTP: return writeQTP(file, image);
		case FileType::OBJ: [[fallthrough]];
		case FileType::PCD: [[fallthrough]];
		case FileType::PLY: [[fallthrough]];
		case FileType::PTS: [[fallthrough]];
		case FileType::UFO: [[fallthrough]];
		case FileType::XYZ: [[fallthrough]];
		case FileType::XYZI: [[fallthrough]];
		case FileType::XYZN: [[fallthrough]];
		case FileType::XYZRGB: [[fallthrough]];
		case FileType::UNKNOWN:
			std::println(stderr, "[UFO | Write] Unknown image file type: {}", file.c_str());
			return false;
	}
	return false;
}
}  // namespace ufo

#endif  // UFO_IO_HPP