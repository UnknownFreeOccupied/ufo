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

#ifndef UFO_IO_FILE_TYPE_HPP
#define UFO_IO_FILE_TYPE_HPP

// STL
#include <filesystem>

namespace ufo
{
/**
 * @brief Enumerates all file formats supported by the UFO I/O subsystem.
 *
 * Used as a discriminator by format-dispatch routines to select the correct
 * reader / writer without inspecting the file extension more than once.
 *
 * | Enumerator | Extension(s)        | Description                                   |
 * |------------|---------------------|-----------------------------------------------|
 * | `UNKNOWN`  | —                   | Unrecognised or missing extension.            |
 * | `UFO`      | `.ufo`              | Native UFOMap binary format.                  |
 * | `XYZ`      | `.xyz`              | ASCII XYZ point cloud (x y z per line).       |
 * | `XYZI`     | `.xyzi`             | ASCII XYZ + intensity.                        |
 * | `XYZN`     | `.xyzn`             | ASCII XYZ + surface normal (nx ny nz).        |
 * | `XYZRGB`   | `.xyzrgb`           | ASCII XYZ + RGB colour (r g b).               |
 * | `PTS`      | `.pts`              | Leica PTX/PTS point cloud.                    |
 * | `PLY`      | `.ply`              | Stanford Triangle Format (polygon/point mesh).|
 * | `PCD`      | `.pcd`              | PCL Point Cloud Data format.                  |
 * | `OBJ`      | `.obj`              | Wavefront OBJ mesh/point format.              |
 * | `JPEG`     | `.jpeg`, `.jpg`     | JPEG compressed image.                        |
 * | `PNG`      | `.png`              | PNG lossless compressed image.                |
 * | `QTP`      | `.qtp`              | UFO quantised-tree point-cloud format.        |
 *
 * @see fileType()
 */
enum class FileType {
	UNKNOWN,  //!< Unrecognised or missing file extension.
	UFO,      //!< Native UFOMap binary format (`.ufo`).
	XYZ,      //!< ASCII XYZ point cloud (`.xyz`).
	XYZI,     //!< ASCII XYZ + intensity (`.xyzi`).
	XYZN,     //!< ASCII XYZ + surface normal (`.xyzn`).
	XYZRGB,   //!< ASCII XYZ + RGB colour (`.xyzrgb`).
	PTS,      //!< Leica PTS point cloud (`.pts`).
	PLY,      //!< Stanford PLY mesh / point cloud (`.ply`).
	PCD,      //!< PCL Point Cloud Data (`.pcd`).
	OBJ,      //!< Wavefront OBJ (`.obj`).
	JPEG,     //!< JPEG compressed image (`.jpeg`, `.jpg`).
	PNG,      //!< PNG lossless image (`.png`).
	QTP       //!< UFO quantised-tree point-cloud format (`.qtp`).
};

/**
 * @brief Infers the `FileType` from the extension of `file` (case-insensitive).
 *
 * The extension is extracted via `std::filesystem::path::extension()`, converted to
 * lowercase, and looked up in a static map of known extensions. Both `.jpeg` and
 * `.jpg` map to `FileType::JPEG`.
 *
 * @param file  Path whose extension determines the returned type.
 *
 * @return The matching `FileType`, or `FileType::UNKNOWN` if the extension is not
 *         recognised.
 */
[[nodiscard]] FileType fileType(std::filesystem::path const& file);
}  // namespace ufo

#endif  // UFO_IO_FILE_TYPE_HPP