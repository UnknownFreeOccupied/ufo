// UFO
#include <ufo/io/io.hpp>

// STL
#include <filesystem>
#include <format>
#include <stdexcept>

namespace ufo
{
// Cloud

CloudProperties cloudProperties(std::filesystem::path const& file)
{
	switch (fileType(file)) {
		case FileType::OBJ: return cloudPropertiesOBJ(file);
		case FileType::PCD: return cloudPropertiesPCD(file);
		case FileType::PLY: return cloudPropertiesPLY(file);
		case FileType::PTS: return cloudPropertiesPTS(file);
		case FileType::UFO: return cloudPropertiesUFO(file);
		case FileType::XYZ: return cloudPropertiesXYZ(file);
		case FileType::XYZI: return cloudPropertiesXYZI(file);
		case FileType::XYZN: return cloudPropertiesXYZN(file);
		case FileType::XYZRGB: return cloudPropertiesXYZRGB(file);
		case FileType::JPEG: [[fallthrough]];
		case FileType::PNG: [[fallthrough]];
		case FileType::UNKNOWN: break;
	}

	throw std::runtime_error(std::format(
	    "[UFO | Cloud Properties] Unknown point cloud file type: {}", file.c_str()));
}

// Image

ImageProperties imageProperties(std::filesystem::path const& file)
{
	switch (fileType(file)) {
		case FileType::JPEG: return imagePropertiesJPEG(file);
		case FileType::PNG: return imagePropertiesPNG(file);
		case FileType::OBJ: [[fallthrough]];
		case FileType::PCD: [[fallthrough]];
		case FileType::PLY: [[fallthrough]];
		case FileType::PTS: [[fallthrough]];
		case FileType::UFO: [[fallthrough]];
		case FileType::XYZ: [[fallthrough]];
		case FileType::XYZI: [[fallthrough]];
		case FileType::XYZN: [[fallthrough]];
		case FileType::XYZRGB: [[fallthrough]];
		case FileType::UNKNOWN: break;
	}

	throw std::runtime_error(
	    std::format("[UFO | Image Properties] Unknown image file type: {}", file.c_str()));
}
}  // namespace ufo