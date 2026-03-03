// UFO
#include <ufo/io/file_handler.hpp>
#include <ufo/io/obj.hpp>

// STL
#include <filesystem>
#include <format>
#include <stdexcept>

namespace ufo
{
CloudProperties cloudPropertiesOBJ(std::filesystem::path const& file)
{
	FileHandler fp(file.c_str(), "rb");

	if (!fp) {
		throw std::runtime_error(std::format(
		    "[UFO | Cloud Properties OBJ] Failed to open file: {}", file.string()));
	}

	CloudProperties prop;
	prop.alpha     = false;
	prop.intensity = false;

	for (char line[1024]; nullptr != std::fgets(line, sizeof line, fp.get());) {
		Vec3d   p;
		FineRGB c;
		Normal  n;
		if (3 == std::sscanf(line, "vn %f %f %f", &n.x, &n.y, &n.z)) {
			prop.normal = true;
		} else if (6 == std::sscanf(line, "v %lf %lf %lf %f %f %f", &p.x, &p.y, &p.z, &c.red,
		                            &c.green, &c.blue)) {
			prop.color = true;
		}

		if (prop.color && prop.normal) {
			break;
		}
	}

	return prop;
}
}  // namespace ufo