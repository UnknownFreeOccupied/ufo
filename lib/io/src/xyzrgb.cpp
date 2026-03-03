// UFO
#include <ufo/io/xyzrgb.hpp>

// STL
#include <filesystem>

namespace ufo
{
CloudProperties cloudPropertiesXYZRGB(std::filesystem::path const& file)
{
	CloudProperties prop;
	prop.color     = true;
	prop.alpha     = false;
	prop.intensity = false;
	prop.normal    = false;

	return prop;
}
}  // namespace ufo