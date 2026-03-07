// UFO
#include <ufo/io/xyz.hpp>

// STL
#include <filesystem>

namespace ufo
{
CloudProperties cloudPropertiesXYZ(std::filesystem::path const& file)
{
	CloudProperties prop;
	prop.color     = false;
	prop.alpha     = false;
	prop.intensity = false;
	prop.normal    = false;

	return prop;
}
}  // namespace ufo