// UFO
#include <ufo/io/xyzi.hpp>

// STL
#include <filesystem>

namespace ufo
{
CloudProperties cloudPropertiesXYZI(std::filesystem::path const& file)
{
	CloudProperties prop;
	prop.color     = false;
	prop.alpha     = false;
	prop.intensity = true;
	prop.normal    = false;

	return prop;
}
}  // namespace ufo