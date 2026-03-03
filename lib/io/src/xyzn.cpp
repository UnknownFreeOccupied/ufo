// UFO
#include <ufo/io/xyzn.hpp>

// STL
#include <filesystem>

namespace ufo
{
CloudProperties cloudPropertiesXYZN(std::filesystem::path const& file)
{
	CloudProperties prop;
	prop.color     = false;
	prop.alpha     = false;
	prop.intensity = false;
	prop.normal    = true;

	return prop;
}
}  // namespace ufo