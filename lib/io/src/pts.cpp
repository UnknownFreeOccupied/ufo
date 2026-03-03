// UFO
#include <ufo/io/file_handler.hpp>
#include <ufo/io/pts.hpp>

// STL
#include <filesystem>
#include <format>
#include <stdexcept>

namespace ufo
{
CloudProperties cloudPropertiesPTS(std::filesystem::path const& file)
{
	FileHandler fp(file.c_str(), "rb");

	if (!fp) {
		throw std::runtime_error(std::format(
		    "[UFO | Cloud Properties PTS] Failed to open file: {}", file.string()));
	}

	char        line[1024];
	std::size_t size{};
	if (nullptr != std::fgets(line, sizeof line, fp.get())) {
		std::sscanf(line, "%zu", &size);
	}

	if (0 == size) {
		throw std::runtime_error(std::format(
		    "[UFO | Cloud Properties PTS] Unable to read header of file: {}", file.string()));
	}

	std::array<double, 7> fields;
	auto                  num_fields =
	    std::sscanf(line, "%lf %lf %lf %lf %lf %lf %lf", &fields[0], &fields[1], &fields[2],
	                &fields[3], &fields[4], &fields[5], &fields[6]);

	CloudProperties prop;
	prop.color     = 6 == num_fields || 7 == num_fields;
	prop.alpha     = false;
	prop.intensity = 4 == num_fields || 7 == num_fields;
	prop.normal    = false;

	return prop;
}
}  // namespace ufo