// UFO
#include <ufo/io/file_handler.hpp>
#include <ufo/io/ufo.hpp>

// STL
#include <filesystem>
#include <format>
#include <stdexcept>

namespace ufo
{
CloudProperties cloudPropertiesUFO(std::filesystem::path const& file)
{
	FileHandler fp(file.c_str(), "rb");

	if (!fp) {
		throw std::runtime_error(std::format(
		    "[UFO | Cloud Properties UFO] Failed to open file: {}", file.string()));
	}

	// TODO: Implement

	return {};
}
}  // namespace ufo